#include <recognition/tracker.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

namespace pacv
{
//Constructor
Tracker::Tracker(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor)
    :Module<Tracker>(n,ns,stor), started(false), lost_it(false),
    error_count(0), centroid_counter(0)
{
    config=std::make_shared<TrackerConfig>();
}

void
Tracker::setBasicNodeConfig(BasicConfig::Ptr config)
{
    basic_config = config;
}

void
Tracker::init()
{
    if(!nh){
        ROS_ERROR("[Tracker::%s]\tNode Handle not initialized, Module must call spawn() first!",__func__);
        return;
    }
    srv_track_object = nh->advertiseService("track_object", &Tracker::cb_track_object, this);
    srv_stop = nh->advertiseService("stop_track", &Tracker::cb_stop_tracker, this);
    srv_grasp = nh->advertiseService("grasp_verification", &Tracker::cb_grasp, this);
    std::string mark_topic(getFatherNamespace()+"/markers");
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>(mark_topic, 1);
    scene=boost::make_shared<PXC>();
    model=boost::make_shared<PXC>();
    orig_model=boost::make_shared<PXC>();
    bounding_box = std::make_shared<Box>();
    crd =boost::make_shared<pcl::registration::CorrespondenceRejectorDistance>();
    teDQ =boost::make_shared<pcl::registration::TransformationEstimationDualQuaternion<PX,PX,float>>();
    //init node params
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh->getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
}

void Tracker::deInit()
{
    started = lost_it = false;
    centroid_counter = error_count = 0;
    //this frees memory when module is killed
    marks.reset();
    scene.reset();
    model.reset();
    orig_model.reset();
    bounding_box.reset();
    crd.reset();
    teDQ.reset();
}

TrackerConfig::Ptr
Tracker::getConfig() const
{
    return config;
}
//tracker step
void Tracker::track()
{
    storage->read_scene_processed(scene);
    if (error_count >= 30)
    {
        //failed 30 times in a row
        ROS_ERROR("[Tracker::%s] Object is lost ... stopping tracker...",__func__);
        started = false;
        lost_it = true;
        return;
    }
    // Eigen::Matrix4f inv_trans;
    PXC::Ptr target (new PXC);
    // inv_trans = transform->inverse();
    //transform scene in object ref frame.
    crop_a_box(scene, target, (*bounding_box)*factor, false, *transform, false);
    if (target->points.size() <= 20)
    {
        ROS_ERROR("[Tracker::%s] Not enought points in bounding box, retryng with larger bounding box", __func__);
        factor += 0.5;
        rej_distance +=0.005;
        ++error_count;
        return;
    }
    /*
     *  Alignment
     */
    //check if user changed leaf size
    double val;
    PXC::Ptr aligned = boost::make_shared<PXC>();
    basic_config->get("downsampling_leaf_size", val);
    if (val != leaf)
    {
      leaf = val;
      vg.setInputCloud (orig_model);
      vg.setLeafSize(leaf, leaf, leaf);
      vg.filter (*model);
      icp.setInputSource(model);
      pcl::CentroidPoint<PX> mc;
      for (int i=0; i<model->points.size(); ++i)
          mc.add(model->points[i]);
      mc.get(model_centroid);
    }
    crd->setMaximumDistance(rej_distance);
    icp.setInputTarget(target);
    if (centroid_counter >=5)
    {
        pcl::CentroidPoint<PX> tc;
        for (int i=0; i<target->points.size(); ++i)
            tc.add(target->points[i]);
        PX target_centroid, mc_transformed;
        mc_transformed = pcl::transformPoint(model_centroid, Eigen::Affine3f(*transform));
        tc.get(target_centroid);
        Eigen::Matrix4f Tcen, guess;
        Tcen << 1, 0, 0,  (target_centroid.x - mc_transformed.x),
                0, 1, 0,  (target_centroid.y - mc_transformed.y),
                0, 0, 1,  (target_centroid.z - mc_transformed.z),
                0, 0, 0,  1;
        guess = Tcen*(*transform);
        icp.align(*aligned, guess);
        centroid_counter = 0;
        ROS_WARN("[Tracker::%s] Centroid Translation Performed!",__func__);
        if (icp.getFitnessScore() < 0.001 )
        {
            fitness = icp.getFitnessScore();
            *(this->transform) = icp.getFinalTransformation();
        }
    }
    // else if (disturbance_counter >= 10 || manual_disturbance)
    // {
    //     boost::random::mt19937 gen(std::time(0));
    //     boost::random::uniform_int_distribution<> angle(20,90);
    //     float angx,angy,angz;
    //     if (angle(gen)%2)
    //         angx = -D2R*angle(gen);
    //     else
    //         angx = D2R*angle(gen);
    //     Eigen::AngleAxisf rotx (angx, Eigen::Vector3f::UnitX());
    //     T_rotx << rotx.matrix()(0,0), rotx.matrix()(0,1), rotx.matrix()(0,2), 0,
    //            rotx.matrix()(1,0), rotx.matrix()(1,1), rotx.matrix()(1,2), 0,
    //            rotx.matrix()(2,0), rotx.matrix()(2,1), rotx.matrix()(2,2), 0,
    //            0,                0,                  0,                 1;
    //     if (angle(gen)%2)
    //         angz = -D2R*angle(gen);
    //     else
    //         angz = D2R*angle(gen);
    //     Eigen::AngleAxisf rotz (angz, Eigen::Vector3f::UnitZ());
    //     T_rotz << rotz.matrix()(0,0), rotz.matrix()(0,1), rotz.matrix()(0,2), 0,
    //            rotz.matrix()(1,0), rotz.matrix()(1,1), rotz.matrix()(1,2), 0,
    //            rotz.matrix()(2,0), rotz.matrix()(2,1), rotz.matrix()(2,2), 0,
    //            0,                0,                  0,                 1;
    //     #<{(|
    //        if (angle(gen)%2)
    //        angy = -D2R*angle(gen);
    //        else
    //        angy = D2R*angle(gen);
    //        Eigen::AngleAxisf roty (angy, Eigen::Vector3f::UnitY());
    //        T_roty << roty.matrix()(0,0), roty.matrix()(0,1), roty.matrix()(0,2), 0,
    //        roty.matrix()(1,0), roty.matrix()(1,1), roty.matrix()(1,2), 0,
    //        roty.matrix()(2,0), roty.matrix()(2,1), roty.matrix()(2,2), 0,
    //        0,                0,                  0,                 1;
    //        |)}>#
    //     Eigen::Matrix4f disturbed;
    //     disturbed = (T_rotz*T_rotx*inv_trans).inverse();
    //     if (disturbance_done >5 || manual_disturbance)
    //     {
    //         manual_disturbance = false;
    //         ROS_WARN("[Tracker::%s] Triggered Heavy Disturbance!",__func__);
    //         Eigen::AngleAxisf rothz (3.14159/2, Eigen::Vector3f::UnitZ());
    //         Eigen::AngleAxisf rothx (3.14159/2, Eigen::Vector3f::UnitX());
    //         Eigen::AngleAxisf rothy (3.14159/2, Eigen::Vector3f::UnitY());
    //         Eigen::Matrix4f Tx,Tz,Ty;
    //         if (angle(gen)%2)
    //         {
    //             Tx << rothx.matrix()(0,0), rothx.matrix()(0,1), rothx.matrix()(0,2), 0,
    //                rothx.matrix()(1,0), rothx.matrix()(1,1), rothx.matrix()(1,2), 0,
    //                rothx.matrix()(2,0), rothx.matrix()(2,1), rothx.matrix()(2,2), 0,
    //                0,                0,                  0,                 1;
    //             disturbed = (Tx*inv_trans).inverse();
    //         }
    //         else if (angle(gen)%3)
    //         {
    //             Ty << rothy.matrix()(0,0), rothy.matrix()(0,1), rothy.matrix()(0,2), 0,
    //                rothy.matrix()(1,0), rothy.matrix()(1,1), rothy.matrix()(1,2), 0,
    //                rothy.matrix()(2,0), rothy.matrix()(2,1), rothy.matrix()(2,2), 0,
    //                0,                0,                  0,                 1;
    //             disturbed = (Ty*inv_trans).inverse();
    //         }
    //         else
    //         {
    //             Tz << rothz.matrix()(0,0), rothz.matrix()(0,1), rothz.matrix()(0,2), 0,
    //                rothz.matrix()(1,0), rothz.matrix()(1,1), rothz.matrix()(1,2), 0,
    //                rothz.matrix()(2,0), rothz.matrix()(2,1), rothz.matrix()(2,2), 0,
    //                0,                0,                  0,                 1;
    //             disturbed = (Tz*inv_trans).inverse();
    //         }
    //         disturbance_done = 0;
    //     }
    //     else
    //         ROS_WARN("[Tracker::%s] Triggered Disturbance! With angles %g, %g",__func__,  angx/D2R, angz/D2R);
    //     icp.align(*tmp, disturbed);
    //     ++disturbance_done;
    //     disturbance_counter = 0;
    //     if (icp.getFitnessScore() < 0.001 )
    //     {
    //          fitness = icp.getFitnessScore();
    //          *(this->transform) = icp.getFinalTransformation();
    //     }
    //  }
    else{
        icp.align(*aligned, *transform);
        fitness = icp.getFitnessScore();
        *(transform) = icp.getFinalTransformation();
    }
    this->storage->write_obj_transform_by_index(index, this->transform);
    //adjust distance and factor according to fitness
    if (fitness > 0.0008 ) //something is probably wrong
    {
        rej_distance +=0.001;
        factor += 0.05;
        if (rej_distance > 2.0)
            rej_distance = 2.0;
        if (factor > 5.0)
            factor = 5.0;
    // ++disturbance_counter;
        ++centroid_counter;
        return;
    }
    else if (fitness < 0.0005) //all looks good, lower factors and distances
    {
        rej_distance -=0.005;
        if(rej_distance < 0.025)
            rej_distance = 0.025; //we dont want to go lower than this
        factor -=0.05;
        if(factor < 1.3)
            factor = 1.3;
    }
    error_count = 0;
  // disturbance_counter = 0;
    centroid_counter = 0;
  // disturbance_done = 0;
}

void
Tracker::create_markers()
{
    std::string ref_frame;
    storage->read_sensor_ref_frame(ref_frame);
    marks = std::make_shared<visualization_msgs::MarkerArray>();
    geometry_msgs::Pose pose;
    fromEigen(*transform, pose);
    visualization_msgs::Marker obj_mark;
    create_object_marker(pose, obj_name, obj_mark);
    obj_mark.header.frame_id = ref_frame.c_str();
    obj_mark.color.r = 0.0f;
    obj_mark.color.g = 1.0f;
    obj_mark.color.b = 0.0f;
    marks->markers.push_back(obj_mark);
    bool val;
    config->get("publish_bounding_box", val);
    if (val)
        create_bb_marker(pose);
}

void
Tracker::create_bb_marker(geometry_msgs::Pose pose)
{
    if (!marks)
        return;
    visualization_msgs::Marker bb_mark;
    std::string ref_frame;
    storage->read_sensor_ref_frame(ref_frame);
    create_box_marker((*bounding_box)*factor, bb_mark, false);
    bb_mark.header.frame_id = ref_frame.c_str();
    bb_mark.color.r = 1.0f;
    bb_mark.color.b = 0.7f;
    bb_mark.color.g = 0.0f;
    bb_mark.pose = pose;
    bb_mark.ns = "Tracked Object Bounding Box";
    marks->markers.push_back(bb_mark);
}

void
Tracker::update_markers()
{
    if (!marks)
        return;
    geometry_msgs::Pose pose;
    fromEigen(*transform, pose);
    for (auto& x: marks->markers)
        x.pose = pose;
    bool val;
    config->get("publish_bounding_box", val);
    if (val && marks->markers.size() < 2)
        create_bb_marker(pose);
    if (!val && marks->markers.size() > 1)
        marks->markers.resize(1);
}
void
Tracker::publish_markers()
{
    if (marks && pub_markers.getNumSubscribers()>0){
        pub_markers.publish(*marks);
    }
}

void
Tracker::spinOnce()
{
    if (started){
        track();
        update_markers();
        publish_markers();
    }
    queue_ptr->callAvailable(ros::WallDuration(0));
}

bool Tracker::cb_track_object(pacman_vision_comm::track_object::Request& req, pacman_vision_comm::track_object::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Tracker::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (req.name.empty()){
        ROS_ERROR("[Tracker::%s] You need to provide the name of the object you want to track!", __func__);
        return false;
    }
    std::string models_path (ros::package::getPath("asus_scanner_models"));
    if (storage->search_obj_name(req.name, index)){
        ROS_ERROR("[Tracker::%s] Cannot find %s from the pool of already estimated objects, check spelling or run a Pose Estimation first!", __func__, req.name.c_str());
        return false;
    }
    obj_name.first = (req.name);
    std::vector<std::string> vst;
    boost::split(vst, req.name, boost::is_any_of("_"), boost::token_compress_on);
    obj_name.second = vst.at(0);
    if (!this->storage->read_obj_transform_by_index(index, transform)){
        ROS_ERROR("[Tracker::%s] Cannot find %s transform from the pool of already estimated transforms, check spelling or run an estimation first!", __func__, req.name.c_str());
        return false;
    }
    boost::filesystem::path model_path (models_path + "/" + obj_name.second + "/" + obj_name.second + ".pcd");
    if (boost::filesystem::exists(model_path) && boost::filesystem::is_regular_file(model_path))
    {
        if (pcl::io::loadPCDFile(model_path.c_str(), *orig_model))
        {
            ROS_ERROR("[Tracker::%s] Error loading model %s",__func__, model_path.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("[Tracker::%s] Requested model (%s) does not exists in asus_scanner_models package",__func__, model_path.stem().c_str());
        return false;
    }
    basic_config->get("downsampling_leaf_size", leaf);
    vg.setInputCloud (orig_model);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter (*model);
    //Get the minimum and maximum values on each of the 3 (x-y-z) dimensions of model
    //also get model centroid
    pcl::CentroidPoint<PX> mc;
    std::vector<float> xvec,yvec,zvec;
    for (int i=0; i<model->points.size(); ++i)
    {
        xvec.push_back(model->points[i].x);
        yvec.push_back(model->points[i].y);
        zvec.push_back(model->points[i].z);
        mc.add(model->points[i]);
    }
    bounding_box->x1 = *std::min_element(xvec.begin(), xvec.end());
    bounding_box->y1 = *std::min_element(yvec.begin(), yvec.end());
    bounding_box->z1 = *std::min_element(zvec.begin(), zvec.end());
    bounding_box->x2 = *std::max_element(xvec.begin(), xvec.end());
    bounding_box->y2 = *std::max_element(yvec.begin(), yvec.end());
    bounding_box->z2 = *std::max_element(zvec.begin(), zvec.end());
    mc.get(model_centroid);

    //init icps
    icp.setUseReciprocalCorrespondences(false);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-9);
    icp.setEuclideanFitnessEpsilon(1e-9);
    //Correspondence Rejector(s)
    crd->setMaximumDistance(rej_distance);
    icp.addCorrespondenceRejector(crd);
    icp.setInputSource(model);
    //Transformation Estimation
    icp.setTransformationEstimation(teDQ);

    storage->write_tracked_index(index);
    //we are ready to start
    started = true;
    return true;
}

bool Tracker::cb_stop_tracker(pacman_vision_comm::stop_track::Request& req, pacman_vision_comm::stop_track::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Tracker::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    started = false;
    lost_it = false;
    storage->write_obj_transform_by_index(index, transform);
    index = -1;
    storage->write_tracked_index(index);
    marks.reset();
    return true;
}


bool Tracker::cb_grasp(pacman_vision_comm::grasp_verification::Request& req, pacman_vision_comm::grasp_verification::Response& res)
{
  //TODO
  return true;
}
}
