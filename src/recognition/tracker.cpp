#include <recognition/tracker.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <chrono>

namespace pacv
{
//Constructor
Tracker::Tracker(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor)
    :Module<Tracker>(n,ns,stor), started(false), lost_it(false),
    error_count(0), centroid_counter(0), disturbance_counter(0), rej_distance(0.03)
{
    config=std::make_shared<TrackerConfig>();
}

void
Tracker::setBasicNodeConfig(BasicConfig::Ptr config)
{
    basic_config = config;
}

void
Tracker::getObjList(std::list<std::string> &list) const
{
    if(est_names){
        list.clear();
        for (const auto& x: *est_names)
        {
            list.push_back(x.first);
        }
    }
}

void
Tracker::init()
{
    if(!nh){
        ROS_ERROR("[Tracker::%s]\tNode Handle not initialized, Module must call spawn() first!",__func__);
        return;
    }
    index = -1;
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
    centroid_counter = error_count = disturbance_counter = 0;
    rej_distance = 0.03;
    factor = 1.1;
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
    centroid_counter = error_count = disturbance_counter = 0;
    //this frees memory when module is killed
    marks.reset();
    scene.reset();
    model.reset();
    orig_model.reset();
    bounding_box.reset();
    crd.reset();
    teDQ.reset();
    est_names.reset();
    model_feat.reset();
    model_normals.reset();
}

TrackerConfig::Ptr
Tracker::getConfig() const
{
    return config;
}
//tracker step
void Tracker::track()
{
    storage->readSceneProcessed(scene);
    if (error_count >= 30){
        //failed 30 times in a row
        ROS_ERROR("[Tracker::%s] Object is lost ... stopping tracker...",__func__);
        started = false;
        lost_it = true;
        return;
    }
    PXC::Ptr target (new PXC);
    crop_a_box(scene, target, (*bounding_box)*factor, false, *transform, false);
    if (target->points.size() <= 15){
        ROS_ERROR_THROTTLE(10,"[Tracker::%s] Not enought points in bounding box, retryng with larger bounding box", __func__);
        factor += 0.2;
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
    if (val != leaf){
      leaf = val;
      computeModel();
      icp.setInputSource(model);
      pcl::CentroidPoint<PX> mc;
      for (size_t i=0; i<model->points.size(); ++i)
          mc.add(model->points[i]);
      mc.get(model_centroid);
    }
    // bool feat_align(true);
    // if (feat_align){
    //     NTC::Ptr target_n = boost::make_shared<NTC>();
    //     pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_f = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    //     ne.setRadiusSearch(2.0f*leaf);
    //     ne.useSensorOriginAsViewPoint();
    //     ne.setInputCloud(target);
    //     ne.compute(*target_n);
    //     fpfh.setInputNormals(target_n);
    //     fpfh.setInputCloud(target);
    //     fpfh.setRadiusSearch(3.5f*leaf);
    //     fpfh.compute(*target_f);
    //     //Assemble correspondences based on model-target features
    //     SearchT tree (true, CreatorT(new IndexT(4)));
    //     tree.setPointRepresentation (RepT(new pcl::DefaultFeatureRepresentation<pcl::FPFHSignature33>));
    //     tree.setChecks(256);
    //     tree.setInputCloud(target_f);
    //     //Search model features over target features
    //     //If model features are n, these will be n*k_nn matrices
    //     std::vector<std::vector<int>> k_idx;
    //     std::vector<std::vector<float>> k_dist;
    //     int k_nn(1);
    //     tree.nearestKSearch (*model_feat, std::vector<int> (), k_nn, k_idx, k_dist);
    //     //define a distance threshold
    //     float dist_thresh_m = 125.0f;
    //     //fill in model-target correpsondences
    //     pcl::Correspondences corr_m_over_t;
    //     for(size_t i=0; i < k_idx.size(); ++i)
    //     {
    //         if (k_dist[i][0] > dist_thresh_m){
    //             //we have a correspondence
    //             PX p1 (model->points[i]);
    //             PX p2 (target->points[k_idx[i][0]]);
    //             Eigen::Vector3f diff (p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
    //             float eu_dist = diff.squaredNorm();
    //             //Add a correspondence only if distance is below threshold
    //             pcl::Correspondence cor(i, k_idx[i][0], eu_dist);
    //             corr_m_over_t.push_back(cor);
    //         }
    //     }
    //     //Estimate the rigid transformation of model -> target
    //     teDQ->estimateRigidTransformation(*model, *target, corr_m_over_t, *transform);
    // }
    crd->setMaximumDistance(rej_distance);
    icp.setInputTarget(target);
    if (centroid_counter >=10){
        pcl::CentroidPoint<PX> tc;
        for (size_t i=0; i<target->points.size(); ++i)
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
        centroid_counter = 0;
    }
    else if (disturbance_counter >= 20)
    {
        float angx = D2R*UniformRealIn(30.0,90.0,true);
        float angy = D2R*UniformRealIn(30.0,90.0,true);
        float angz = D2R*UniformRealIn(30.0,90.0,true);
        Eigen::Matrix4f T_rotx, T_roty, T_rotz;
        if (UniformIntIn(0,1))
            angx *= -1;
        if (UniformIntIn(0,1))
            angy *= -1;
        if (UniformIntIn(0,1))
            angz *= -1;
        Eigen::AngleAxisf rotx (angx, Eigen::Vector3f::UnitX());
        T_rotx<< rotx.matrix()(0,0), rotx.matrix()(0,1), rotx.matrix()(0,2), 0,
                rotx.matrix()(1,0), rotx.matrix()(1,1), rotx.matrix()(1,2), 0,
                rotx.matrix()(2,0), rotx.matrix()(2,1), rotx.matrix()(2,2), 0,
                0,                0,                  0,                 1;
        Eigen::AngleAxisf roty (angy, Eigen::Vector3f::UnitY());
        T_roty<< roty.matrix()(0,0), roty.matrix()(0,1), roty.matrix()(0,2), 0,
                roty.matrix()(1,0), roty.matrix()(1,1), roty.matrix()(1,2), 0,
                roty.matrix()(2,0), roty.matrix()(2,1), roty.matrix()(2,2), 0,
                0,                0,                  0,                 1;
        Eigen::AngleAxisf rotz (angz, Eigen::Vector3f::UnitZ());
        T_rotz<< rotz.matrix()(0,0), rotz.matrix()(0,1), rotz.matrix()(0,2), 0,
                rotz.matrix()(1,0), rotz.matrix()(1,1), rotz.matrix()(1,2), 0,
                rotz.matrix()(2,0), rotz.matrix()(2,1), rotz.matrix()(2,2), 0,
                0,                0,                  0,                 1;
        Eigen::Matrix4f disturbed, inverse;
        inverse = transform->inverse();
        disturbed = (T_rotz*T_roty*T_rotx*inverse).inverse();
        ROS_WARN("[Tracker::%s] Triggered Disturbance! With angles %g, %g, %g",__func__,  angx*R2D, angy*R2D, angz*R2D);
        icp.align(*aligned, disturbed);
        disturbance_counter = 0;
    }
    else
        icp.align(*aligned, *transform);
    fitness = icp.getFitnessScore();
    // ROS_WARN("Fitness %g", fitness);
    *(transform) = icp.getFinalTransformation();
    //adjust distance and factor according to fitness
    if (fitness > 0.001 ){
        //fitness is high something is prolly wrong
        rej_distance +=0.001;
        factor += 0.05;
        if (rej_distance > 2.0)
            rej_distance = 2.0;
        if (factor > 5.0)
            factor = 5.0;
        ++disturbance_counter;
        ++centroid_counter;
    }
    else if (fitness < 0.0006){
        //all looks good
        rej_distance -=0.005;
        if(rej_distance < 0.015)
            rej_distance = 0.015; //we dont want to go lower than this
        factor -=0.05;
        if(factor < 1.1)
            factor = 1.1;
        error_count = 0;
        disturbance_counter = 0;
        centroid_counter = 0;
    }
}

void
Tracker::create_markers()
{
    std::string ref_frame;
    storage->readSensorFrame(ref_frame);
    marks = std::make_shared<visualization_msgs::MarkerArray>();
    geometry_msgs::Pose pose;
    fromEigen(*transform, pose);
    visualization_msgs::Marker obj_mark;
    create_object_marker(pose, obj_name, obj_mark);
    obj_mark.header.frame_id = ref_frame.c_str();
    obj_mark.color.r = 1.0f;
    obj_mark.color.g = 0.0f;
    obj_mark.color.b = 0.5f;
    obj_mark.ns = obj_name.first;
    obj_mark.id = 0;
    marks->markers.push_back(obj_mark);
}

void
Tracker::create_bb_marker(geometry_msgs::Pose pose)
{
    if (!marks)
        return;
    visualization_msgs::Marker bb_mark;
    std::string ref_frame;
    storage->readSensorFrame(ref_frame);
    create_box_marker((*bounding_box)*factor, bb_mark, false);
    bb_mark.header.frame_id = ref_frame.c_str();
    bb_mark.color.r = 0.8f;
    bb_mark.color.b = 0.8f;
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
    if(transform){
        bool val;
        config->get("publish_bounding_box", val);
        fromEigen(*transform, pose);
        marks->markers.resize(1);
        if (val)
            create_bb_marker(pose);
        for (auto& x: marks->markers)
            x.pose = pose;
    }
}
void
Tracker::publish_markers()
{
    bool val;
    config->get("publish_markers", val);
    if (val && marks && pub_markers.getNumSubscribers()>0){
        pub_markers.publish(*marks);
    }
}

void
Tracker::broadcast_tf()
{
    bool val;
    config->get("broadcast_tf", val);
    if (!val || !started)
        return;
    std::string frame;
    storage->readSensorFrame(frame);
    tf::Transform tran;
    if(transform){
        fromEigen(*transform, tran);
        tf_brc.sendTransform(tf::StampedTransform(tran, ros::Time::now(), frame.c_str(), obj_name.first.c_str()));
    }
}

void
Tracker::spinOnce()
{
    if (started){
        auto begin_time=std::chrono::high_resolution_clock::now();
        track();
        update_markers();
        publish_markers();
        broadcast_tf();
        auto end_time=std::chrono::high_resolution_clock::now();
        auto elapsed_time=std::chrono::duration_cast<std::chrono::milliseconds>(end_time - begin_time).count();
        ROS_INFO_THROTTLE(10,"[Tracker::%s]\tStep time: %ld ms.",__func__,elapsed_time);
    }
    else if (lost_it){
        find_object_in_scene();
    }
    else{
        if(!storage->readObjNames(est_names))
            ROS_WARN_THROTTLE(30,"[Tracker::%s]\tLooks like no Pose Estimation has been performed, perform one in order to start using the object tracker.",__func__);
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
    if (!storage->searchObjName(req.name, index)){
        ROS_ERROR("[Tracker::%s] Cannot find %s from the pool of already estimated objects, check spelling or run a Pose Estimation first!", __func__, req.name.c_str());
        return false;
    }
    obj_name.first = (req.name);
    std::vector<std::string> vst;
    boost::split(vst, req.name, boost::is_any_of("_"), boost::token_compress_on);
    obj_name.second = vst.at(0);
    if (!storage->readObjTransformByIndex(index, transform)){
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
    computeModel();
    //Get the minimum and maximum values on each of the 3 (x-y-z) dimensions of model
    //also get model centroid
    pcl::CentroidPoint<PX> mc;
    std::vector<float> xvec,yvec,zvec;
    for (size_t i=0; i<model->points.size(); ++i)
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

    storage->writeTrackedIndex(index);
    //we are ready to start
    started = true;
    create_markers();
    return true;
}

void
Tracker::computeModel()
{
    vg.setInputCloud (orig_model);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter (*model);
    //Get model features
    // model_normals = boost::make_shared<NTC>();
    // ne.setRadiusSearch(2.0f*leaf);
    // ne.useSensorOriginAsViewPoint();
    // ne.setInputCloud(model);
    // ne.compute(*model_normals);
    // model_feat = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    // fpfh.setInputNormals(model_normals);
    // fpfh.setInputCloud(model);
    // fpfh.setRadiusSearch(3.5f*leaf);
    // fpfh.compute(*model_feat);
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
    storage->writeObjTransformByIndex(index, transform);
    index = -1;
    storage->writeTrackedIndex(index);
    marks.reset();
    est_names.reset();
    factor = 1.1;
    rej_distance = 0.03;
    return true;
}


bool Tracker::cb_grasp(pacman_vision_comm::grasp_verification::Request& req, pacman_vision_comm::grasp_verification::Response& res)
{
  //TODO
  return true;
}

void
Tracker::find_object_in_scene()
{
    storage->readSceneProcessed(scene);
    if (scene->points.size() > model->points.size()/3)
    {
        pcl::CentroidPoint<PX> tc;
        for (size_t i=0; i<scene->points.size(); ++i)
            tc.add(scene->points[i]);
        PX target_centroid, mc_transformed;
        mc_transformed = pcl::transformPoint(model_centroid, Eigen::Affine3f(*transform));
        tc.get(target_centroid);
        Eigen::Matrix4f Tcen, guess;
        Tcen << 1, 0, 0,  (target_centroid.x - mc_transformed.x),
             0, 1, 0,  (target_centroid.y - mc_transformed.y),
             0, 0, 1,  (target_centroid.z - mc_transformed.z),
             0, 0, 0,  1;
        guess = Tcen*(*transform);
        *transform = guess;
        this->started = true;
        this->lost_it = false;
        this->error_count = 0;
        this->centroid_counter = 0;
        ROS_INFO("[Tracker::%s]\tFound something that could be the object, trying to track that",__func__);
        return;
    }
    else
    {
        ROS_WARN_THROTTLE(30, "[Tracker::%s]\tNothing is found on scene yet...",__func__);
        return;
    }
}
}

