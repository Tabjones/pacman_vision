#include <recognition/estimator.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>

namespace pacv
{
//Constructor
Estimator::Estimator(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor)
    :Module<Estimator>(n,ns,stor)
{
    scene=boost::make_shared<PXC>();
    config=std::make_shared<EstimatorConfig>();
    db_path = (ros::package::getPath("pacman_vision") + "/database" );
    if (!boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path))
        ROS_WARN("[Estimator][%s] Database for pose estimation does not exists!! Plese put one in /database folder, before trying to perform a pose estimation.",__func__);
    srv_estimate = nh.advertiseService("estimate", &Estimator::cb_estimate, this);
    std::string mark_topic("/"+getFatherNamespace()+"/markers");
    ROS_WARN("%s  TODO REMOVE ME!!",mark_topic.c_str());
    pub_markers = nh.advertise<visualization_msgs::Marker>(mark_topic, 1);
    //tmp set params to dump into default
    // nh.setParam("object_calibration", false);
    // nh.setParam("iterations", 5);
    // nh.setParam("neighbors", 20);
    // nh.setParam("cluster_tol", 0.05);
    ////////////////////////////////////////////////////////
    init();
}

void
Estimator::init()
{
    //init node params
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh.getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
    pe.setParam("verbosity",1);
    pe.setRMSEThreshold(0.003);
    config->get("iterations", iter);
    pe.setStepIterations(iter);
    config->get("neighbors", neigh);
    pe.setParam("lists_size",neigh);
    pe.setParam("downsamp",0);
    pe.loadAndSetDatabase(this->db_path);
}

EstimatorConfig::Ptr
Estimator::getConfig() const
{
    return config;
}
// void
// BasicNode::updateIfNeeded(const BasicNode::ConfigPtr conf, bool reset)
// {
//     if (reset){
//         init();
//         return;
//     }
//     if (conf){
//         if (config->cropping != conf->cropping){
//             LOCK guard(mtx_config);
//             config->cropping = conf->cropping;
//         }
//         if (config->downsampling != conf->downsampling){
//             LOCK guard(mtx_config);
//             config->downsampling = conf->downsampling;
//         }
//         if (config->segmenting != conf->segmenting){
//             LOCK guard(mtx_config);
//             config->segmenting = conf->segmenting;
//         }
//         if (config->keep_organized != conf->keep_organized){
//             LOCK guard(mtx_config);
//             config->keep_organized = conf->keep_organized;
//         }
//         if (config->publish_limits != conf->publish_limits){
//             LOCK guard(mtx_config);
//             config->publish_limits = conf->publish_limits;
//         }
//         // config->publish_plane = conf->publish_plane;
//         if (config->limits != conf->limits){
//             LOCK guard(mtx_config);
//             config->limits = conf->limits;
//             update_markers();
//         }
//         if (config->downsampling_leaf_size != conf->downsampling_leaf_size){
//             LOCK guard(mtx_config);
//             config->downsampling_leaf_size = conf->downsampling_leaf_size;
//         }
//         if (config->plane_tolerance != conf->plane_tolerance){
//             LOCK guard(mtx_config);
//             config->plane_tolerance = conf->plane_tolerance;
//         }
//     }
// }
//
void
Estimator::publish_markers()
{
    if (marks && pub_markers.getNumSubscribers()>0){
        for(const auto& m: marks->markers)
        {
            m.header.stamp = ros::Time();
            pub_markers.publish(m);
        }
    }
    /*
     * if (config->publish_plane && pub_markers.getNumSubscribers()>0){
     *     mark_plane.header.stamp = ros::Time();
     *     pub_markers.publish(mark_plane);
     * }
     */
}
int
Estimator::extract_clusters()
{
    storage->read_scene_processed(scene);
    if (scene->empty()){
        ROS_WARN("[Estimator][%s]\tProcessed scene is empty, cannot continue...",__func__);
        return -1;
    }
    double tol;
    config->get("cluster_tol", tol);
    ROS_INFO("[Estimator][%s]\tExtracting object clusters with cluster tolerance of %g",__func__, tol);
    //objects
    pcl::ExtractIndices<PX> extract;
    pcl::EuclideanClusterExtraction<PX> ec;
    pcl::search::KdTree<PX>::Ptr tree (new pcl::search::KdTree<PX>);
    std::vector<pcl::PointIndices> cluster_indices;
    //cluster extraction
    tree->setInputCloud(scene);
    ec.setInputCloud(scene);
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(tol);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(scene->points.size());
    ec.extract(cluster_indices);
    int size = (int)cluster_indices.size();
    clusters.reset (new std::vector<PXC> );
    clusters->resize(size);
    names.reset(new std::vector<std::pair<std::string, std::string>>);
    names->resize(size);
    estimations.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);
    estimations->resize(size);
    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
    {
        PXC::Ptr object (new PXC);
        extract.setInputCloud(scene);
        extract.setIndices(boost::make_shared<pcl::PointIndices>(*it));
        extract.setNegative(false);
        extract.filter(clusters->at(j));
    }
    ROS_INFO("[Estimator][%s]\tFound %d clusters of possible objects.",__func__,size);
    return size;
}

bool
Estimator::cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Estimator][%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (this->estimate()){
        geometry_msgs::Pose pose;
        for (int i=0; i<estimations->size(); ++i)
        {
            fromEigen(estimations->at(i), pose);
            pacman_vision_comm::pe pose_est;
            pose_est.pose = pose;
            pose_est.name = names->at(i).first;
            pose_est.id = names->at(i).second;
            pose_est.parent_frame = scene->header.frame_id;
            res.estimated.poses.push_back(pose_est);
        }
        ROS_INFO("[Estimator][%s]\tPose Estimation complete!", __func__);
        return true;
    }
    else{
        ROS_WARN("[Estimator][%s]\tPose Estimation failed!", __func__);
        return false;
    }
}

bool
Estimator::estimate()
{
    int size = this->extract_clusters();
    if (size < 1){
        ROS_ERROR("[Estimator][%s]\tNo object clusters found in scene, aborting pose estimation...",__func__);
        return false;
    }
    bool calib;
    int it,k;
    config->get("iterations", it);
    if (it!=iter){
        iter = it;
        pe.setStepIterations(iter);
    }
    config->get("neighbors", k);
    if (k!= neigh){
        neigh = k;
        pe.setParam("lists_size", neigh);
    }
    config->get("object_calibration", calib);
    for (int i=0; i<size; ++i)
    {
        pe.setTarget(clusters->at(i).makeShared(), "object");
        pel::Candidate pest;
        pe.estimate(pest);
        std::string name = pest.getName();
        std::vector<std::string> vst;
        boost::split (vst, name, boost::is_any_of("_"), boost::token_compress_on);
        if (calib)
            names->at(i).first = "object";
        else
            names->at(i).first = vst.at(0);
        estimations->at(i) = pest.getTransformation();
        names->at(i).second = vst.at(0);
        ROS_INFO("[Estimator][%s]\tFound %s.",__func__,name.c_str());
    }
    //first check if we have more copy of the same object in names
    for (int i=0; i<names->size(); ++i)
    {
        int count(1);
        std::string name_original = names->at(i).first;
        if (i>0){
            for (int j=0; j<i; ++j)
            {
                if (names->at(i).first.compare(names->at(j).first) == 0){
                    //i-th name is equal to j-th name
                    names->at(i).first = name_original + "_" + std::to_string(++count);
                }
            }
        }
    }
    //Save estimations in Storage
    this->storage->write_obj_clusters(this->clusters);
    this->storage->write_obj_names(this->names);
    this->storage->write_obj_transforms(this->estimations);
    //elaborate new markers
    create_markers();
    return true;
}

void
Estimator::create_markers()
{
    std::string ref_frame;
    storage->read_sensor_ref_frame(ref_frame);
    marks = std::make_shared<visualization_msgs::MarkerArray>();
    for (int i=0; i<estimations->size(); ++i) //if size is zero dont do anything
    {
        geometry_msgs::Pose pose;
        tf::Transform trans;
        fromEigen(estimations->at(i), pose, trans);
        visualization_msgs::Marker marker;
        marker.header.frame_id = ref_frame.c_str();
        marker.header.stamp = ros::Time();
        marker.ns=names->at(i).second.c_str();
        if (names->at(i).second.compare(names->at(i).first) != 0){
            std::vector<std::string> vst;
            boost::split(vst, names->at(i).first, boost::is_any_of("_"), boost::token_compress_on);
            int id = std::stoi(vst.at(vst.size()-1));
            marker.id = id;
        }
        else{
            marker.id = 1;
        }
        marker.scale.x=1;
        marker.scale.y=1;
        marker.scale.z=1;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        std::string mesh_path ("package://asus_scanner_models/" + names->at(i).second + "/" + names->at(i).second + ".stl");
        marker.mesh_resource = mesh_path.c_str();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.lifetime = ros::Duration(1);
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.3f;
        marker.color.a = 1.0f;
        marks->markers.push_back(marker);
    }
}

void
Estimator::spinOnce()
{
    publish_markers();
    queue_ptr->callAvailable(ros::WallDuration(0));
}
}
