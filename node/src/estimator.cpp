#include <pacman_vision/estimator.h>

//Constructor
Estimator::Estimator(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor, const ros::Rate rate)
    :Module<Estimator>(n,ns,stor,rate)
{
    scene.reset(new PXC);
    config.reset(new EstimatorConfig);
    db_path = (ros::package::getPath("pacman_vision") + "/database" );
    if (!boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path))
        ROS_WARN("[Estimator][%s] Database for pose estimation does not exists!! Plese put one in /database folder, before trying to perform a pose estimation.",__func__);
    srv_estimate = nh.advertiseService("estimate", &Estimator::cb_estimate, this);
    //tmp set params to dump into default
    nh.setParam("object_calibration", false);
    nh.setParam("iterations", 5);
    nh.setParam("neighbors", 20);
    nh.setParam("cluster_tol", 0.05);
    ////////////////////////////////////////////////////////
    init();
}

void
Estimator::init()
{
    //init params
    nh.param<bool>("object_calibration", config->object_calibration, false); //legacy param for phase space star-object calibration
    nh.param<int>("iterations", config->iterations, 5);
    nh.param<int>("neighbors", config->neighbors, 20);
    nh.param<double>("cluster_tol", config->clus_tol, 0.05);
    pe.setParam("verbosity",2);
    pe.setRMSEThreshold(0.003);
    pe.setStepIterations(config->iterations);
    pe.setParam("lists_size",config->neighbors);
    pe.setParam("downsamp",0);
    pe.loadAndSetDatabase(this->db_path);
}

Estimator::ConfigPtr
Estimator::getConfig() const
{
    return config;
}

void
Estimator::updateIfNeeded(const Estimator::ConfigPtr conf, bool reset)
{
    if (reset){
        init();
        return;
    }
    if (conf){
        if (config->object_calibration != conf->object_calibration){
            LOCK guard(mtx_config);
            config->object_calibration = conf->object_calibration;
        }
        if (config->clus_tol != conf->clus_tol){
            LOCK guard(mtx_config);
            config->clus_tol = conf->clus_tol;
        }
        if (config->iterations != conf->iterations){
            LOCK guard(mtx_config);
            config->iterations = conf->iterations;
            pe.setStepIterations(config->iterations);
        }
        if (config->neighbors != conf->neighbors){
            LOCK guard(mtx_config);
            config->neighbors = conf->neighbors;
            pe.setParam("lists_size", config->neighbors);
        }
    }
}

int
Estimator::extract_clusters()
{
    storage->read_scene_processed(scene);
    if (scene->empty()){
        ROS_WARN("[Estimator][%s]\tProcessed scene is empty, cannot continue...",__func__);
        return -1;
    }
    {
        LOCK guard(mtx_config);
        ROS_INFO("[Estimator][%s]\tExtracting object clusters with cluster tolerance of %g",__func__, config->clus_tol);
    }
    //objects
    pcl::ExtractIndices<PX> extract;
    pcl::EuclideanClusterExtraction<PX> ec;
    pcl::search::KdTree<PX>::Ptr tree (new pcl::search::KdTree<PX>);
    std::vector<pcl::PointIndices> cluster_indices;
    //cluster extraction
    tree->setInputCloud(scene);
    ec.setInputCloud(scene);
    ec.setSearchMethod(tree);
    {
        LOCK guard(mtx_config);
        ec.setClusterTolerance(config->clus_tol);
    }
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
    if (this->isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Estimator][%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (this->estimate()){
        geometry_msgs::Pose pose;
        tf::Transform trans;
        for (int i=0; i<estimations->size(); ++i)
        {
            fromEigen(estimations->at(i), pose, trans);
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
    for (int i=0; i<size; ++i)
    {
        pe.setTarget(clusters->at(i).makeShared(), "object");
        pel::Candidate pest;
        pe.estimate(pest);
        std::string name = pest.getName();
        std::vector<std::string> vst;
        boost::split (vst, name, boost::is_any_of("_"), boost::token_compress_on);
        if (config->object_calibration)
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
    return true;
}

void Estimator::spin()
{
    ROS_INFO("[Estimator]\tEstimator module extract euclidean clusters from current scene and tries to identify each of them by matching with provided database. For the Estimator to work properly please enable at least plane segmentation during scene processing.");
    while(nh.ok() && is_running)
    {
        spinOnce();
        spin_rate.sleep();
    }
}
