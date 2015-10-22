#include <pacman_vision/estimator.h>

//Constructor
Estimator::Estimator(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor)
{
    this->scene.reset(new PXC);
    this->nh = ros::NodeHandle (n, "estimator");
    this->queue_ptr.reset(new ros::CallbackQueue);
    this->nh.setCallbackQueue(&(*this->queue_ptr));
    this->storage = stor;
    this->db_path = (ros::package::getPath("pacman_vision") + "/database" );
    if (!boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path))
        ROS_WARN("[Estimator][%s] Database for pose estimation does not exists!! Plese put one in /database folder, before trying to perform a pose estimation."
                                                                    ,__func__);
    this->srv_estimate = nh.advertiseService("estimate",
                                                &Estimator::cb_estimate, this);
    //init params
    nh.param<bool>("/pacman_vision/object_calibration", calibration, false);
    disabled = false;
    nh.param<int>("/pacman_vison/iterations", iterations, 5);
    nh.param<int>("/pacman_vision/neighbors", neighbors, 20);
    nh.param<double>("/pacman_vision/cluster_tol", clus_tol, 0.05);
    pe.setParam("verbosity",2);
    pe.setRMSEThreshold(0.003);
    pe.setStepIterations(iterations);
    pe.setParam("lists_size",neighbors);
    pe.setParam("downsamp",0);
    pe.loadAndSetDatabase(this->db_path);
    ROS_INFO("[Estimator] Estimator module extract euclidean clusters from current scene and tries to identify each of them by matching with provided database. For the Estimator to work properly please enable at least plane segmentation during scene processing.");
}
Estimator::~Estimator()
{
    this->nh.shutdown();
}

int
Estimator::extract_clusters()
{
    this->storage->read_scene_processed(this->scene);
    if (scene->empty())
    {
        ROS_WARN("[Estimator][%s] Processed scene is empty, cannot continue..."
                                                                    ,__func__);
        return -1;
    }
    ROS_INFO("[Estimator][%s] Extracting object clusters with cluster tolerance of %g"
                                                        ,__func__, clus_tol);
    //objects
    pcl::ExtractIndices<PX> extract;
    pcl::EuclideanClusterExtraction<PX> ec;
    pcl::search::KdTree<PX>::Ptr tree (new pcl::search::KdTree<PX>);
    std::vector<pcl::PointIndices> cluster_indices;
    //cluster extraction
    tree->setInputCloud(scene);
    ec.setInputCloud(scene);
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(clus_tol);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(scene->points.size());
    ec.extract(cluster_indices);
    int size = (int)cluster_indices.size();
    clusters.reset (new std::vector<PXC> );
    clusters->resize(size);
    names.reset(new std::vector<std::pair<std::string, std::string> > );
    names->resize(size);
    estimations.reset(new std::vector<Eigen::Matrix4f,
                                Eigen::aligned_allocator<Eigen::Matrix4f>>);
    estimations->resize(size);
    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator
            it=cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
    {
        PXC::Ptr object (new PXC);
        extract.setInputCloud(scene);
        extract.setIndices(boost::make_shared<PointIndices>(*it));
        extract.setNegative(false);
        extract.filter(clusters->at(j));
    }
    ROS_INFO("[Estimator][%s] Found %d clusters of possible objects.",__func__
                                                                        ,size);
    return size;
}

bool
Estimator::cb_estimate(pacman_vision_comm::estimate::Request& req,
                                pacman_vision_comm::estimate::Response& res)
{
    if (this->disabled)
    {
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Estimator][%s] Estimator module is temporary disabled, exiting..."
                                                                    ,__func__);
        return false;
    }
    if (this->estimate())
    {
        geometry_msgs::Pose pose;
        tf::Transform trans;
        for (int i=0; i<estimations->size(); ++i)
        {
            fromEigen(estimations->at(i), pose, trans);
            pacman_vision_comm::pe pose_est;
            pose_est.pose = pose;
            pose_est.name = names->at(i).first;
            pose_est.id = names->at(i).second;
            pose_est.parent_frame = "/kinect2_rgb_optical_frame";
            res.estimated.poses.push_back(pose_est);
        }
        ROS_INFO("[Estimator][%s] Pose Estimation complete!", __func__);
        return true;
    }
    else
    {
        ROS_WARN("[Estimator][%s] Pose Estimation failed!", __func__);
        return false;
    }
}

bool
Estimator::estimate()
{
    int size = this->extract_clusters();
    if (size < 1)
    {
        ROS_ERROR("[Estimator][%s] No object clusters found in scene, aborting pose estimation..."
                                                                    ,__func__);
        return false;
    }
    for (int i=0; i<size; ++i)
    {
        pe.setTarget(clusters->at(i).makeShared(), "object");
        pel::Candidate pest;
        pe.estimate(pest);
        std::string name = pest.getName();
        std::vector<std::string> vst;
        boost::split (vst, name, boost::is_any_of("_"),
                                                    boost::token_compress_on);
        if (this->calibration)
            names->at(i).first = "object";
        else
            names->at(i).first = vst.at(0);
        estimations->at(i) = pest.getTransformation();
        names->at(i).second = vst.at(0);
        ROS_INFO("[Estimator][%s] Found %s.",__func__,name.c_str());
    }
    //first check if we have more copy of the same object in names
    for (int i=0; i<names->size(); ++i)
    {
        int count(1);
        std::string name_original = names->at(i).first;
        if (i>0)
        {
            for (int j=0; j<i; ++j)
            {
                if (names->at(i).first.compare(names->at(j).first) == 0)
                { //i-th name is equal to j-th name
                    names->at(i).first = name_original + "_"
                                                    + std::to_string(++count);
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

void
Estimator::spin_once()
{
    //process this module callbacks
    this->queue_ptr->callAvailable(ros::WallDuration(0));
}

