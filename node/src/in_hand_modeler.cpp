#include <pacman_vision/in_hand_modeler.h>

InHandModeler::InHandModeler(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor): has_transform(false), do_iterations(false)
{
    this->nh = ros::NodeHandle(n, "in_hand_modeler");
    this->queue_ptr.reset(new ros::CallbackQueue);
    this->nh.setCallbackQueue(&(*this->queue_ptr));
    this->storage = stor;
    this->srv_start = nh.advertiseService("start", &InHandModeler::cb_start, this);
    this->srv_stop = nh.advertiseService("stop", &InHandModeler::cb_stop, this);
    this->model.reset(new PC);
    nh.param<bool>("/pacman_vision/ignore_clicked_point",ignore_clicked_point , false);
    nh.param<double>("/pacman_vision/model_leaf_size",model_ls, 0.001);
    std::string work_dir_s;
    nh.param<std::string>("/pacman_vision/work_dir", work_dir_s, "InHandModeler");
    work_dir = work_dir_s;
    sub_clicked = nh.subscribe(nh.resolveName("/clicked_point"), 1, &InHandModeler::cb_clicked, this);
    pub_model = nh.advertise<PC> ("in_hand_model",1);
    crd.reset(new pcl::registration::CorrespondenceRejectorDistance);
    teDQ.reset(new pcl::registration::TransformationEstimationDualQuaternion<PT, PT, float>);
}

bool
InHandModeler::computeModelTransform(PT pt, float nx, float ny, float nz)
{
    try
    {
        Eigen::Vector3f new_z(nx,ny,nz); //Z axis is the plane normal
        new_z.normalize();
        //Find a vector as close as possible to x-kinect but orthonormal to new_z
        Eigen::Vector3f kin_x(Eigen::Vector3f::UnitX());
        Eigen::Vector3f new_x;
        new_x = kin_x - (new_z*(new_z.dot(kin_x)));
        new_x.normalize();
        //New_y is new_z cross new_x;
        Eigen::Vector3f new_y;
        new_y = new_z.cross(new_x);
        new_y.normalize();

        /*
         * Rotation matrix of new model reference system with respect to
         * kinect ref. frame. Translation that express model ref system centre
         * with respect to kinect (the clicked point).
         * Then Compose the 4x4 rototraslation
         */
        T_km << new_x[0], new_y[0], new_z[0], pt.x,
                new_x[1], new_y[1], new_z[1], pt.y,
                new_x[2], new_y[2], new_z[2], pt.z,
                0,        0,        0,        1;
        T_mk = T_km.inverse();
        has_transform = true;
    }
    catch (...)
    {
        return (false);
    }
    return (true);
}

bool InHandModeler::saveModel()
{
    //TODO
}

void InHandModeler::cb_clicked (const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if(!ignore_clicked_point){
        float nx,ny,nz;
        PT pt;
        //Get clicked point
        pt.x = msg->point.x;
        pt.y = msg->point.y;
        pt.z = msg->point.z;
        std::string sensor;
        this->storage->read_scene_processed(actual_k);
        this->storage->read_sensor_ref_frame(sensor);
        if (msg->header.frame_id.compare(sensor) != 0){
            tf_listener.waitForTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(1.0));
            tf::StampedTransform t_msg;
            tf_listener.lookupTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), t_msg);
            Eigen::Matrix4f T;
            geometry_msgs::Pose pose;
            fromTF(t_msg, T, pose);
            PT pt_t;
            pt_t = pcl::transformPoint<PT>(pt, Eigen::Affine3f(T));
            pt.x = pt_t.x;
            pt.y = pt_t.y;
            pt.z = pt_t.z;
        }
        //compute a normal around its neighborhood (3cm)
        pcl::search::KdTree<PT> kdtree;
        std::vector<int> idx(actual_k->points.size());
        std::vector<float> dist(actual_k->points.size());
        kdtree.setInputCloud(actual_k);
        kdtree.radiusSearch(pt, 0.03, idx, dist);
        pcl::NormalEstimation<PT, pcl::Normal> ne;
        ne.setInputCloud(actual_k);
        ne.useSensorOriginAsViewPoint();
        float curv;
        ne.computePointNormal (*actual_k, idx, nx,ny,nz, curv);
        //Compute model transform
        if (!computeModelTransform(pt,nx,ny,nz)){
            ROS_WARN("[InHandModeler][%s]\tFailed to compute Model Transform, please click again!", __func__);
            return;
        }
    }
}

bool
InHandModeler::cb_start(pacman_vision_comm::start_modeler::Request& req, pacman_vision_comm::start_modeler::Response& res)
{
    if (!has_transform){
        ROS_ERROR("[InHandModeler][%s]\tNo model transform defined, please click and publish a point in rviz", __func__);
        return (false);
    }
    //Init ICP
    icp.setUseReciprocalCorrespondences(true);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(1e-4);
    crd->setMaximumDistance(0.02); //2cm
    icp.addCorrespondenceRejector(crd);
    icp.setTransformationEstimation(teDQ);
    //transform actual scene in model ref frame
    storage->read_scene_processed(actual_k);
    if (!actual_m)
        actual_m.reset(new PC);
    pcl::transformPointCloud(*actual_k, *actual_m, T_mk);
    //initialize the model with actual scene
    if (!model)
        model.reset(new PC);
    if (!model_ds)
        model_ds.reset(new PC);
    pcl::copyPointCloud(*actual_m, *model);
    vg.setInputCloud(model);
    vg.setLeafSize(model_ls, model_ls, model_ls);
    vg.filter(*model_ds);
    //start iterations
    do_iterations = true;
    return (true);
}

bool
InHandModeler::cb_stop(pacman_vision_comm::stop_modeler::Request& req, pacman_vision_comm::stop_modeler::Response& res)
{
    do_iterations = false;
    //TODO add save model to disk
    return (true);
}

void
InHandModeler::iterate_once()
{
    //TODO
}

void
InHandModeler::spin_once()
{
    if (has_transform){
        tf::Transform t_km;
        geometry_msgs::Pose pose;
        fromEigen(T_km, pose, t_km);
        std::string sensor_ref_frame;
        this->storage->read_sensor_ref_frame(sensor_ref_frame);
        tf_broadcaster.sendTransform(tf::StampedTransform(t_km, ros::Time::now(), sensor_ref_frame.c_str(), "in_hand_model_frame"));
    }
    if (do_iterations)
        iterate_once();
    //process this module callbacks
    this->queue_ptr->callAvailable(ros::WallDuration(0));
}
