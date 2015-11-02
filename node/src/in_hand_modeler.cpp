#include <pacman_vision/in_hand_modeler.h>
#include <pcl/common/time.h>

InHandModeler::InHandModeler(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor): has_transform(false), do_acquisition(false),
            oct_adj(0.003), oct_cd(0.003), do_alignment(false), oct_adj_frames(0.01), do_removal(false)
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
    teDQ.reset(new pcl::registration::TransformationEstimationDualQuaternion<PT,PT,float>);
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
    if(!ignore_clicked_point && !do_acquisition){
        float nx,ny,nz;
        PT pt;
        //Get clicked point
        pt.x = msg->point.x;
        pt.y = msg->point.y;
        pt.z = msg->point.z;
        std::string sensor;
        PC::Ptr scene;
        this->storage->read_scene_processed(scene);
        this->storage->read_sensor_ref_frame(sensor);
        if (msg->header.frame_id.compare(sensor) != 0){
            //Put the point read from rviz into kinect reference frame (if needed)
            tf_listener.waitForTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(1.0));
            tf::StampedTransform t_msg;
            tf_listener.lookupTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), t_msg);
            Eigen::Matrix4f T;
            geometry_msgs::Pose pose;
            fromTF(t_msg, T, pose);
            PT pt_t;
            pt_t = pcl::transformPoint<PT>(pt, Eigen::Affine3f(T));
            //now the point is in kinect ref. frame.
            pt.x = pt_t.x;
            pt.y = pt_t.y;
            pt.z = pt_t.z;
        }
        //compute a normal around the point neighborhood (2cm)
        pcl::search::KdTree<PT> kdtree;
        std::vector<int> idx(scene->points.size());
        std::vector<float> dist(scene->points.size());
        kdtree.setInputCloud(scene);
        kdtree.radiusSearch(pt, 0.02, idx, dist);
        pcl::NormalEstimationOMP<PT, pcl::Normal> ne_point;
        ne_point.setInputCloud(scene);
        ne_point.useSensorOriginAsViewPoint();
        float curv;
        ne_point.computePointNormal (*scene, idx, nx,ny,nz, curv);
        //Compute model transform
        if (!computeModelTransform(pt,nx,ny,nz)){
            ROS_WARN("[InHandModeler][%s]\tFailed to compute Model Transform, please click again!", __func__);
            return;
        }
        //save this scene into the sequence, so that we can use it as a start.
        if (!cloud_sequence.empty())
            cloud_sequence.clear();
        cloud_sequence.push_back(scene);
    }
    else{
        ROS_INFO("[InHandModeler][%s]Ignoring clicked point as requested or because already started processing...", __func__);
    }
}

bool
InHandModeler::cb_start(pacman_vision_comm::start_modeler::Request& req, pacman_vision_comm::start_modeler::Response& res)
{
    if (!has_transform){
        ROS_ERROR("[InHandModeler][%s]\tNo model transform defined, please click and publish a point in rviz", __func__);
        return (false);
    }
    if (do_alignment || do_acquisition || do_removal){
        ROS_ERROR("[InHandModeler][%s]\talignment is already started!", __func__);
        return (false);
    }
    ///////////////////////Init Registration////////////////////////////////////
    //RANSAC maximum iterations
    alignment.setMaximumIterations(50000);
    /*
     * The number of point correspondences  to sample between source and target.
     * At minimum, 3 points are required to calculate a pose.
     */
    alignment.setNumberOfSamples(3);
    /*
     * Instead of matching  each source FPFH descriptor to  its nearest matching
     * feature  in the  target, we  can  choose between  the N  best matches  at
     * random.  This increases  the  iterations necessary,  but  also makes  the
     * algorithm robust towards outlier matches.
     */
    alignment.setCorrespondenceRandomness(5);
    //Use  dual quaternion  method to  estimate a  rigid transformation  between
    //correspondences.
    //alignment.setTransformationEstimation(teDQ);
    /*
     * The alignment  class uses the CorrespondenceRejectorPoly  class for early
     * elimination of bad poses  based on pose-invariant geometric consistencies
     * of  the inter-distances  between sampled  points  on the  source and  the
     * target. The closer  this value is set  to 1, the more  greedy and thereby
     * fast  the algorithm  becomes. However,  this also  increases the  risk of
     * eliminating good poses when noise is present.
     */
    alignment.setSimilarityThreshold(0.9f);
    // Reject correspondences more distant than this value.
    // This is heuristically set to 10 times point density
    alignment.setMaxCorrespondenceDistance(2.5f * 0.005f);
    /*
     * In many  practical scenarios,  large parts  of the  source in  the target
     * scene are not visible, either due to clutter, occlusions or both. In such
     * cases, we  need to allow  for pose hypotheses that  do not align  all the
     * source  points to  the target  scene.  The absolute  number of  correctly
     * aligned points is determined using the inlier threshold, and if the ratio
     * of this number to the total number of points in the source is higher than
     * the specified inlier fraction, we accept a pose hypothesis as valid.
     */
    alignment.setInlierFraction(0.25f);

    //initialize the model with first cloud from sequence
    model.reset(new PC);
    model_ds.reset(new PC);
    pcl::copyPointCloud(*cloud_sequence.front(), *model);
    vg.setInputCloud(model);
    vg.setLeafSize(model_ls, model_ls, model_ls);
    vg.filter(*model_ds);
    //initialize the iterator pointers
    align_it = cloud_sequence.begin();
    remove_it = cloud_sequence.begin();
    //start registering
    do_acquisition = true;
    return (true);
}

bool
InHandModeler::cb_stop(pacman_vision_comm::stop_modeler::Request& req, pacman_vision_comm::stop_modeler::Response& res)
{
    if(!do_acquisition){
        ROS_ERROR("[InHandModeler][%s]\tAcquisition is already stopped!", __func__);
        return (false);
    }
    do_acquisition = false;
    //wait for alignment to end
    //alignment_driver.join();
    //TODO add save model to disk
    //model.reset();
    //model_ds.reset();
    //cloud_sequence.clear();
    return (true);
}

void
InHandModeler::alignSequence()
{
    //we use two elements at a time, so point to last one used
    ++align_it;
    while (do_alignment)
    {
        //determine if we have to wait for do_removal thread
        if (align_it == remove_it)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
            continue;
        }
        PC::Ptr target, source;
        //source and target normals
        NC::Ptr source_n(new NC), target_n(new NC);
        //source and target features
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_f(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_f(new pcl::PointCloud<pcl::FPFHSignature33>);
        //get source and target
        {
            LOCK guard(mtx_sequence);
            target = cloud_sequence.front();
            cloud_sequence.pop_front();
            source = cloud_sequence.front();
            ++align_it;
        }
        //downsample
        const float leaf = 0.005f;
        vg.setLeafSize(leaf, leaf, leaf);
        vg.setInputCloud(target);
        vg.filter(*target);
        vg.setInputCloud(source);
        vg.filter(*source);
        //estimate normals
        ne.setRadiusSearch(2.5f*leaf);
        ne.useSensorOriginAsViewPoint();
        ne.setInputCloud(source);
        ne.compute(*source_n);
        ne.setInputCloud(target);
        ne.compute(*target_n);
        //estimate features
        fpfh.setRadiusSearch(5.0f*leaf);
        fpfh.setInputCloud(source);
        fpfh.setInputNormals(source_n);
        fpfh.compute(*source_f);
        fpfh.setInputCloud(target);
        fpfh.setInputNormals(target_n);
        fpfh.compute(*target_f);
        //do the alignment
        alignment.setMaxCorrespondenceDistance(2.5f*leaf);
        alignment.setInputSource(source);
        alignment.setSourceFeatures(source_f);
        alignment.setInputTarget(target);
        alignment.setTargetFeatures(target_f);
        PC::Ptr source_aligned(new PC);
        pcl::ScopeTime t("alignment");
        alignment.align(*source_aligned);
        if (alignment.hasConverged()){
            //save result into sequence to be used as next target
            {
                LOCK guard(mtx_sequence);
                cloud_sequence.front() = source_aligned;
            }
            //update model
            PC::Ptr tmp (new PC);
            {
                LOCK guard(mtx_model);
                *model += *source_aligned;
                if(model->points.size() > 1e4){
                    vg.setInputCloud(model);
                    vg.setLeafSize(0.001, 0.001, 0.001);
                    vg.filter(*tmp);
                    pcl::copyPointCloud(*tmp, *model);
                }
                vg.setInputCloud(model);
                vg.setLeafSize(model_ls, model_ls, model_ls);
                vg.filter(*tmp);
                pcl::transformPointCloud(*tmp, *model_ds, T_mk);
            }
        }
        //TODO add octomap hand removal
        else{
            ROS_ERROR("[InHandModeler][%s]Alignment FAILED!",__func__);
            //TODO add error handling and termination
        }
        {
            LOCK guard(mtx_sequence);
            if(cloud_sequence.size() < 2)
            {
                ROS_INFO("[InHandModeler][%s]Finished alignment!", __func__);
                break;
            }
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }//endwhile
    do_alignment = false;
}

void
InHandModeler::removeSimilarFramesFromSequence()
{
    while (do_removal)
    {
        if (remove_it + 1 == cloud_sequence.end()){
            if(do_acquisition){
                boost::this_thread::sleep(boost::posix_time::milliseconds(200));
                continue;
            }
            else{
                break;
            }
        }
        PC::Ptr current(new PC);
        PC::Ptr next_c(new PC);
        {
            LOCK guard(mtx_sequence);
            current = *remove_it;
            next_c = *remove_it+1;
        }
        oct_adj_frames.setInputCloud(current);
        oct_adj_frames.addPointsFromInputCloud();
        oct_adj_frames.switchBuffers();
        oct_adj_frames.setInputCloud(next_C);
        oct_adj_frames.addPointsFromInputCloud();
        std::vector<int> changes;
        oct_adj_frames.getPointIndicesFromNewVoxels(changes);
        oct_adj_frames.deleteCurrentBuffer();
        oct_adj_frames.deletePreviousBuffer();
        if (changes.size() > next_c->size() * 0.1){
            //more than 10% of points have changed, we keep it
            ++remove_it;
        }
        else{
            //frames are almost equal we remove one
            LOCK guard(mtx_sequence);
            cloud_sequence.erase(remove_it+1);
        }
    }//endwhile
    do_removal = false;
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
    if (do_acquisition){
        PC::Ptr scene (new PC);
        this->storage->read_scene_processed(scene);
        {
            LOCK guard(mtx_sequence);
            cloud_sequence.push_back(scene);
        }
    }
    if (!do_removal){
        if (cloud_sequence.size()>10){
            //time to start the removal thread
            do_removal = true;
            remove_driver = boost::thread(&InHandModeler::removeSimilarFramesFromSequence, this);
        }
    }
    if (!do_alignment){
        if (cloud_sequence.size()>20){
            //time to start the alignment thread
            do_alignment = true;
            alignment_driver = boost::thread(&InHandModeler::alignSequence, this);
        }
    }
    {
        LOCK guard(mtx_model);
        if (model_ds)
            if(!model_ds->empty() && pub_model.getNumSubscribers()>0)
                pub_model.publish(*model_ds);
    }
    //process this module callbacks
    this->queue_ptr->callAvailable(ros::WallDuration(0));
    ROS_WARN("Sequence size: %d", (int)cloud_sequence.size()); //TODO remove
}
