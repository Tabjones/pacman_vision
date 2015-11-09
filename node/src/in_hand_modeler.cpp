#include <pacman_vision/in_hand_modeler.h>
#include <pcl/common/time.h>

InHandModeler::InHandModeler(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor): has_transform(false), do_acquisition(false),
            oct_adj(0.003), oct_cd(0.003), do_alignment(false), oct_cd_frames(0.01), do_frame_fusion(false), leaf(0.005f),
            leaf_f(0.001f), start_alignment(false), start_fusion(false), frames(0)
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
    fpfh.reset(new pcl::FPFHEstimationOMP<PT, NT, pcl::FPFHSignature33>);
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
    /*
     * if(!ignore_clicked_point && !do_acquisition){
     *     float nx,ny,nz;
     *     PT pt;
     *     //Get clicked point
     *     pt.x = msg->point.x;
     *     pt.y = msg->point.y;
     *     pt.z = msg->point.z;
     *     std::string sensor;
     *     PC::Ptr scene;
     *     this->storage->read_scene_processed(scene);
     *     this->storage->read_sensor_ref_frame(sensor);
     *     if (msg->header.frame_id.compare(sensor) != 0){
     *         //Put the point read from rviz into kinect reference frame (if needed)
     *         tf_listener.waitForTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(1.0));
     *         tf::StampedTransform t_msg;
     *         tf_listener.lookupTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), t_msg);
     *         Eigen::Matrix4f T;
     *         geometry_msgs::Pose pose;
     *         fromTF(t_msg, T, pose);
     *         PT pt_t;
     *         pt_t = pcl::transformPoint<PT>(pt, Eigen::Affine3f(T));
     *         //now the point is in kinect ref. frame.
     *         pt.x = pt_t.x;
     *         pt.y = pt_t.y;
     *         pt.z = pt_t.z;
     *     }
     *     //compute a normal around the point neighborhood (2cm)
     *     pcl::search::KdTree<PT> kdtree;
     *     std::vector<int> idx(scene->points.size());
     *     std::vector<float> dist(scene->points.size());
     *     kdtree.setInputCloud(scene);
     *     kdtree.radiusSearch(pt, 0.02, idx, dist);
     *     pcl::NormalEstimationOMP<PT, pcl::Normal> ne_point;
     *     ne_point.setInputCloud(scene);
     *     ne_point.useSensorOriginAsViewPoint();
     *     float curv;
     *     ne_point.computePointNormal (*scene, idx, nx,ny,nz, curv);
     *     //Compute model transform
     *     if (!computeModelTransform(pt,nx,ny,nz)){
     *         ROS_WARN("[InHandModeler][%s]\tFailed to compute Model Transform, please click again!", __func__);
     *         return;
     *     }
     *     //save this scene into the sequence, so that we can use it as a start.
     *     if (!cloud_sequence.empty())
     *         cloud_sequence.clear();
     *     cloud_sequence.push_back(scene);
     * }
     * else{
     *     ROS_INFO("[InHandModeler][%s]Ignoring clicked point as requested or because already started processing...", __func__);
     * }
     */
}

bool
InHandModeler::cb_start(pacman_vision_comm::start_modeler::Request& req, pacman_vision_comm::start_modeler::Response& res)
{
    /*
     * if (!has_transform){
     *     ROS_ERROR("[InHandModeler][%s]\tNo model transform defined, please click and publish a point in rviz", __func__);
     *     return (false);
     * }
     */
    if (do_alignment || do_acquisition || do_frame_fusion){
        ROS_ERROR("[InHandModeler][%s]\talignment is already started!", __func__);
        return (false);
    }
    // ///////////////////////Init Registration////////////////////////////////////
    // //RANSAC maximum iterations
    // alignment.setMaximumIterations(50000);
    // #<{(|
    //  * The number of point correspondences  to sample between source and target.
    //  * At minimum, 3 points are required to calculate a pose.
    //  |)}>#
    // alignment.setNumberOfSamples(3);
    // #<{(|
    //  * Instead of matching  each source FPFH descriptor to  its nearest matching
    //  * feature  in the  target, we  can  choose between  the N  best matches  at
    //  * random.  This increases  the  iterations necessary,  but  also makes  the
    //  * algorithm robust towards outlier matches.
    //  |)}>#
    // alignment.setCorrespondenceRandomness(5);
    // //Use  dual quaternion  method to  estimate a  rigid transformation  between
    // //correspondences.
    // //alignment.setTransformationEstimation(teDQ);
    // #<{(|
    //  * The alignment  class uses the CorrespondenceRejectorPoly  class for early
    //  * elimination of bad poses  based on pose-invariant geometric consistencies
    //  * of  the inter-distances  between sampled  points  on the  source and  the
    //  * target. The closer  this value is set  to 1, the more  greedy and thereby
    //  * fast  the algorithm  becomes. However,  this also  increases the  risk of
    //  * eliminating good poses when noise is present.
    //  |)}>#
    // alignment.setSimilarityThreshold(0.9f);
    // // Reject correspondences more distant than this value.
    // // This is heuristically set to 10 times point density
    // alignment.setMaxCorrespondenceDistance(2.5f * 0.005f);
    // #<{(|
    //  * In many  practical scenarios,  large parts  of the  source in  the target
    //  * scene are not visible, either due to clutter, occlusions or both. In such
    //  * cases, we  need to allow  for pose hypotheses that  do not align  all the
    //  * source  points to  the target  scene.  The absolute  number of  correctly
    //  * aligned points is determined using the inlier threshold, and if the ratio
    //  * of this number to the total number of points in the source is higher than
    //  * the specified inlier fraction, we accept a pose hypothesis as valid.
    //  |)}>#
    // alignment.setInlierFraction(0.25f);
    //
    //initialize the model with first cloud from sequence
    model.reset(new PC);
    model_ds.reset(new PC);
    cloud_sequence.clear();
    // pcl::copyPointCloud(*cloud_sequence.front(), *model);
    // vg.setInputCloud(model);
    // vg.setLeafSize(model_ls, model_ls, model_ls);
    // vg.filter(*model_ds);
    //start acquiring sequence
    do_acquisition = true;
    do_alignment = do_frame_fusion = false;
    start_alignment = start_fusion = false;
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
    do_frame_fusion = true;
    fuse_it = cloud_sequence.begin();
    ROS_INFO("[InHandModeler]\tPlease wait for processing to end...");
    return (true);
}

void
InHandModeler::alignSequence()
{
    //TODO remove multithreading
    pcl::visualization::PCLVisualizer v("registration"); //todo temp visualization
    int v1(0), v2(1);
    v.createViewPort(0.0,0.0,0.5,1.0, v1);
    v.createViewPort(0.5,0.0,1.0,1.0, v2);
    //initialize the iterator pointers
    align_it = cloud_sequence.begin();
    //we use two elements at a time, so point to last one used
    ++align_it;
    bool tmp(true);
    while (do_alignment)
    {
        //determine if we have to wait for do_frame_fusion thread
        if (align_it == fuse_it && do_frame_fusion)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
            continue;
        }
        PC::Ptr target(new PC), source(new PC);
        //tmp
        if(tmp){
            v.addPointCloud(source,"source",v1);
            v.addPointCloud(target,"target",v2);
            tmp=false;
        }
        //
        //source and target normals
        NC::Ptr source_n(new NC), target_n(new NC);
        //source and target features
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_f(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_f(new pcl::PointCloud<pcl::FPFHSignature33>);
        //get source and target and downsample them
        {
            vg.setLeafSize(leaf, leaf, leaf);
            LOCK guard(mtx_sequence);
            //downsample
            vg.setInputCloud(cloud_sequence.front().makeShared());
            vg.filter(*target);
            cloud_sequence.pop_front();
            vg.setInputCloud(cloud_sequence.front().makeShared());
            vg.filter(*source);
            ++align_it;
            //TODO: Need to check later if source and target needs to be switched
            //(Fri 06 Nov 2015 08:07:25 PM CET -- tabjones)
        }

        //estimate normals
        ne.setRadiusSearch(2.0f*leaf);
        ne.useSensorOriginAsViewPoint();
        ne.setInputCloud(target);
        ne.compute(*target_n);
        ne.setInputCloud(source);
        ne.compute(*source_n);
        //initialize feature estimator and compute it for target
        std::vector<float> scales;
        scales.push_back(2.0f*leaf);
        scales.push_back(2.5f*leaf);
        scales.push_back(3.0f*leaf);
        scales.push_back(3.5f*leaf);
        pcl::DefaultPointRepresentation<pcl::FPFHSignature33> point_rep;
        fpfh->setInputNormals(target_n);
        fpfh->setInputCloud(target);
        persistance.setInputCloud(target);
        persistance.setScalesVector(scales);
        persistance.setFeatureEstimator(fpfh);
        persistance.setPointRepresentation(point_rep.makeShared());
        persistance.setAlpha(2); //presumably this is std_dev multiplier
        persistance.setDistanceMetric(CS); //use chi squared distance
        boost::shared_ptr<std::vector<int>> target_p_idx (new std::vector<int>);
        persistance.determinePersistentFeatures(*target_f, target_p_idx);
        //do it again for source
        persistance.setInputCloud(source);
        persistance.setScalesVector(scales);
        fpfh->setInputNormals(source_n);
        fpfh->setInputCloud(source);
        persistance.setFeatureEstimator(fpfh);
        persistance.setAlpha(2); //presumably this is std_dev multiplier
        persistance.setDistanceMetric(CS); //use chi squared distance
        boost::shared_ptr<std::vector<int>> source_p_idx (new std::vector<int>);
        persistance.determinePersistentFeatures(*source_f, source_p_idx);
        //tmp color pers feat red
        uint8_t r=255, g=0, b=0;
        uint32_t rgb = ( (uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b );
        for (size_t i=0; i<source_p_idx->size(); ++i)
            source->points[source_p_idx->at(i)].rgb = *reinterpret_cast<float*>(&rgb);
        for (size_t i=0; i<target_p_idx->size(); ++i)
            target->points[target_p_idx->at(i)].rgb = *reinterpret_cast<float*>(&rgb);
        v.updatePointCloud(source, "source");
        v.updatePointCloud(target, "target");
        v.spinOnce(1000,true);
        ROS_INFO("[InHandModeler] One step");
        ////

        //do the alignment
//
//         alignment.setmaxcorrespondencedistance(8.0f*leaf);
//         alignment.setinputsource(source);
//         // alignment.setindices(same);
//         alignment.setsourcefeatures(source_f);
//         alignment.setinputtarget(target);
//         alignment.settargetfeatures(target_f);
//         pc::ptr source_aligned(new pc);
//         pcl::scopetime t("alignment");
//         alignment.align(*source_aligned);
//         if (alignment.hasconverged()){
//             //save result into sequence to be used as next target
//             {
//                 eigen::matrix4f t = alignment.getfinaltransformation();
//                 lock guard(mtx_sequence);
//                 pcl::transformpointcloud(*cloud_sequence.front(),*source_aligned, t);
//                 cloud_sequence.front() = source_aligned;
//             }
//             //update model
//             pc::ptr tmp (new pc);
//             {
//                 lock guard(mtx_model);
//                 *model += *source_aligned;
//                 if(model->points.size() > 1e4){
//                     vg.setinputcloud(model);
//                     vg.setleafsize(0.001, 0.001, 0.001);
//                     vg.filter(*tmp);
//                     pcl::copypointcloud(*tmp, *model);
//                 }
//                 vg.setinputcloud(model);
//                 vg.setleafsize(model_ls, model_ls, model_ls);
//                 vg.filter(*tmp);
//                 pcl::transformpointcloud(*tmp, *model_ds, t_mk);
//             }
//         }
//         //todo add octomap hand removal
//         else{
//             ros_error("[inhandmodeler][%s]alignment failed!",__func__);
//             //todo add error handling and termination
//         }
//         {
//             lock guard(mtx_sequence);
//             if(cloud_sequence.size() < 2)
//             {
//                 ros_info("[inhandmodeler][%s]finished alignment!", __func__);
//                 break;
//             }
//         }
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }//endwhile
    do_alignment = false;
}

void
InHandModeler::fuseSimilarFrames()
{
    std::list<PC>::iterator fuse_next = ++fuse_it;
    --fuse_it;
    if (fuse_next == cloud_sequence.end()){
            //finished traversing sequence, let's get out of here
            do_alignment = true;
            do_frame_fusion = false;
            align_it = cloud_sequence.begin();
            ROS_INFO("[InHandModeler][%s]\tFinished!",__func__);
            return;
    }
    PC::Ptr current(new PC);
    PC::Ptr next_c(new PC);
    current = fuse_it->makeShared();
    next_c = fuse_next->makeShared();
    oct_cd_frames.setInputCloud(current);
    oct_cd_frames.addPointsFromInputCloud();
    oct_cd_frames.switchBuffers();
    oct_cd_frames.setInputCloud(next_c);
    oct_cd_frames.addPointsFromInputCloud();
    std::vector<int> changes;
    oct_cd_frames.getPointIndicesFromNewVoxels(changes);
    oct_cd_frames.deleteCurrentBuffer();
    oct_cd_frames.deletePreviousBuffer();
    if (changes.size() > next_c->size() * 0.04){
        //from new  frame more than  4% of  points were not  in previous
        //one, most likely there was a  motion, so we keep the new frame
        //into sequence and move on.
        ++fuse_it;
        return;
    }
    else{
        //old and  new frames are  almost equal in point  differences we
        //can fuse them togheter into a single frame and resample.
        *current += *next_c;
        pcl::VoxelGrid<PT> resamp;
        resamp.setLeafSize(leaf_f,leaf_f,leaf_f);
        resamp.setInputCloud(current);
        resamp.filter(*fuse_it);
        cloud_sequence.erase(fuse_next);
    }
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
        PC::Ptr scene;
        this->storage->read_scene_processed(scene);
        cloud_sequence.push_back(*scene);
        ++frames;
    }
    if (!do_acquisition && frames>1 && do_frame_fusion)
        //perform fusion of two frames if necessary
        fuseSimilarFrames();
    if (!do_acquisition && frames>1 && do_alignment)
        //time to start the alignment
        alignSequence();
    if (model_ds)
        if(!model_ds->empty() && pub_model.getNumSubscribers()>0)
            pub_model.publish(*model_ds);
    //process this module callbacks
    this->queue_ptr->callAvailable(ros::WallDuration(0));
    //ROS_WARN("sequence size: %d", (int)cloud_sequence.size()); //todo remove
}
