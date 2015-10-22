//TODO:
//0)Add hand box markers, rework vito geometry and scale
//0.5)Add modular pose scanner, based on find turn table and hdf5
//0.75) Finish pose scanner
//1)when tracker re-finds object in scene: save relative transform hand-object, so you can start from it
//2)Fill tracker service grasp verification
//3)Make a separated thread for Kinect2Processor ?! (wait until libfreenect2 is more developed)

#include <pacman_vision/vision_node.h>

VisionNode::VisionNode() : box_scale(1.0f), rqt_init(true)
{
    this->nh = ros::NodeHandle("pacman_vision");
    this->storage.reset(new Storage);
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
    this->kinect2.reset(new Kinect2Processor);
#endif
    this->scene.reset(new PC);
    this->limits.reset(new Box);
    //first call of dynamic reconfigure callback will only set gui
    //to loaded parameters
    //service callback init
    srv_get_scene = nh.advertiseService("get_scene_processed",
                                            &VisionNode::cb_get_scene, this);
    pub_scene = nh.advertise<PC> ("scene_processed", 5);
    //init node params
    nh.param<bool>("enable_estimator", en_estimator, false);
    nh.param<bool>("enable_tracker", en_tracker, false);
    nh.param<bool>("enable_broadcaster", en_broadcaster, false);
    nh.param<bool>("enable_listener", en_listener, false);
    nh.param<bool>("enable_supervoxels", en_supervoxels, false);
    nh.param<bool>("enable_scanner", en_scanner, false);
    nh.param<bool>("passthrough", filter, true);
    nh.param<bool>("downsampling", downsample, false);
    nh.param<bool>("plane_segmentation", plane, false);
    nh.param<bool>("keep_organized", keep_organized, false);
    nh.param<int>("external_kinect2_resolution", sensor.resolution, 1);
    nh.param<double>("pass_xmax", limits->x2, 0.5);
    nh.param<double>("pass_xmin", limits->x1, -0.5);
    nh.param<double>("pass_ymax", limits->y2, 0.5);
    nh.param<double>("pass_ymin", limits->y1, -0.5);
    nh.param<double>("pass_zmax", limits->z2, 1.0);
    nh.param<double>("pass_zmin", limits->z1, 0.3);
    nh.param<double>("downsample_leaf_size", leaf, 0.01);
    nh.param<bool>("crop_right_arm", crop_r_arm, false);
    nh.param<bool>("crop_left_arm", crop_l_arm, false);
    nh.param<bool>("crop_right_hand", crop_r_hand, false);
    nh.param<bool>("crop_left_hand", crop_l_hand, false);
    nh.param<bool>("use_table_transform", use_table_trans, false);
    nh.param<int>("sensor_type", sensor.type, 0);
    nh.param<bool>("Master_Disable", master_disable, false);
    nh.param<double>("plane_tolerance", plane_tol, 0.004);
    //set callback for dynamic reconfigure
    this->dyn_srv.setCallback(boost::bind(&VisionNode::cb_reconfigure, this,
                                                                    _1, _2));
#ifndef PACMAN_VISION_WITH_KINECT2_SUPPORT
    //force use of openni2
    sensor.type = 2;
    sensor.needs_update = true;
#endif
}

//when service to get scene is called
bool
VisionNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req,
                                pacman_vision_comm::get_scene::Response& res)
{
    if (!master_disable)
    {
        if (this->scene_processed)
        {
            sensor_msgs::PointCloud2 msg;
            if (req.save.compare("false") != 0)
            {
                std::string home = std::getenv("HOME");
                pcl::io::savePCDFile( (home + "/" + req.save + ".pcd").c_str(),
                                                            *scene_processed);
                ROS_INFO("[PaCMaN Vision][%s] Processed scene saved to %s"
                        ,__func__, (home + "/" + req.save + ".pcd").c_str());
            }
            pcl::toROSMsg(*scene_processed, msg);
            res.scene = msg;
            ROS_INFO("[PaCMaN Vision][%s] Sent processed scene to service response."
                                                                    ,__func__);
            return true;
        }
        else
        {
            ROS_WARN("[PaCMaN Vision][%s] No Processed Scene to send to Service!"
                                                                    ,__func__);
            return false;
        }
    }
    else
    {
        ROS_WARN("[PaCMaN Vision][%s] Node is globally disabled, renable it!"
                                                                    ,__func__);
        return false;
    }
}

//when new msg from sensor arrive
void
VisionNode::cb_kinect(const sensor_msgs::PointCloud2::ConstPtr& message)
{
    if(!master_disable)
    {
        if (!this->scene_processed)
            this->scene_processed.reset( new PC);
        pcl::fromROSMsg (*message, *(this->scene));
        // Save untouched scene into storage
        this->storage->write_scene(this->scene);
        process_scene();
        publish_scene_processed();
    }
}

////////////////////////////////
///////Spinner threads//////////
////////////////////////////////
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
void
VisionNode::spin_estimator()
{
    ROS_INFO("[Estimator] Estimator module will try to perform a Pose Estimation on each object found in the Processed Scene. It isolates possible objects by means of Euclidean Clustering, thus a plane segmentation should be performed on the scene. Estimator needs an object database, previously created, which must be put into database folder. More info is available on the Module Readme.");
    //spin until we disable it or it dies somehow
    while (this->en_estimator && this->estimator_module)
    {
        if(!plane)
        {
            ROS_WARN_THROTTLE(30,"[Estimator] Estimator module will not function properly without plane segmentation. Please enable at least plane segmentation for scene processing.");
        }
        if(downsample && (leaf < 0.001 || leaf > 0.01))
        {
            ROS_WARN_THROTTLE(30,"[Estimator] Estimator module uses a prebuilt database of poses with its own downsampling leaf size. Database downsampling leaf size is hardcoded at 0.005, thus it is not recommended to use a scene leaf size too far from that value. Current scene leaf size is %g", leaf);
        }
        //This module actually does nothing directly,
        //it just waits for user to call the service
        this->estimator_module->spin_once();
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        //estimator could try to go at 2Hz (no need to process services faster)
    }
    //estimator got stopped
    return;
}

void
VisionNode::spin_tracker()
{
    ROS_INFO("[Tracker] Tracker module will try to track an already Pose Estimated object (from Estimator) as it moves around. An object model (its complete mesh and point cloud) must be present inside asus_scanner_models ROS package.");
    //spin until we disable it or it dies somehow
    while (this->en_tracker && this->tracker_module)
    {
        if (downsample)
            this->tracker_module->leaf = this->leaf;
        else
        {
            ROS_WARN_THROTTLE(30, "[Tracker] Tracker module will not function properly without scene downsampling, please enable it.");
        }
        //spin it
        if (this->tracker_module->started)
        {
            if (this->en_estimator && this->estimator_module)
            {
                //Better to temporary disable Estimator while tracker is
                //tracking so it doesnt mess up with estimated objects
                //in storage!
                this->estimator_module->disabled = true;
            }
            this->tracker_module->track();
        }
        else if (this->tracker_module->lost_it &&
                                                !this->tracker_module->started)
        {
            //The object is lost...  what now!? Lets try to find it
            this->tracker_module->find_object_in_scene();
        }
        else
        {
            //Tracker is not started
            if (this->en_estimator && this->estimator_module)
            {
                //Re-enable Estimator if it was disabled,
                //tracker is not tracking anymore
                this->estimator_module->disabled = false;
            }
        }
        this->tracker_module->spin_once();
        boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        //tracker could try to go as fast as reasonably possible (50Hz)
    }
    //tracker got stopped
    return;
}
#endif

void
VisionNode::spin_broadcaster()
{
    ROS_INFO("[Broadcaster] Broadcaster module will publish tfs and markers calculated by the other modules or the base node itself. Look at the namespace of markers in the MarkerArray.");
    //spin until we disable it or it dies somehow
    while (this->en_broadcaster && this->broadcaster_module)
    {
        //Clear previous markers
        broadcaster_module->markers.markers.clear();
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
        //Check if we have to publish estimated objects or tracked one
        if ( (this->en_estimator && this->estimator_module) ||
                                    (this->tracker_module && this->en_tracker))
        {
            if (broadcaster_module->obj_markers || broadcaster_module->obj_tf
                                            || broadcaster_module->tracker_bb)
                //this takes care of markers and TFs of all pose estimated
                //objects, plus tracked object and its bounding box
                broadcaster_module->elaborate_estimated_objects_markers();
        }
#endif

        //publish Passthrough filter limits as a box
        if(filter && broadcaster_module->pass_limits)
        {
            visualization_msgs::Marker box_marker;
            if(this->broadcaster_module->create_box_marker(box_marker, *limits))
            {
                box_marker.color.r = 1.0f;
                box_marker.color.g = 0.0f;
                box_marker.color.b = 0.0f;
                box_marker.color.a = 1.0f;
                box_marker.pose.position.x=0;
                box_marker.pose.position.y=0;
                box_marker.pose.position.z=0;
                box_marker.pose.orientation.x=0;
                box_marker.pose.orientation.y=0;
                box_marker.pose.orientation.z=0;
                box_marker.pose.orientation.w=1;
                box_marker.ns = "PassThrough Filter Limits";
                box_marker.id = 1;
                this->broadcaster_module->markers.markers.push_back(box_marker);
            }
        }

        if(listener_module && en_listener)
        {
            if (crop_r_arm && broadcaster_module->arm_boxes)
            {
                if (!this->storage->read_right_arm(right_arm))
                {
                    right_arm.reset(new std::vector<Eigen::Matrix4f,
                                    Eigen::aligned_allocator<Eigen::Matrix4f>>);
                    right_arm->resize(7);
                    for (auto& x: *right_arm)
                        x.setIdentity();
                }
                for (int i=0; i<right_arm->size();++i)
                {
                    visualization_msgs::Marker box;
                    create_arm_box_marker(right_arm->at(i), box,
                                                lwr_arm[i]*box_scale, i, true);
                    this->broadcaster_module->markers.markers.push_back(box);
                }
            }
            if (crop_l_arm && broadcaster_module->arm_boxes)
            {
                if (!this->storage->read_left_arm(left_arm))
                {
                    left_arm.reset(new std::vector<Eigen::Matrix4f,
                                    Eigen::aligned_allocator<Eigen::Matrix4f>>);
                    left_arm->resize(7);
                    for (auto& x: *left_arm)
                        x.setIdentity();
                }
                for (int i=0; i<left_arm->size();++i)
                {
                    visualization_msgs::Marker box;
                    create_arm_box_marker(left_arm->at(i), box,
                                            lwr_arm[i]*box_scale, i, false);
                    this->broadcaster_module->markers.markers.push_back(box);
                }
            }
            //TODO add hand boxes
        }

        //Actually do the broadcasting.
        //This also sets timestamps of all markers pushed inside the array
        this->broadcaster_module->broadcast_once();
        //spin
        this->broadcaster_module->spin_once();
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        //broadcaster could try to go at 20Hz
    }
    //broadcaster got stopped
    return;
}

void
VisionNode::spin_listener()
{
    size_t count_to_table (0);
    ROS_INFO("[Listener] Listener module will try to read Vito Robot arms and hands transformations to perform hands/arms cropping on the processed scene.");
    //instant fetch of table transform, it will refetch it later
    listener_module->listen_table();
    this->listener_module->spin_once();
    //spin until we disable it or it dies somehow
    while (this->en_listener && this->listener_module)
    {
        if (count_to_table > 10000)
        {
            //Re-read table, as a precaution, but it should not change
            listener_module->listen_table();
            count_to_table = 0;
        }
        //do the listening
        this->listener_module->listen_once();
        //spin
        this->listener_module->spin_once();
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        //listener could try to go at 20Hz
        ++count_to_table;
    }
    //listener got stopped
    return;
}

void
VisionNode::spin_supervoxels()
{
    ROS_INFO("[Supervoxels] Supervoxels module will segment processed scene into SuperVoxel clusters and republish it.");
    //spin until we disable it or it dies somehow
    while (this->en_supervoxels && this->supervoxels_module)
    {
        if (!this->supervoxels_module->serviced)
            this->supervoxels_module->clustering();

        //spin
        this->supervoxels_module->spin_once();
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        //Supervoxels can try to go at 20Hz
    }
    //supervoxels got stopped
    return;
}

/*
   void VisionNode::spin_scanner()
   {
   ROS_INFO("[Pose Scanner] Pose Scanner module will perform acquisition of objects poses when calling appropriated service.");
//spin until we disable it or it dies somehow
while (this->en_scanner && this->scanner_module)
{
//spin
this->scanner_module->spin_once();
boost::this_thread::sleep(boost::posix_time::milliseconds(100)); //Scanner can try to go at 10Hz
}
//scanner got stopped
return;
}
*/
////////////////////////////////////
////// Checkers methods ////////////
////////////////////////////////////
void
VisionNode::check_modules()
{
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
    //check if we want estimator module and it is not started,
    //or if it is started but we want it disabled
    if (this->en_estimator && !this->estimator_module && !master_disable)
    {
        ROS_WARN("[PaCMaN Vision] Started Estimator module");
        this->estimator_module.reset( new Estimator(this->nh, this->storage) );
        //spawn a thread to handle the module spinning
        estimator_driver = boost::thread(&VisionNode::spin_estimator, this);
    }
    else if (!this->en_estimator && this->estimator_module)
    {
        ROS_WARN("[PaCMaN Vision] Stopped Estimator module");
        //wait for the thread to stop, if not already,
        //if already stopped this results in a no_op
        estimator_driver.join();
        //kill the module
        this->estimator_module.reset();
    }

    //check if we want tracker module and it is not started,
    //or if it is started but we want it disabled
    if (this->en_tracker && !this->tracker_module && !master_disable)
    {
        ROS_WARN("[PaCMaN Vision] Started Tracker module");
        this->tracker_module.reset( new Tracker(this->nh, this->storage) );
        //spawn a thread to handle the module spinning
        tracker_driver = boost::thread(&VisionNode::spin_tracker, this);
    }
    else if (!this->en_tracker && this->tracker_module)
    {
        ROS_WARN("[PaCMaN Vision] Stopped Tracker module");
        //wait for the thread to stop, if not already,
        //if already stopped this results in a no_op
        tracker_driver.join();
        //kill the module
        this->tracker_module.reset();
    }
#endif

    //check if we want broadcaster module and it is not started,
    //or if it is started but we want it disabled
    if (this->en_broadcaster && !this->broadcaster_module && !master_disable)
    {
        ROS_WARN("[PaCMaN Vision] Started Broadcaster module");
        this->broadcaster_module.reset( new Broadcaster(this->nh, this->storage) );
        //spawn a thread to handle the module spinning
        broadcaster_driver = boost::thread(&VisionNode::spin_broadcaster, this);
    }
    else if (!this->en_broadcaster && this->broadcaster_module)
    {
        ROS_WARN("[PaCMaN Vision] Stopped Broadcaster module");
        //wait for the thread to stop, if not already,
        //if already stopped this results in a no_op
        broadcaster_driver.join();
        //kill the module
        this->broadcaster_module.reset();
    }

    //check if we want listener module and it is not started,
    //or if it is started but we want it disabled
    if (this->en_listener && !this->listener_module && !master_disable)
    {
        ROS_WARN("[PaCMaN Vision] Started Vito Listener module");
        this->listener_module.reset( new Listener(this->nh, this->storage) );
        //spawn a thread to handle the module spinning
        listener_driver = boost::thread(&VisionNode::spin_listener, this);
    }
    else if (!this->en_listener && this->listener_module)
    {
        ROS_WARN("[PaCMaN Vision] Stopped Vito Listener module");
        //wait for the thread to stop, if not already,
        //if already stopped this results in a no_op
        listener_driver.join();
        //kill the module
        this->listener_module.reset();
    }

    //check if we want supervoxels module and it is not started,
    //or if it is started but we want it disabled
    if (this->en_supervoxels && !this->supervoxels_module && !master_disable)
    {
        ROS_WARN("[PaCMaN Vision] Started Supervoxels module");
        this->supervoxels_module.reset( new Supervoxels(this->nh, this->storage) );
        //spawn a thread to handle the module spinning
        supervoxels_driver = boost::thread(&VisionNode::spin_supervoxels, this);
    }
    else if (!this->en_supervoxels && this->supervoxels_module)
    {
        ROS_WARN("[PaCMaN Vision] Stopped Supervoxels module");
        //wait for the thread to stop, if not already,
        //if already stopped this results in a no_op
        supervoxels_driver.join();
        //kill the module
        this->supervoxels_module.reset();
    }
    /*
    //check if we want pose scanner module and it is not started, or if it is started but we want it disabled
    if (this->en_scanner && !this->scanner_module && !master_disable)
    {
    ROS_WARN("[PaCMaN Vision] Started Pose Scanner module");
    this->scanner_module.reset( new PoseScanner(this->nh, this->storage) );
    //spawn a thread to handle the module spinning
    scanner_driver = boost::thread(&VisionNode::spin_scanner, this);
    }
    else if (!this->en_scanner && this->scanner_module)
    {
    ROS_WARN("[PaCMaN Vision] Stopped Pose Scanner module");
    //wait for the thread to stop, if not already, if already stopped this results in a no_op
    scanner_driver.join();
    //kill the module
    this->scanner_module.reset();
    }
    */
}

void
VisionNode::check_sensor()
{
    if (sensor.type == 1)
    {
        //Use external kinect2 bridge
        if (sensor.needs_update)
        {
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
            std::string topic;
            if (sensor.resolution == 2)
                topic = nh.resolveName("/kinect2/hd/points");
            else if (sensor.resolution == 1)
                topic = nh.resolveName("/kinect2/qhd/points");
            else if (sensor.resolution == 0)
                topic = nh.resolveName("/kinect2/sd/points");
            else //unhandled default to sd
                topic = nh.resolveName("/kinect2/sd/points");
            sensor.ref_frame = "/kinect2_rgb_optical_frame";
            this->storage->write_sensor_ref_frame(sensor.ref_frame);
            sub_kinect = nh.subscribe(topic, 5, &VisionNode::cb_kinect, this);
            if (this->kinect2->started || this->kinect2->initialized)
            {
                kinect2->stop();
                kinect2->close();
            }
#endif
            sensor.needs_update = false;
        }
    }
    else if (sensor.type == 2)
    {
        //use external openni2
        if (sensor.needs_update)
        {
            std::string topic;
            topic = nh.resolveName("/camera/depth_registered/points");
            sensor.ref_frame = "/camera_rgb_optical_frame";
            this->storage->write_sensor_ref_frame(sensor.ref_frame);
            sub_kinect = nh.subscribe(topic, 5, &VisionNode::cb_kinect, this);
            sensor.needs_update = false;
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
            if (this->kinect2->started || this->kinect2->initialized)
            {
                kinect2->stop();
                kinect2->close();
            }
#endif
        }
    }
    else
    {
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
        if (sensor.needs_update)
        {
            sensor.ref_frame = "/kinect2_reference_frame";
            this->storage->write_sensor_ref_frame(sensor.ref_frame);
        }
        //Use internal sensor processor, no subscriber needed
        sub_kinect.shutdown();
#endif
    }
}

void
VisionNode::spin_once()
{
    if(master_disable)
    {
        sub_kinect.shutdown();
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
        if (kinect2->started)
            kinect2->stop();
        if (sensor.type != 0 && kinect2->initialized)
            kinect2->close();
#endif
        sensor.needs_update = true;
    }
    else
    {
        this->check_sensor();
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
        if (sensor.type == 0)
        {
            if (!kinect2->initialized)
                kinect2->initDevice();
            if (!kinect2->started)
                kinect2->start();
            kinect2->processData();
            kinect2->computePointCloud(this->scene);
            if (!this->scene_processed)
                this->scene_processed.reset( new PC);
            tf::Vector3 v_t(0.0385,0,0);
            tf::Quaternion q_zero;
            q_zero.setRPY(0,0,0);
            tf::Transform t_zero(q_zero, v_t);
            this->tf_sensor_ref_frame_brcaster.sendTransform(
                                tf::StampedTransform(t_zero, ros::Time::now(),
                                "/kinect2_anchor", sensor.ref_frame.c_str()));
            pcl_conversions::toPCL(ros::Time::now(), this->scene->header.stamp);
            this->scene->header.frame_id = sensor.ref_frame;
            this->scene->header.seq = 0;
            this->storage->write_scene(this->scene);
            this->process_scene();
            this->publish_scene_processed();
        }
#endif
    }
    ros::spinOnce();
    this->check_modules();
}

//dynamic reconfigure callback
void
VisionNode::cb_reconfigure(pacman_vision::pacman_visionConfig &config,
                                                                uint32_t level)
{
    //initial gui init based on params (just do it once)
    if (rqt_init)
    {
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
        config.enable_estimator = en_estimator;
        config.enable_tracker= en_tracker;
        //estimator
        nh.getParam("object_calibration",
                            config.groups.estimator_module.object_calibration);
        nh.getParam("iterations", config.groups.estimator_module.iterations);
        nh.getParam("neighbors", config.groups.estimator_module.neighbors);
        nh.getParam("cluster_tol", config.groups.estimator_module.cluster_tol);
        //tracker
        config.groups.tracker_module.tracker_disturbance = false;
        nh.setParam("tracker_disturbance", false);
        nh.getParam("estimation_type",
                                config.groups.tracker_module.estimation_type);
        //broadcaster specific
        nh.getParam("publish_tf", config.groups.broadcaster_module.publish_tf);
        nh.getParam("estimated_objects",
                            config.groups.broadcaster_module.estimated_objects);
        nh.getParam("tracker_bounding_box",
                        config.groups.broadcaster_module.tracker_bounding_box);
#endif
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
        config.external_kinect2_resolution = sensor.resolution;
        config.sensor_type = sensor.type;
#endif
        config.enable_broadcaster = en_broadcaster;
        config.enable_listener = en_listener;
        config.enable_supervoxels = en_supervoxels;
        config.enable_scanner = en_scanner;
        config.Master_Disable = master_disable;
        config.groups.base_node_filters.downsampling = downsample;
        config.groups.base_node_filters.passthrough = filter;
        config.groups.base_node_filters.plane_segmentation = plane;
        config.groups.base_node_filters.plane_tolerance = plane_tol;
        config.groups.base_node_filters.keep_organized = keep_organized;
        config.groups.base_node_filters.downsample_leaf_size = leaf;
        config.groups.base_node_filters.pass_xmax = limits->x2;
        config.groups.base_node_filters.pass_xmin = limits->x1;
        config.groups.base_node_filters.pass_ymax = limits->y2;
        config.groups.base_node_filters.pass_ymin = limits->y1;
        config.groups.base_node_filters.pass_zmax = limits->z2;
        config.groups.base_node_filters.pass_zmin = limits->z1;
        //listener
        nh.getParam("crop_right_arm",
                                config.groups.listener_module.crop_right_arm);
        nh.getParam("crop_left_arm",
                                config.groups.listener_module.crop_left_arm);
        nh.getParam("crop_right_hand",
                                config.groups.listener_module.crop_right_hand);
        nh.getParam("crop_left_hand",
                                config.groups.listener_module.crop_left_hand);
        nh.getParam("use_table_transform",
                            config.groups.listener_module.use_table_transform);
        nh.getParam("geometry_scale",
                                config.groups.listener_module.geometry_scale);
        //broadcaster
        nh.getParam("passthrough_limits",
                        config.groups.broadcaster_module.passthrough_limits);
        nh.getParam("arm_boxes", config.groups.broadcaster_module.arm_boxes);
        nh.getParam("sensor_fake_calibration",
                    config.groups.broadcaster_module.sensor_fake_calibration);
        //Supervoxels
        nh.getParam("use_service",
                                config.groups.supervoxels_module.use_service);
        nh.getParam("voxel_resolution",
                            config.groups.supervoxels_module.voxel_resolution);
        nh.getParam("seed_resolution",
                            config.groups.supervoxels_module.seed_resolution);
        nh.getParam("color_importance",
                            config.groups.supervoxels_module.color_importance);
        nh.getParam("spatial_importance",
                        config.groups.supervoxels_module.spatial_importance);
        nh.getParam("normal_importance",
                        config.groups.supervoxels_module.normal_importance);
        nh.getParam("refinement_iterations",
                    config.groups.supervoxels_module.refinement_iterations);
        nh.getParam("normals_search_radius",
                    config.groups.supervoxels_module.normals_search_radius);
        //Pose Scanner
        nh.getParam("ignore_clicked_point",
                        config.groups.pose_scanner_module.ignore_clicked_point);
        nh.getParam("work_dir", config.groups.pose_scanner_module.work_dir);
        nh.getParam("table_pass", config.groups.pose_scanner_module.table_pass);
        //Finish gui initialization
        this->rqt_init = false;
        this->sensor.needs_update = true;
        ROS_WARN("[PaCMaN Vision] Rqt-Reconfigure Default Values Initialized");
        return;
    }
    //Normal behaviour
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
    this->en_estimator    = config.enable_estimator;
    this->en_tracker      = config.enable_tracker;
    //Estimator Module
    if (this->estimator_module && this->en_estimator)
    {
        this->estimator_module->calibration =
                            config.groups.estimator_module.object_calibration;
        this->estimator_module->iterations =
                                    config.groups.estimator_module.iterations;
        this->estimator_module->neighbors =
                                    config.groups.estimator_module.neighbors;
        this->estimator_module->clus_tol =
                                    config.groups.estimator_module.cluster_tol;
        this->estimator_module->pe.setParam("lists_size",
                                                estimator_module->neighbors);
        this->estimator_module->pe.setStepIterations(
                                                estimator_module->iterations);
    }
    //Tracker Module
    if (this->tracker_module && this->en_tracker)
    {
        this->tracker_module->type =
                                config.groups.tracker_module.estimation_type;
        if (config.groups.tracker_module.tracker_disturbance)
        {
            tracker_module->manual_disturbance = true;
            config.groups.tracker_module.tracker_disturbance = false;
        }
    }
#endif
    this->en_broadcaster  = config.enable_broadcaster;
    this->en_listener     = config.enable_listener;
    this->en_supervoxels  = config.enable_supervoxels;
    this->en_scanner      = config.enable_scanner;
    //Global Node Disable
    this->master_disable = config.Master_Disable;
    if (this->master_disable)
    {
        //Force disable of all modules
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
        config.enable_estimator = config.enable_tracker = false;
        en_estimator = en_tracker = false;
#endif
        config.enable_listener = config.enable_broadcaster =
                    config.enable_supervoxels = config.enable_scanner = false;
        en_broadcaster = en_listener = en_supervoxels = en_scanner = false;
    }
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
    if (sensor.resolution != config.external_kinect2_resolution)
    {
        sensor.resolution = config.external_kinect2_resolution;
        this->sensor.needs_update = true;
    }
    if (sensor.type != config.sensor_type)
    {
        sensor.type = config.sensor_type;
        this->sensor.needs_update = true;
    }
#endif
    //filters
    this->downsample = config.groups.base_node_filters.downsampling;
    this->filter = config.groups.base_node_filters.passthrough;
    this->plane = config.groups.base_node_filters.plane_segmentation;
    this->plane_tol = config.groups.base_node_filters.plane_tolerance;
    this->keep_organized = config.groups.base_node_filters.keep_organized;
    this->leaf = config.groups.base_node_filters.downsample_leaf_size;
    this->limits->x2 = config.groups.base_node_filters.pass_xmax;
    this->limits->x1 = config.groups.base_node_filters.pass_xmin;
    this->limits->y2 = config.groups.base_node_filters.pass_ymax;
    this->limits->y1 = config.groups.base_node_filters.pass_ymin;
    this->limits->z2 = config.groups.base_node_filters.pass_zmax;
    this->limits->z1 = config.groups.base_node_filters.pass_zmin;
    //Listener Module
    if (this->listener_module && this->en_listener)
    {
        this->crop_r_arm = config.groups.listener_module.crop_right_arm;
        this->crop_l_arm = config.groups.listener_module.crop_left_arm;
        this->crop_r_hand = config.groups.listener_module.crop_right_hand;
        this->crop_l_hand = config.groups.listener_module.crop_left_hand;
        this->box_scale = listener_module->box_scale =
                                config.groups.listener_module.geometry_scale;
        this->use_table_trans = config.groups.listener_module.use_table_transform;
        this->listener_module->listen_left_arm = this->crop_l_arm;
        this->listener_module->listen_right_arm = this->crop_r_arm;
        this->listener_module->listen_left_hand = this->crop_l_hand;
        this->listener_module->listen_right_hand = this->crop_r_hand;
    }
    //Broadcaster Module
    if (this->broadcaster_module && this->en_broadcaster)
    {
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
        this->broadcaster_module->obj_tf =
                                config.groups.broadcaster_module.publish_tf;
        this->broadcaster_module->obj_markers =
                            config.groups.broadcaster_module.estimated_objects;
        this->broadcaster_module->tracker_bb =
                        config.groups.broadcaster_module.tracker_bounding_box;
#endif
        this->broadcaster_module->pass_limits =
                            config.groups.broadcaster_module.passthrough_limits;
        this->broadcaster_module->arm_boxes =
                                    config.groups.broadcaster_module.arm_boxes;
        this->broadcaster_module->sensor_fake_calibration =
                    config.groups.broadcaster_module.sensor_fake_calibration;
    }
    //Supervoxels Module
    if (this->supervoxels_module && this->en_supervoxels)
    {
        this->supervoxels_module->serviced =
                                config.groups.supervoxels_module.use_service;
        this->supervoxels_module->voxel_res =
                            config.groups.supervoxels_module.voxel_resolution;
        this->supervoxels_module->seed_res =
                            config.groups.supervoxels_module.seed_resolution;
        this->supervoxels_module->color_imp =
                            config.groups.supervoxels_module.color_importance;
        this->supervoxels_module->spatial_imp =
                        config.groups.supervoxels_module.spatial_importance;
        this->supervoxels_module->normal_imp =
                            config.groups.supervoxels_module.normal_importance;
        this->supervoxels_module->num_iterations =
                        config.groups.supervoxels_module.refinement_iterations;
        this->supervoxels_module->normal_radius =
                        config.groups.supervoxels_module.normals_search_radius;
    }
    //Pose Scanner Module
    /*if (this->scanner_module && this->en_scanner)
      {
      this->scanner_module->table_pass   = config.groups.pose_scanner_module.table_pass;
      this->scanner_module->work_dir  = config.groups.pose_scanner_module.work_dir;
      this->scanner_module->ignore_clicked_point = config.groups.pose_scanner_module.ignore_clicked_point;
      }
      */
    ROS_INFO("[PaCMaN Vision] Reconfigure request executed");
}

void VisionNode::shutdown()
{
    en_tracker = en_broadcaster = en_estimator = en_listener =
                                        en_supervoxels = en_scanner = false;
    this->check_modules();
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
    this->kinect2->close();
#endif
    ROS_INFO("[PaCMan Vision] Shutting down...");
    //Wait for other threads
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    ROS_INFO("[PaCMan Vision] Bye!");
}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "pacman_vision");
    VisionNode node;
    ros::Rate rate(100); //try to go at 100hz
    while (node.nh.ok())
    {
        node.spin_once();
        rate.sleep();
    }
    node.shutdown();
    return 0;
}
