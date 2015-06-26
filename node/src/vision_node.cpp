#include "pacman_vision/vision_node.h"

VisionNode::VisionNode()
{
  this->nh = ros::NodeHandle("pacman_vision");
  this->storage.reset(new Storage);
  this->scene.reset(new PC);
  this->limits.reset(new Box);
  //first call of dynamic reconfigure callback will only set gui to loaded parameters
  rqt_init = true;
  //service callback init
  srv_get_scene = nh.advertiseService("get_scene_processed", &VisionNode::cb_get_scene, this);
  pub_scene = nh.advertise<PC> ("scene_processed", 3);
  //TODO possibly add them to dynamic reconfigure
  crop_r_arm = crop_l_arm = crop_r_hand = crop_l_hand = use_table_trans = false;
  //init node params
  nh.param<bool>("enable_estimator", en_estimator, false);
  nh.param<bool>("enable_tracker", en_tracker, false);
  nh.param<bool>("enable_broadcaster", en_broadcaster, false);
  nh.param<bool>("enable_listener", en_listener, false);
  nh.param<bool>("passthrough", filter, true);
  nh.param<bool>("downsampling", downsample, false);
  nh.param<bool>("plane_segmentation", plane, false);
  nh.param<bool>("keep_organized", keep_organized, false);
  nh.param<int>("kinect2_resolution", kinect2_resolution, 1);
  nh.param<double>("pass_xmax", limits->x2, 0.5);
  nh.param<double>("pass_xmin", limits->x1, -0.5);
  nh.param<double>("pass_ymax", limits->y2, 0.5);
  nh.param<double>("pass_ymin", limits->y1, -0.5);
  nh.param<double>("pass_zmax", limits->z2, 1.0);
  nh.param<double>("pass_zmin", limits->z1, 0.3);
  nh.param<double>("leaf_size", leaf, 0.01);
  nh.param<double>("plane_tolerance", plane_tol, 0.004);
  //set callback for dynamic reconfigure
  this->dyn_srv.setCallback(boost::bind(&VisionNode::cb_reconfigure, this, _1, _2));
}

bool VisionNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res)
{
  if (this->scene_processed)
  {
    sensor_msgs::PointCloud2 msg;
    if (req.save.compare("false") != 0)
    {
      std::string home = std::getenv("HOME");
      pcl::io::savePCDFile( (home + "/" + req.save + ".pcd").c_str(), *scene_processed);
      ROS_INFO("[PaCMaN Vision][%s] Processed scene saved to %s",__func__, (home + "/" + req.save + ".pcd").c_str() );
    }
    pcl::toROSMsg(*scene_processed, msg);
    res.scene = msg;
    ROS_INFO("[PaCMaN Vision][%s] Sent processed scene to service response.", __func__);
    return true;
  }
  else
  {
    ROS_WARN("[PaCMaN Vision][%s] No Processed Scene to send to Service!", __func__);
    return false;
  }
}

void VisionNode::cb_kinect(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  if (!this->scene_processed)
    this->scene_processed.reset( new PC);
  pcl::fromROSMsg (*message, *(this->scene));
  // Save untouched scene into storage
  this->storage->write_scene(this->scene);
  PC::Ptr tmp (new PC);
  //filters
  //check if we need to filter scene
  if (filter)
  {
    PassThrough<PT> pass;
    //check if have a table transform
    if (table_trans && use_table_trans)
    {
      this->storage->read_table(table_trans);
      pcl::CropBox<PT> cb;
      Eigen::Matrix4f inv_trans;
      inv_trans = table_trans->inverse();
      //check if we need to maintain scene organized
      if (keep_organized)
        cb.setKeepOrganized(true);
      else
        cb.setKeepOrganized(false);
      cb.setInputCloud (this->scene);
      Eigen::Vector4f min,max;
      //hardcoded table dimensions TODO make it dynamic reconfigurable if possible
      min << -0.1, -1.15, -0.1, 1;
      max << 0.825, 0.1, 1.5, 1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter (*(this->scene_processed));
    }
    else
    {
      PC::Ptr t (new PC);
      //check if we need to maintain scene organized
      if (keep_organized)
        pass.setKeepOrganized(true);
      else
        pass.setKeepOrganized(false);
      pass.setInputCloud (this->scene);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (limits->z1, limits->z2);
      pass.filter (*tmp);
      pass.setInputCloud (tmp);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (limits->y1, limits->y2);
      pass.filter (*t);
      pass.setInputCloud (t);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (limits->x1, limits->x2);
      pass.filter (*(this->scene_processed));
    }
  }
  //check if we need to downsample scene
  if (downsample) //cannot keep organized cloud after voxelgrid
  {
    VoxelGrid<PT> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setDownsampleAllData(true);
    if (filter)
      vg.setInputCloud (this->scene_processed);
    else
      vg.setInputCloud (this->scene);
    vg.filter (*tmp);
    pcl::copyPointCloud(*tmp, *(this->scene_processed));
  }
  if (plane)
  {
    pcl::SACSegmentation<PT> seg;
    pcl::ExtractIndices<PT> extract;
    //coefficients
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //plane segmentation
    if (filter || downsample)
      seg.setInputCloud(this->scene_processed);
    else
      seg.setInputCloud(this->scene);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (this->plane_tol);
    seg.segment(*inliers, *coefficients);
    //extract what's on top of plane
    extract.setInputCloud(seg.getInputCloud());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*tmp);
    pcl::copyPointCloud(*tmp, *(this->scene_processed));
  }
  //crop arms if listener set it
  if ( (crop_r_arm || crop_l_arm || crop_r_hand || crop_l_hand) && this->en_listener && this->listener_module )
  {
    PC::Ptr input (new PC);
    PC::Ptr output (new PC);
    pcl::CropBox<PT> cb;
    Eigen::Matrix4f inv_trans;
    Eigen::Vector4f min,max;
    if (filter || downsample || plane)
      pcl::copyPointCloud(*(this->scene_processed), *input);
    else
      pcl::copyPointCloud(*(this->scene), *input);
    if (crop_l_arm && left_arm)
    {
      //left2
      cb.setInputCloud(input);
      min << -0.06, -0.06, -0.06, 1;
      max << 0.06, 0.0938, 0.1915, 1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_arm->at(0).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter (*(tmp));
      //left3
      cb.setInputCloud(tmp);
      min << -0.06,-0.06,0,1;
      max << 0.06,0.0938,0.2685,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_arm->at(1).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //left4
      cb.setInputCloud(input);
      min << -0.06,-0.0938,-0.06,1;
      max << 0.06,0.06,0.1915,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_arm->at(2).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //left5
      cb.setInputCloud(tmp);
      min << -0.06,-0.0555,0,1;
      max << 0.06,0.06,0.2585,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_arm->at(3).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //left6
      cb.setInputCloud(input);
      min << -0.0711,-0.0555,-0.0711,1;
      max << 0.0711,0.0795,0.057,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_arm->at(4).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //left7
      cb.setInputCloud(tmp);
      min << -0.04,-0.0399,-0.031,1;
      max << 0.04,0.0399,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_arm->at(5).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      if (!crop_r_arm && !crop_r_hand && !crop_l_hand)
        pcl::copyPointCloud(*input, *(this->scene_processed));
    }
    if (crop_r_arm && right_arm)
    {
      //right2
      cb.setInputCloud(input);
      min << -0.06, -0.06, -0.06, 1;
      max << 0.06, 0.0938, 0.1915, 1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_arm->at(0).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //right3
      cb.setInputCloud(tmp);
      min << -0.06,-0.06,0,1;
      max << 0.06,0.0938,0.2685,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_arm->at(1).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //right4
      cb.setInputCloud(input);
      min << -0.06,-0.0938,-0.06,1;
      max << 0.06,0.06,0.1915,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_arm->at(2).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //right5
      cb.setInputCloud(tmp);
      min << -0.06,-0.0555,0,1;
      max << 0.06,0.06,0.2585,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_arm->at(3).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //right6
      cb.setInputCloud(input);
      min << -0.0711,-0.0555,-0.0711,1;
      max << 0.0711,0.0795,0.057,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_arm->at(4).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //right7
      cb.setInputCloud(tmp);
      min << -0.04,-0.0399,-0.031,1;
      max << 0.04,0.0399,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_arm->at(5).inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      if (!crop_r_hand && !crop_l_hand)
        pcl::copyPointCloud(*input, *(this->scene_processed));
    }
    if (crop_r_hand && right_hand)
    {
      //right hand //TODO hand measures
      cb.setInputCloud(input);
      min << 0,0,0,1;
      max << 0,0,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_hand->inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      if (!crop_l_hand)
        pcl::copyPointCloud(*tmp, *(this->scene_processed));
    }
    if (crop_l_hand && left_hand)
    {
      //left_hand
      if (crop_r_hand && right_hand)
        cb.setInputCloud(tmp);
      else
        cb.setInputCloud(input);
      min << 0,0,0,1;
      max << 0,0,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_hand->inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*(this->scene_processed));
    }
  }
  //republish processed cloud
  /* |passt   | voxelgrid   |segment | | arms or hands croppings                                 */
  if (filter || downsample || plane || crop_l_arm || crop_r_arm || crop_r_hand || crop_l_hand )
    pub_scene.publish(*scene_processed);
  else
    pub_scene.publish(*scene);
  //save scene processed into storage
  this->storage->write_scene_processed(this->scene_processed);
}

void VisionNode::check_modules()
{
  //check if we want estimator module and it is not started, or if it is started but we want it disabled
  if (this->en_estimator && !this->estimator_module)
  {
    ROS_WARN("[PaCMaN Vision] Started Estimator module");
    this->estimator_module.reset( new Estimator(this->nh, this->storage) );
    //spawn a thread to handle the module spinning
    estimator_driver = boost::thread(&VisionNode::spin_estimator, this);
  }
  else if (!this->en_estimator && this->estimator_module)
  {
    ROS_WARN("[PaCMaN Vision] Stopped Estimator module");
    //wait for the thread to stop, if not already, if already stopped this results in a no_op
    estimator_driver.join();
    //kill the module
    this->estimator_module.reset();
  }

  //check if we want broadcaster module and it is not started, or if it is started but we want it disabled
  if (this->en_broadcaster && !this->broadcaster_module)
  {
    ROS_WARN("[PaCMaN Vision] Started Broadcaster module");
    this->broadcaster_module.reset( new Broadcaster(this->nh, this->storage) );
    //spawn a thread to handle the module spinning
    broadcaster_driver = boost::thread(&VisionNode::spin_broadcaster, this);
  }
  else if (!this->en_broadcaster && this->broadcaster_module)
  {
    ROS_WARN("[PaCMaN Vision] Stopped Broadcaster module");
    //wait for the thread to stop, if not already, if already stopped this results in a no_op
    broadcaster_driver.join();
    //kill the module
    this->broadcaster_module.reset();
  }

  //check if we want tracker module and it is not started, or if it is started but we want it disabled
  if (this->en_tracker && !this->tracker_module)
  {
    ROS_WARN("[PaCMaN Vision] Started Tracker module");
    this->tracker_module.reset( new Tracker(this->nh, this->storage) );
    //spawn a thread to handle the module spinning
    tracker_driver = boost::thread(&VisionNode::spin_tracker, this);
  }
  else if (!this->en_tracker && this->tracker_module)
  {
    ROS_WARN("[PaCMaN Vision] Stopped Tracker module");
    //wait for the thread to stop, if not already, if already stopped this results in a no_op
    tracker_driver.join();
    //kill the module
    this->tracker_module.reset();
  }

  //check if we want listener module and it is not started, or if it is started but we want it disabled
  if (this->en_listener && !this->listener_module)
  {
    ROS_WARN("[PaCMaN Vision] Started Vito Listener module");
    this->listener_module.reset( new Listener(this->nh, this->storage) );
    //spawn a thread to handle the module spinning
    listener_driver = boost::thread(&VisionNode::spin_listener, this);
  }
  else if (!this->en_listener && this->listener_module)
  {
    ROS_WARN("[PaCMaN Vision] Stopped Vito Listener module");
    //wait for the thread to stop, if not already, if already stopped this results in a no_op
    listener_driver.join();
    //kill the module
    this->listener_module.reset();
  }
}
///////Spinner threads//////////
////////////////////////////////
void VisionNode::spin_estimator()
{
  ROS_INFO("[Estimator] Estimator module will try to perform a Pose Estimation on each object found in the Processed Scene.");
  ROS_INFO("[Estimator] It isolates possible objects by means of Euclidean Clustering, thus a plane segmentation should be performed on the scene.");
  ROS_INFO("[Estimator] Estimator needs an object database, previously created, which must be put into database folder. More info is available on the Module Readme.");
  int count_to_spam (0);
  //spin until we disable it or it dies somehow
  while (this->en_estimator && this->estimator_module)
  {
    if(count_to_spam > 50)
    {
      count_to_spam = 0;
      if(!plane)
      {
        ROS_WARN("[Estimator] Estimator module will not function properly without plane segmentation.");
        ROS_WARN("[Estimator] Please enable at least plane segmentation for scene processing.");
      }
      if(downsample && (leaf < 0.001 || leaf > 0.01))
      {
        ROS_WARN("[Estimator] Estimator module uses a prebuilt database of poses with its own downsampling leaf size.");
        ROS_WARN("[Estimator] Database downsampling leaf size should be 0.005, thus it is not recommended to use a scene leaf size too far from that value.");
        ROS_WARN("[Estimator] Now, scene leaf size is %g", leaf);
      }
    }
    //This module actually does nothing directly, it just waits for user to call the service
    this->estimator_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100)); //estimator could try to go at 10Hz (no need to process those services faster)
    ++count_to_spam;
  }
  //estimator got stopped
  return;
}

void VisionNode::spin_broadcaster()
{
  //spin until we disable it or it dies somehow
  while (this->en_broadcaster && this->broadcaster_module)
  {
    //Clear previous markers
    broadcaster_module->markers.markers.clear();
    //Check if we have to publish estimated objects or tracked one
    if (  (this->en_estimator && this->estimator_module) || (this->tracker_module && this->en_tracker) )
    {
      //this takes care of markers and TFs of all pose estimated objects, plus tracked object
      broadcaster_module->elaborate_estimated_objects();
    }

    //publish Passthrough filter limits as a box
    if(limits && filter)
    {
      visualization_msgs::Marker box_marker;
      if(this->broadcaster_module->create_box_marker(box_marker, limits))
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
    //Actually do the broadcasting. This also sets timestamps of all markers pushed inside the array
    this->broadcaster_module->broadcast_once();
    //spin
    this->broadcaster_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(33)); //broadcaster could try to go at 30Hz
  }
  //broadcaster got stopped
  return;
}

void VisionNode::spin_tracker()
{
  int count_to_spam (0);
  ROS_INFO("[Tracker] Tracker module will try to track an already Pose Estimated object (from Estimator) as it moves around.");
  ROS_INFO("[Tracker] An object model (its complete mesh and point cloud) must be present inside asus_scanner_models ROS package.");
  //spin until we disable it or it dies somehow
  while (this->en_tracker && this->tracker_module)
  {
    if (downsample)
      this->tracker_module->leaf = this->leaf;
    else
    {
      if (count_to_spam > 50)
      {
        ROS_WARN("[Tracker] Tracker module will not function properly without scene downsampling, please enable it.");
        count_to_spam = 0;
      }
    }
    //spin it
    if (this->tracker_module->started)
    {
      if (this->en_estimator && this->estimator_module)
      {
        //Better to temporary disable Estimator while tracker is tracking
        //so it doesnt mess up with estimated objects is storage
        this->estimator_module->disabled = true;
      }
      this->tracker_module->track();
    }
    else if (this->tracker_module->lost_it && !this->tracker_module->started)
    {
      //The object is lost...  what now!? Lets try to find it
      this->tracker_module->find_object_in_scene();
    }
    else
    {
      //Tracker is not started
      if (this->en_estimator && this->estimator_module)
      {
        //Re-enable Estimator if it was disabled, tracker is not tracking anymore
        this->estimator_module->disabled = false;
      }
    }
    ++count_to_spam;
    this->tracker_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10)); //tracker could try to go as fast as possible
  }
  //tracker got stopped
  return;
}

void VisionNode::spin_listener()
{
  int count_to_table (0);
  ROS_INFO("[Listener] Listener module will try to read Vito Robot arms and hands transformations to perform hands/arms cropping on the processed scene.");
  //instant fetch of table transform, it will refetch it later
  listener_module->listen_table();
  this->listener_module->spin_once();
  //spin until we disable it or it dies somehow
  while (this->en_listener && this->listener_module)
  {
    if (count_to_table > 1000)
    {
      //Re-read table, as a precaution, but it should not have been changed
      listener_module->listen_table();
      count_to_table = 0;
    }
    //do the listening
    this->listener_module->listen_once();
    //spin
    this->listener_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50)); //listener could try to go at 20Hz
    ++count_to_table;
  }
  //listener got stopped
  return;
}

//dynamic reconfigure callback
void VisionNode::cb_reconfigure(pacman_vision::pacman_visionConfig &config, uint32_t level)
{
  //initial gui init based on params (just do it once)
  if (rqt_init)
  {
    config.enable_estimator = en_estimator;
    config.enable_broadcaster = en_broadcaster;
    config.enable_listener = en_listener;
    config.enable_tracker= en_tracker;
    config.downsampling = downsample;
    config.passthrough = filter;
    config.plane_segmentation = plane;
    config.plane_tolerance = plane_tol;
    config.keep_organized = keep_organized;
    config.leaf_size = leaf;
    config.pass_xmax = limits->x2;
    config.pass_xmin = limits->x1;
    config.pass_ymax = limits->y2;
    config.pass_ymin = limits->y1;
    config.pass_zmax = limits->z2;
    config.pass_zmin = limits->z1;
    config.point_cloud_resolution = kinect2_resolution;
    //subscribe to pointcloud topic
    std::string topic;
    if (kinect2_resolution == 2)
      topic = nh.resolveName("/kinect2/hd/points");
    else if (kinect2_resolution == 1)
      topic = nh.resolveName("/kinect2/qhd/points");
    else if (kinect2_resolution == 0)
      topic = nh.resolveName("/kinect2/sd/points");
    else //unhandled default to sd
      topic = nh.resolveName("/kinect2/sd/points");
    sub_kinect = nh.subscribe(topic, 2, &VisionNode::cb_kinect, this);

    config.tracker_disturbance = false;
    //listener
    nh.getParam("listener_crop_right_arm", config.listener_crop_right_arm);
    nh.getParam("listener_crop_left_arm", config.listener_crop_left_arm);
    nh.getParam("listener_crop_right_hand", config.listener_crop_right_hand);
    nh.getParam("listener_crop_left_hand", config.listener_crop_left_hand);
    //estimator
    nh.getParam("estimator_object_calibration", config.estimator_object_calibration);
    nh.getParam("estimator_iterations", config.estimator_iterations);
    nh.getParam("estimator_neighbors", config.estimator_neighbors);
    nh.getParam("estimator_clus_tol", config.estimator_clus_tol);
    //broadcaster
    nh.getParam("broadcaster_tf", config.broadcaster_tf);
    nh.getParam("broadcaster_rviz_markers", config.broadcaster_rviz_markers);
    //tracker
    nh.getParam("tracker_estimation_type", config.tracker_estimation_type);
    this->rqt_init = false;
    ROS_WARN("[PaCMaN Vision] Rqt-Reconfigure Default Values Initialized");
  }
  this->en_estimator = config.enable_estimator;
  this->en_tracker = config.enable_tracker;
  this->en_broadcaster = config.enable_broadcaster;
  this->en_listener = config.enable_listener;
  this->filter = config.passthrough;
  this->plane = config.plane_segmentation;
  this->plane_tol = config.plane_tolerance;
  this->downsample = config.downsampling;
  this->keep_organized = config.keep_organized;
  this->limits->x1 = config.pass_xmin;
  this->limits->x2 = config.pass_xmax;
  this->limits->y1 = config.pass_ymin;
  this->limits->y2 = config.pass_ymax;
  this->limits->z1 = config.pass_zmin;
  this->limits->z2 = config.pass_zmax;
  this->leaf = config.leaf_size;
  if (this->kinect2_resolution != config.point_cloud_resolution)
  {
    this->kinect2_resolution = config.point_cloud_resolution;
    std::string topic;
    if (kinect2_resolution == 2)
      topic = nh.resolveName("/kinect2/hd/points");
    else if (kinect2_resolution == 1)
      topic = nh.resolveName("/kinect2/qhd/points");
    else if (kinect2_resolution == 0)
      topic = nh.resolveName("/kinect2/sd/points");
    else //unhandled default to sd
      topic = nh.resolveName("/kinect2/sd/points");
    //resubscribe (also kills previous subscription)
    sub_kinect = nh.subscribe(topic, 2, &VisionNode::cb_kinect, this);
  }
  if (this->listener_module && this->en_listener)
  {
    this->crop_r_arm = config.listener_crop_right_arm;
    this->crop_l_arm = config.listener_crop_left_arm;
    this->crop_r_hand = config.listener_crop_right_hand;
    this->crop_l_hand = config.listener_crop_left_hand;
  }
  if (this->estimator_module && this->en_estimator)
  {
    this->estimator_module->calibration = config.estimator_object_calibration;
    this->estimator_module->iterations = config.estimator_iterations;
    this->estimator_module->pe.setParam("progItera",estimator_module->iterations);
    this->estimator_module->neighbors = config.estimator_neighbors;
    this->estimator_module->pe.setParam("kNeighbors",estimator_module->neighbors);
    this->estimator_module->clus_tol = static_cast<double>(config.estimator_clus_tol);
  }
  if (this->broadcaster_module && this->en_broadcaster)
  {
    this->broadcaster_module->obj_tf = config.broadcaster_tf;
    this->broadcaster_module->rviz_markers = config.broadcaster_rviz_markers;
  }
  if (this->tracker_module && this->en_tracker)
  {
    this->tracker_module->type = config.tracker_estimation_type;
    if (config.tracker_disturbance)
    {
      tracker_module->manual_disturbance = true;
      config.tracker_disturbance = false;
    }
  }
  ROS_INFO("[PaCMaN Vision] Reconfigure request accepted");
}

void VisionNode::spin_once()
{
    ros::spinOnce();
    this->check_modules();
}

void VisionNode::shutdown()
{
  en_tracker = en_broadcaster = en_estimator = en_listener = false;
  this->check_modules();
  boost::this_thread::sleep(boost::posix_time::seconds(1));
}

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "pacman_vision");
  VisionNode node;
  ros::Rate rate(30); //try to go at 30hz
  while (node.nh.ok())
  {
    node.spin_once();
    rate.sleep();
  }
  node.shutdown();
  return 0;
}
