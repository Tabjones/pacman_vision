#include "pacman_vision/vision_node.h"

VisionNode::VisionNode()
{
  this->nh = ros::NodeHandle("pacman_vision");
  this->scene.reset(new PC);
  this->scene_processed.reset(new PC);
  this->storage.reset(new Storage);
  //first call of dynamic reconfigure callback will only set gui to loaded parameters
  rqt_init = true;
  //service callback init
  srv_get_scene = nh.advertiseService("get_scene_processed", &VisionNode::cb_get_scene, this);
  pub_scene = nh.advertise<PC> ("scene_processed", 3);
  table_trans.setIdentity();

  crop_r_arm = crop_l_arm = crop_r_hand = crop_l_hand = false;
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
  nh.param<double>("pass_xmax", xmax, 0.5);
  nh.param<double>("pass_xmin", xmin, -0.5);
  nh.param<double>("pass_ymax", ymax, 0.5);
  nh.param<double>("pass_ymin", ymin, -0.5);
  nh.param<double>("pass_zmax", zmax, 1.0);
  nh.param<double>("pass_zmin", zmin, 0.3);
  nh.param<double>("leaf_size", leaf, 0.01);
  nh.param<double>("plane_tolerance", plane_tol, 0.004);
  //set callback for dynamic reconfigure
  this->dyn_srv.setCallback(boost::bind(&VisionNode::cb_reconfigure, this, _1, _2));
}

bool VisionNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res)
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

void VisionNode::cb_kinect(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  pcl::fromROSMsg (*message, *(this->scene));
  // Save untouched scene into storage
  this->storage->write_scene(this->scene);
  PC::Ptr tmp (new PC);
  //filters
  //check if we need to filter scene
  if (filter)
  {
    PassThrough<PT> pass;
    //check if have a listener and thus a table transform and hands
    if (en_listener && listener_module)
    {
      pcl::CropBox<PT> cb;
      Eigen::Matrix4f inv_trans;
      inv_trans = table_trans.inverse();
      //check if we need to maintain scene organized
      if (keep_organized)
        cb.setKeepOrganized(true);
      else
        cb.setKeepOrganized(false);
      cb.setInputCloud (this->scene);
      Eigen::Vector4f min,max;
      //hardcoded table dimensions TODO make it dynamic
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
      pass.setFilterLimits (zmin, zmax);
      pass.filter (*tmp);
      pass.setInputCloud (tmp);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (ymin, ymax);
      pass.filter (*t);
      pass.setInputCloud (t);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (xmin, xmax);
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
    if (crop_l_arm)
    {
      //left2
      cb.setInputCloud(input);
      min << -0.06, -0.06, -0.06, 1;
      max << 0.06, 0.0938, 0.1915, 1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_2.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter (*(tmp));
      //left3
      cb.setInputCloud(tmp);
      min << -0.06,-0.06,0,1;
      max << 0.06,0.0938,0.2685,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_3.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //left4
      cb.setInputCloud(input);
      min << -0.06,-0.0938,-0.06,1;
      max << 0.06,0.06,0.1915,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_4.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //left5
      cb.setInputCloud(tmp);
      min << -0.06,-0.0555,0,1;
      max << 0.06,0.06,0.2585,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_5.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //left6
      cb.setInputCloud(input);
      min << -0.0711,-0.0555,-0.0711,1;
      max << 0.0711,0.0795,0.057,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_6.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //left7
      cb.setInputCloud(tmp);
      min << -0.04,-0.0399,-0.031,1;
      max << 0.04,0.0399,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_7.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      if (!crop_r_arm && !crop_r_hand && !crop_l_hand)
        pcl::copyPointCloud(*input, *(this->scene_processed));
    }
    if (crop_r_arm)
    {
      //right2
      cb.setInputCloud(input);
      min << -0.06, -0.06, -0.06, 1;
      max << 0.06, 0.0938, 0.1915, 1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_2.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //right3
      cb.setInputCloud(tmp);
      min << -0.06,-0.06,0,1;
      max << 0.06,0.0938,0.2685,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_3.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //right4
      cb.setInputCloud(input);
      min << -0.06,-0.0938,-0.06,1;
      max << 0.06,0.06,0.1915,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_4.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //right5
      cb.setInputCloud(tmp);
      min << -0.06,-0.0555,0,1;
      max << 0.06,0.06,0.2585,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_5.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      //right6
      cb.setInputCloud(input);
      min << -0.0711,-0.0555,-0.0711,1;
      max << 0.0711,0.0795,0.057,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_6.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      //right7
      cb.setInputCloud(tmp);
      min << -0.04,-0.0399,-0.031,1;
      max << 0.04,0.0399,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_7.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*input);
      if (!crop_r_hand && !crop_l_hand)
        pcl::copyPointCloud(*input, *(this->scene_processed));
    }
    if (crop_r_hand)
    {
      //right hand //TODO hand measures
      cb.setInputCloud(input);
      min << 0,0,0,1;
      max << 0,0,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = right_hand.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*tmp);
      if (!crop_l_hand)
        pcl::copyPointCloud(*tmp, *(this->scene_processed));
    }
    if (crop_l_hand)
    {
      //left_hand
      if (crop_r_hand)
        cb.setInputCloud(tmp);
      else
        cb.setInputCloud(input);
      min << 0,0,0,1;
      max << 0,0,0,1;
      cb.setMin(min);
      cb.setMax(max);
      cb.setNegative(true); //crop what's inside
      inv_trans = left_hand.inverse();
      cb.setTransform(Eigen::Affine3f(inv_trans));
      cb.filter(*(this->scene_processed));
    }
  }
  //republish processed cloud
  /* |passt   | voxelgrid   |segment | | arms or hands croppings                                 */
  if (filter || downsample || plane || ((crop_l_arm || crop_r_arm || crop_r_hand || crop_l_hand) && en_listener && listener_module) )
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
    this->estimator_module.reset( new Estimator(this->nh) );
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
    this->broadcaster_module.reset( new Broadcaster(this->nh) );
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
    this->tracker_module.reset( new Tracker(this->nh) );
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
    this->listener_module.reset( new Listener(this->nh) );
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
  //spin until we disable it or it dies somehow
  while (this->en_estimator && this->estimator_module)
  {
    //push down filtered cloud to estimator
    //lock variables so no-one else can touch what is copied
    mtx_scene.lock();
    mtx_estimator.lock();
    if(this->filter || this->downsample || this->plane)
    {
      copyPointCloud(*(this->scene_processed), *(this->estimator_module->scene));
    }
    else
    {
      ROS_WARN("[PaCMaN Vision] Estimator module will not function properly without scene filtering, enable at least plane segmentation");
      copyPointCloud(*scene, *(this->estimator_module->scene));
    }
    //release lock
    mtx_estimator.unlock();
    mtx_scene.unlock();
    this->estimator_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50)); //estimator could try to go at 20Hz (no need to process those services faster)
  }
  //estimator got stopped
  return;
}

void VisionNode::spin_broadcaster()
{
  //spin until we disable it or it dies somehow
  while (this->en_broadcaster && this->broadcaster_module)
  {
    //check if we have an estimator and or a tracker
    if (this->en_estimator && this->estimator_module)
    { //we have an estimator module running
      //check if it is not busy computing and it has some new estimations to upload
      if (!this->estimator_module->busy && this->estimator_module->up_broadcaster)
      {
        mtx_estimator.lock();
        mtx_broadcaster.lock();
        broadcaster_module->estimated.clear();
        broadcaster_module->names.clear();
        broadcaster_module->ids.clear();
        boost::copy(estimator_module->estimations, back_inserter(broadcaster_module->estimated));
        boost::copy(estimator_module->names, back_inserter(broadcaster_module->names));
        boost::copy(estimator_module->ids, back_inserter(broadcaster_module->ids));
        mtx_broadcaster.unlock();
        estimator_module->up_broadcaster = false;
        mtx_estimator.unlock();
        //process new data
        broadcaster_module->compute_transforms();
      }
    }
    //do the broadcasting
    this->broadcaster_module->broadcast_once();
    //spin
    this->broadcaster_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50)); //broadcaster could try to go at 20Hz
  }
  //broadcaster got stopped
  return;
}

void VisionNode::spin_tracker()
{
  //spin until we disable it or it dies somehow
  while (this->en_tracker && this->tracker_module)
  {
    //push down filtered cloud to tracker
    //lock variables so no-one else can touch what is copied
    mtx_scene.lock();
    mtx_tracker.lock();
    if (filter || downsample || plane)
    {
      copyPointCloud(*(this->scene_processed), *(this->tracker_module->scene));
      this->tracker_module->leaf = this->leaf;
    }
    else
    {
      ROS_WARN("[PaCMaN Vision] Tracker module will not function properly without scene filtering, enable at least downsampling");
      copyPointCloud(*(this->scene), *(this->tracker_module->scene));
      this->tracker_module->leaf = this->leaf;
    }
    //release lock
    mtx_tracker.unlock();
    mtx_scene.unlock();
    //check if we have an estimator running
    if (this->en_estimator && this->estimator_module)
    { //we have an estimator module running
      //check if it is not busy computing and it has some new estimations to upload
      if (!this->estimator_module->busy && this->estimator_module->up_tracker)
      {
        if (!tracker_module->started && !tracker_module->lost_it)
        { //we copy only if tracker is not started already, otherwise we dont care
          mtx_estimator.lock();
          mtx_tracker.lock();
          tracker_module->estimations.clear();
          tracker_module->names.clear();
          boost::copy(estimator_module->estimations, back_inserter(tracker_module->estimations));
          boost::copy(estimator_module->names, back_inserter(tracker_module->names));
          mtx_tracker.unlock();
          mtx_estimator.unlock();
        }
        estimator_module->up_tracker=false;
      }
    }
    //spin
    if (this->tracker_module->started)
    {
      if (tracker_module->to_estimator && this->en_estimator && this->estimator_module)
      {
        if (!this->estimator_module->busy)
        {
          mtx_estimator.lock();
        //TODO FIx this wont work if user disable estimator during tracking and then enables it again (it will segfault!)
          this->estimator_module->estimations.erase(estimator_module->estimations.begin() + tracker_module->index);
          this->estimator_module->names.erase(estimator_module->names.begin() + tracker_module->index);
          this->estimator_module->ids.erase(estimator_module->ids.begin() + tracker_module->index);
          this->estimator_module->up_broadcaster = true;
          mtx_estimator.unlock();
        }
        //if estimator was busy it is computing again so no point in forwarding tracker transform
        tracker_module->to_estimator = false;
      }
    //  boost::timer t;
      this->tracker_module->track();
    //  cout<<"Tracker Step: "<<t.elapsed()<<std::endl;
      this->tracker_module->broadcast_tracked_object();
    }
    else if (this->tracker_module->lost_it && !this->tracker_module->started)
    {
      //The object is lost...  what now!? Lets ask Vito where the hands are
      tracker_module->find_object_in_scene();
    }
    else if (!tracker_module->started && !tracker_module->lost_it && tracker_module->to_estimator)
    {
      if (this->en_estimator && this->estimator_module)
      {
        if (!estimator_module->busy)
        {
          mtx_estimator.lock();
          this->estimator_module->estimations.push_back(tracker_module->transform);
          this->estimator_module->names.push_back(tracker_module->name);
          this->estimator_module->ids.push_back(tracker_module->id);
          this->estimator_module->up_broadcaster = true;
          this->estimator_module->up_tracker = true;
          mtx_estimator.unlock();
        }
        tracker_module->to_estimator = false;
      }
    }
    this->tracker_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10)); //tracker could try to go at 100Hz
  }//end while
  if (this->estimator_module && this->en_estimator)
    this->estimator_module->up_tracker = true;
  //tracker got stopped
  return;
}

void VisionNode::spin_listener()
{
  //fetch table transform
  listener_module->listen_table();
  this->listener_module->spin_once();
  mtx_scene.lock();
  this->table_trans = listener_module->table;
  mtx_scene.unlock();
  //spin until we disable it or it dies somehow
  while (this->en_listener && this->listener_module)
  {
    //do the listening
    this->listener_module->listen_once();
    mtx_scene.lock();
    this->left_2 = listener_module->left_2;
    this->left_3 = listener_module->left_3;
    this->left_4 = listener_module->left_4;
    this->left_5 = listener_module->left_5;
    this->left_6 = listener_module->left_6;
    this->left_7 = listener_module->left_7;
    this->right_2 = listener_module->right_2;
    this->right_3 = listener_module->right_3;
    this->right_4 = listener_module->right_4;
    this->right_5 = listener_module->right_5;
    this->right_6 = listener_module->right_6;
    this->right_7 = listener_module->right_7;
    this->right_hand = listener_module->right_hand;
    this->left_hand = listener_module->left_hand;
    mtx_scene.unlock();
    //spin
    this->listener_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50)); //listener could try to go at 20Hz
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
    config.pass_xmax = xmax;
    config.pass_xmin = xmin;
    config.pass_ymax = ymax;
    config.pass_ymin = ymin;
    config.pass_zmax = zmax;
    config.pass_zmin = zmin;
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
  this->xmin = config.pass_xmin;
  this->xmax = config.pass_xmax;
  this->ymin = config.pass_ymin;
  this->ymax = config.pass_ymax;
  this->zmin = config.pass_zmin;
  this->zmax = config.pass_zmax;
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
    this->broadcaster_module->tf = config.broadcaster_tf;
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
  check_modules();
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
