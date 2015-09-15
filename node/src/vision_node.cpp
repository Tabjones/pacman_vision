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
  pub_scene = nh.advertise<PC> ("scene_processed", 5);
  //init node params
  nh.param<bool>("enable_estimator", en_estimator, false);
  nh.param<bool>("enable_tracker", en_tracker, false);
  nh.param<bool>("enable_broadcaster", en_broadcaster, false);
  nh.param<bool>("enable_listener", en_listener, false);
  nh.param<bool>("enable_supervoxels", en_supervoxels, false);
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
  nh.param<double>("downsample_leaf_size", leaf, 0.01);
  nh.param<bool>("crop_right_arm", crop_r_arm, false);
  nh.param<bool>("crop_left_arm", crop_l_arm, false);
  nh.param<bool>("crop_right_hand", crop_r_hand, false);
  nh.param<bool>("crop_left_hand", crop_l_hand, false);
  nh.param<bool>("use_table_transform", use_table_trans, false);
  nh.param<bool>("use_kinect2", use_kinect2, true);
  nh.param<bool>("Master_Disable", disabled, false);
  nh.param<double>("plane_tolerance", plane_tol, 0.004);
  //set callback for dynamic reconfigure
  this->dyn_srv.setCallback(boost::bind(&VisionNode::cb_reconfigure, this, _1, _2));
}

bool VisionNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res)
{
  if (!disabled)
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
  else
  {
    ROS_WARN("[PaCMaN Vision][%s] Node is globally disabled, renable it!", __func__);
    return false;
  }
}

void VisionNode::cb_kinect(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  if(!disabled)
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
      if (use_table_trans)
      {
        this->storage->read_table(table_trans);
        if (table_trans)
        {
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
    //crop arms if listener is active and we set it
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
        if( this->storage->read_left_arm(left_arm) )
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
          //left6shared_ptr<
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
      }
      if (crop_r_arm)
      {
        if ( this->storage->read_right_arm(right_arm) )
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
      }
      if (crop_r_hand)
      {
        //right hand //TODO hand measures
        this->storage->read_right_hand(right_hand);
        if (right_hand)
        {
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
      }
      if (crop_l_hand)
      {
        //left_hand //TODO hand measures
        this->storage->read_left_hand(left_hand);
        if (left_hand)
        {
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
}

void VisionNode::check_modules()
{
  //check if we want estimator module and it is not started, or if it is started but we want it disabled
  if (this->en_estimator && !this->estimator_module && !disabled)
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
  if (this->en_broadcaster && !this->broadcaster_module && !disabled)
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
  if (this->en_tracker && !this->tracker_module && !disabled)
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
  if (this->en_listener && !this->listener_module && !disabled)
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

  //check if we want supervoxels module and it is not started, or if it is started but we want it disabled
  if (this->en_supervoxels && !this->supervoxels_module && !disabled)
  {
    ROS_WARN("[PaCMaN Vision] Started Supervoxels module");
    this->supervoxels_module.reset( new Supervoxels(this->nh, this->storage) );
    //spawn a thread to handle the module spinning
    supervoxels_driver = boost::thread(&VisionNode::spin_supervoxels, this);
  }
  else if (!this->en_supervoxels && this->supervoxels_module)
  {
    ROS_WARN("[PaCMaN Vision] Stopped Supervoxels module");
    //wait for the thread to stop, if not already, if already stopped this results in a no_op
    supervoxels_driver.join();
    //kill the module
    this->supervoxels_module.reset();
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
    ++count_to_spam;
    boost::this_thread::sleep(boost::posix_time::milliseconds(100)); //estimator could try to go at 10Hz (no need to process those services faster)
  }
  //estimator got stopped
  return;
}

void VisionNode::spin_broadcaster()
{
  ROS_INFO("[Broadcaster] Broadcaster module will publish tfs and markers calculated by the other modules or the base node itself.");
  ROS_INFO("[Broadcaster] Look at the namespace of markers in the MarkerArray.");
  //spin until we disable it or it dies somehow
  while (this->en_broadcaster && this->broadcaster_module)
  {
    //Clear previous markers
    broadcaster_module->markers.markers.clear();
    //Check if we have to publish estimated objects or tracked one
    if ( (this->en_estimator && this->estimator_module) || (this->tracker_module && this->en_tracker) )
    {
      if (broadcaster_module->obj_markers || broadcaster_module->obj_tf || broadcaster_module->tracker_bb)
        //this takes care of markers and TFs of all pose estimated objects, plus tracked object and its bounding box
        broadcaster_module->elaborate_estimated_objects();
    }

    //publish Passthrough filter limits as a box
    if(limits && filter && broadcaster_module->pass_limits)
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

void VisionNode::spin_supervoxels()
{
  ROS_INFO("[Supervoxels] Supervoxels module will segment processed scene into SuperVoxel clusters and republish it.");
  //spin until we disable it or it dies somehow
  while (this->en_supervoxels && this->supervoxels_module)
  {
    if (!this->supervoxels_module->serviced)
      this->supervoxels_module->clustering();

    //spin
    this->supervoxels_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50)); //Supervoxels can try to go at 20Hz
  }
  //supervoxels got stopped
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
    config.enable_supervoxels = en_supervoxels;
    config.kinect2_resolution = kinect2_resolution;
    config.use_kinect2 = use_kinect2;
    config.Master_Disable = disabled;
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
    //subscribe to pointcloud topic
    std::string topic;
    if (use_kinect2)
    {
      if (kinect2_resolution == 2)
        topic = nh.resolveName("/kinect2/hd/points");
      else if (kinect2_resolution == 1)
        topic = nh.resolveName("/kinect2/qhd/points");
      else if (kinect2_resolution == 0)
        topic = nh.resolveName("/kinect2/sd/points");
      else //unhandled default to sd
        topic = nh.resolveName("/kinect2/sd/points");
      sensor_ref_frame = "/kinect2_rgb_optical_frame";
      this->storage->write_sensor_ref_frame(sensor_ref_frame);
    }
    else
    {
      topic = nh.resolveName("/camera/depth_registered/points");
      sensor_ref_frame = "/camera_rgb_optical_frame";
      this->storage->write_sensor_ref_frame(sensor_ref_frame);
    }
    sub_kinect = nh.subscribe(topic, 5, &VisionNode::cb_kinect, this);

    //listener
    nh.getParam("crop_right_arm", config.groups.listener_module.crop_right_arm);
    nh.getParam("crop_left_arm", config.groups.listener_module.crop_left_arm);
    nh.getParam("crop_right_hand", config.groups.listener_module.crop_right_hand);
    nh.getParam("crop_left_hand", config.groups.listener_module.crop_left_hand);
    nh.getParam("use_table_transform", config.groups.listener_module.use_table_transform);
    //estimator
    nh.getParam("object_calibration", config.groups.estimator_module.object_calibration);
    nh.getParam("iterations", config.groups.estimator_module.iterations);
    nh.getParam("neighbors", config.groups.estimator_module.neighbors);
    nh.getParam("cluster_tol", config.groups.estimator_module.cluster_tol);
    //broadcaster
    nh.getParam("publish_tf", config.groups.broadcaster_module.publish_tf);
    nh.getParam("estimated_objects", config.groups.broadcaster_module.estimated_objects);
    nh.getParam("passthrough_limits", config.groups.broadcaster_module.passthrough_limits);
    nh.getParam("tracker_bounding_box", config.groups.broadcaster_module.tracker_bounding_box);
    //tracker
    config.groups.tracker_module.tracker_disturbance = false;
    nh.setParam("tracker_disturbance", false);
    nh.getParam("estimation_type", config.groups.tracker_module.estimation_type);
    //Supervoxels
    nh.getParam("use_service", config.groups.supervoxels_module.use_service);
    nh.getParam("voxel_resolution", config.groups.supervoxels_module.voxel_resolution);
    nh.getParam("seed_resolution", config.groups.supervoxels_module.seed_resolution);
    nh.getParam("color_importance", config.groups.supervoxels_module.color_importance);
    nh.getParam("spatial_importance", config.groups.supervoxels_module.spatial_importance);
    nh.getParam("normal_importance", config.groups.supervoxels_module.normal_importance);
    nh.getParam("refinement_iterations", config.groups.supervoxels_module.refinement_iterations);
    nh.getParam("normals_search_radius", config.groups.supervoxels_module.normals_search_radius);
    //Finish gui initialization
    this->rqt_init = false;
    ROS_WARN("[PaCMaN Vision] Rqt-Reconfigure Default Values Initialized");
  }
  //Normal behaviour
  this->en_estimator    = config.enable_estimator;
  this->en_tracker      = config.enable_tracker;
  this->en_broadcaster  = config.enable_broadcaster;
  this->en_listener     = config.enable_listener;
  this->en_supervoxels  = config.enable_supervoxels;
  //Global Node Disable
  if (this->disabled != config.Master_Disable)
  {
    disabled = config.Master_Disable;
    if (disabled)
    {//disable everything
      config.enable_estimator = config.enable_tracker = config.enable_listener =
        config.enable_broadcaster = config.enable_supervoxels = false;
      en_estimator = en_tracker = en_broadcaster = en_listener = en_supervoxels = false;
      this->sub_kinect.shutdown();
    }
  }
  if (config.Master_Disable)
  {
    config.enable_estimator = config.enable_tracker = config.enable_listener
      = config.enable_broadcaster = config.enable_supervoxels = false;
    en_estimator = en_tracker = en_broadcaster = en_listener = en_supervoxels = false;
  }
  //filters
  this->downsample      = config.groups.base_node_filters.downsampling;
  this->filter          = config.groups.base_node_filters.passthrough;
  this->plane           = config.groups.base_node_filters.plane_segmentation;
  this->plane_tol       = config.groups.base_node_filters.plane_tolerance;
  this->keep_organized  = config.groups.base_node_filters.keep_organized;
  this->leaf            = config.groups.base_node_filters.downsample_leaf_size;
  this->limits->x2      = config.groups.base_node_filters.pass_xmax;
  this->limits->x1      = config.groups.base_node_filters.pass_xmin;
  this->limits->y2      = config.groups.base_node_filters.pass_ymax;
  this->limits->y1      = config.groups.base_node_filters.pass_ymin;
  this->limits->z2      = config.groups.base_node_filters.pass_zmax;
  this->limits->z1      = config.groups.base_node_filters.pass_zmin;
  //handle which kinect topic
  if (!disabled)
  {
    if (this->use_kinect2 != config.use_kinect2)
    {
      this->use_kinect2 = config.use_kinect2;
      std::string topic;
      if (!use_kinect2)
      {
        topic = nh.resolveName("/camera/depth_registered/points");
        sub_kinect = nh.subscribe(topic, 2, &VisionNode::cb_kinect, this);
        sensor_ref_frame = "/camera_rgb_optical_frame";
        this->storage->write_sensor_ref_frame(sensor_ref_frame);
      }
      else
      {
        this->kinect2_resolution = config.kinect2_resolution;
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
        sensor_ref_frame = "/kinect2_rgb_optical_frame";
        this->storage->write_sensor_ref_frame(sensor_ref_frame);
      }
    }
    if (this->kinect2_resolution != config.kinect2_resolution)
    {
      this->kinect2_resolution = config.kinect2_resolution;
      if (config.use_kinect2)
      {
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
    }
  }
  //Listener Module
  if (this->listener_module && this->en_listener)
  {
    this->crop_r_arm    = config.groups.listener_module.crop_right_arm;
    this->crop_l_arm    = config.groups.listener_module.crop_left_arm;
    this->crop_r_hand   = config.groups.listener_module.crop_right_hand;
    this->crop_l_hand   = config.groups.listener_module.crop_left_hand;
    this->use_table_trans   = config.groups.listener_module.use_table_transform;
    this->listener_module->listen_left_arm = this->crop_l_arm;
    this->listener_module->listen_right_arm = this->crop_r_arm;
    this->listener_module->listen_left_hand = this->crop_l_hand;
    this->listener_module->listen_right_hand = this->crop_r_hand;
  }
  //Estimator Module
  if (this->estimator_module && this->en_estimator)
  {
    this->estimator_module->calibration = config.groups.estimator_module.object_calibration;
    this->estimator_module->iterations  = config.groups.estimator_module.iterations;
    this->estimator_module->neighbors   = config.groups.estimator_module.neighbors;
    this->estimator_module->clus_tol    = config.groups.estimator_module.cluster_tol;
    this->estimator_module->pe.setParam("kNeighbors",estimator_module->neighbors);
    this->estimator_module->pe.setParam("progItera",estimator_module->iterations);
  }
  //Broadcaster Module
  if (this->broadcaster_module && this->en_broadcaster)
  {
    this->broadcaster_module->obj_tf      = config.groups.broadcaster_module.publish_tf;
    this->broadcaster_module->obj_markers = config.groups.broadcaster_module.estimated_objects;
    this->broadcaster_module->pass_limits = config.groups.broadcaster_module.passthrough_limits;
    this->broadcaster_module->tracker_bb  = config.groups.broadcaster_module.tracker_bounding_box;
  }
  //Tracker Module
  if (this->tracker_module && this->en_tracker)
  {
    this->tracker_module->type = config.groups.tracker_module.estimation_type;
    if (config.groups.tracker_module.tracker_disturbance)
    {
      tracker_module->manual_disturbance = true;
      config.groups.tracker_module.tracker_disturbance = false;
    }
  }
  //Supervoxels Module
  if (this->supervoxels_module && this->en_supervoxels)
  {
    this->supervoxels_module->serviced   = config.groups.supervoxels_module.use_service;
    this->supervoxels_module->voxel_res  = config.groups.supervoxels_module.voxel_resolution;
    this->supervoxels_module->seed_res   = config.groups.supervoxels_module.seed_resolution;
    this->supervoxels_module->color_imp  = config.groups.supervoxels_module.color_importance;
    this->supervoxels_module->spatial_imp= config.groups.supervoxels_module.spatial_importance;
    this->supervoxels_module->normal_imp = config.groups.supervoxels_module.normal_importance;
    this->supervoxels_module->num_iterations = config.groups.supervoxels_module.refinement_iterations;
    this->supervoxels_module->normal_radius = config.groups.supervoxels_module.normals_search_radius;
  }
  ROS_INFO("[PaCMaN Vision] Reconfigure request executed");
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
