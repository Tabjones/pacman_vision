#include "pacman_vision/vision_node.h"

VisionNode::VisionNode()
{
  this->nh = ros::NodeHandle("pacman_vision");
  this->dyn_srv.setCallback(boost::bind(&VisionNode::cb_reconfigure, this, _1, _2));
  this->scene.reset(new PointCloud<PointXYZRGBA>);
  this->scene_processed.reset(new PointCloud<PointXYZRGBA>);
  this->scene_filtered.reset(new PointCloud<PointXYZRGBA>);
  rqt_init = true;
  //service callback init
  srv_get_scene = nh.advertiseService("get_scene_processed", &VisionNode::cb_get_scene, this);

  //subscribe to depth_registered pointclouds topic
  std::string topic =nh.resolveName("/camera/depth_registered/points");
  sub_openni = nh.subscribe(topic, 3, &VisionNode::cb_openni, this);
  pub_scene = nh.advertise<PointCloud<PointXYZRGBA> > ("scene_processed", 3);

  //init filter params 
  nh.param<bool>("downsampling", downsample, false);
  nh.param<bool>("enable_estimator", en_estimator, false);
  nh.param<bool>("enable_tracker", en_tracker, false);
  nh.param<bool>("filtering", filter, true);
  nh.param<bool>("keep_organized", keep_organized, false);
  nh.param<double>("leaf_size", leaf, 0.01);
  nh.param<double>("pass_xmax", xmax, 0.5);
  nh.param<double>("pass_xmin", xmin, -0.5);
  nh.param<double>("pass_ymax", ymax, 0.5);
  nh.param<double>("pass_ymin", ymin, -0.5);
  nh.param<double>("pass_zmax", zmax, 1.0);
  nh.param<double>("pass_zmin", zmin, 0.3);
}

bool VisionNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scene_processed, msg);
  res.scene = msg; 
  ROS_INFO("[PaCMaN Vision][%s] Sent processed scene to service response.", __func__);
  return true;
}

void VisionNode::cb_openni(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  pcl::fromROSMsg (*message, *(this->scene));
  //filters
  //check if we need to filter scene
  if (filter)
  {
    PassThrough<PointXYZRGBA> pass;
    //check if we need to maintain scene organized
    if (keep_organized)
      pass.setKeepOrganized(true);
    else
      pass.setKeepOrganized(false);
    pass.setInputCloud (this->scene);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (zmin, zmax);
    pass.filter (*(this->scene_processed));
    pass.setInputCloud (this->scene_processed);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ymin, ymax);
    pass.filter (*(this->scene_processed));
    pass.setInputCloud (this->scene_processed);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xmin, xmax);
    pass.filter (*(this->scene_processed));
    copyPointCloud(*(this->scene_processed), *(this->scene_filtered));
  }
  //check if we need to downsample scene
  if (downsample) //cannot keep organized cloud after voxelgrid
  {
    VoxelGrid<PointXYZRGBA> vg;
    vg.setInputCloud (this->scene_filtered);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter (*(this->scene_processed));
  }
  //republish processed cloud
  if (filter || downsample)
    pub_scene.publish(*scene_processed);
  else
    pub_scene.publish(*scene);
}

void VisionNode::check_modules()
{
  //check if we want estimator module and it is not started, or if it is started but we want it disabled
  if (this->en_estimator && !this->estimator_module)
  {
    ROS_INFO("[PaCMaN Vision] Started Estimator module");
    this->estimator_module.reset( new Estimator(this->nh) );
    //spawn a thread to handle the module spinning
    estimator_driver = boost::thread(&VisionNode::spin_estimator, this);
  }
  else if (!this->en_estimator && this->estimator_module)
  {
    ROS_INFO("[PaCMaN Vision] Stopped Estimator module");
    //wait for the thread to stop, if not already, if already stopped this results in a no_op
    estimator_driver.join();
    //kill the module
    this->estimator_module.reset();
  }
}
void VisionNode::spin_estimator()
{
  //spin until we disable it or it dies somehow
  while (this->en_estimator && this->estimator_module)
  {
    //push down filtered cloud to estimator (never pass downsampled one, cause estimator will autodownsample cluster of objects with its own leaf size)
    if(filter)
      copyPointCloud(*scene_filtered, *(this->estimator_module->scene));
    else
      copyPointCloud(*scene, *(this->estimator_module->scene));
    this->estimator_module->spin_once();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100)); //estimator could try to go at 10Hz (no need to process those services faster)
  }
  //estimator got stopped
  return;
}

void VisionNode::cb_reconfigure(pacman_vision::pacman_visionConfig &config, uint32_t level)
{
  //initial gui init based on params (just do it once)
  if (rqt_init)
  {
    config.enable_estimator = en_estimator;
    config.downsampling = downsample;
    config.enable_tracker= en_tracker;
    config.filtering = filter;
    config.keep_organized = keep_organized;
    config.leaf_size = leaf;
    config.pass_xmax = xmax;
    config.pass_xmin = xmin;
    config.pass_ymax = ymax;
    config.pass_ymin = ymin;
    config.pass_zmax = zmax;
    config.pass_zmin = zmin;
    nh.getParam("estimator_clus_tol", config.estimator_clus_tol);
    nh.getParam("estimator_iterations", config.estimator_iterations);
    nh.getParam("estimator_neighbors", config.estimator_neighbors);
    nh.getParam("estimator_object_calibration", config.estimator_object_calibration);
    rqt_init = false;
    ROS_WARN("[PaCMaN Vision] rqt-reconfigure default values initialized");
  }
  this->en_estimator = config.enable_estimator;
  this->en_tracker = config.enable_tracker;
  this->filter = config.filtering;
  this->downsample = config.downsampling;
  this->keep_organized = config.keep_organized;
  this->xmin = config.pass_xmin;
  this->xmax = config.pass_xmax;
  this->ymin = config.pass_ymin;
  this->ymax = config.pass_ymax;
  this->zmin = config.pass_zmin;
  this->zmax = config.pass_zmax;
  this->leaf = config.leaf_size;
  if (this->estimator_module)
  {
    this->estimator_module->calibration = config.estimator_object_calibration;
    this->estimator_module->iterations = config.estimator_iterations;
    this->estimator_module->pe.setParam("progItera",estimator_module->iterations);
    this->estimator_module->neighbors = config.estimator_neighbors;
    this->estimator_module->pe.setParam("kNeighbors",estimator_module->neighbors);
    this->estimator_module->clus_tol = config.estimator_clus_tol;
  }
  ROS_WARN("[PaCMaN Vision] Reconfigure request accepted");
}

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "pacman_vision");
  VisionNode node;
  ros::Rate rate(100); //try to go at 100hz
  while (node.nh.ok())
  {
    ros::spinOnce(); 
    //check if we need to enable/disable other modules
    node.check_modules();
    rate.sleep();
  }
  return 0;
}
