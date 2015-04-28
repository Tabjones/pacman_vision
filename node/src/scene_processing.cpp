#include "pacman_vision/processor.h"
///////////////////
//Processor Class//
///////////////////

//Constructor
Processor::Processor(NHPtr nhptr)
{
  this->nh_ptr.reset();
  this->nh_ptr = nhptr;
  PointCloud<PointXYZRGBA> a;
  scene_processed = a.makeShared();

  //service callbacks
  srv_get_scene = nh_ptr->advertiseService("get_scene", &Processor::cb_get_scene, this);

  //subscribe to depth_registered pointclouds topic
  std::string topic = nh_ptr->resolveName("/camera/depth_registered/points");
  sub_openni = nh_ptr->subscribe(topic, 10, &Processor::cb_openni, this);
  pub_scene = nh_ptr->advertise<sensor_msgs::PointCloud2> ("/pacman_vision/processor/scene", 10);

  //load parameters
  nh_ptr->param<bool>("/pacman_vision/processor/perform_filtering", filter, "false");
  nh_ptr->param<bool>("/pacman_vision/processor/perform_downsampling", downsample, "false");
  nh_ptr->param<bool>("/pacman_vision/processor/keep_organized", keep_organized, "false");
  nh_ptr->param<double>("/pacman_vision/processor/filter_xmin", xmin, -100);
  nh_ptr->param<double>("/pacman_vision/processor/filter_xmax", xmax, 100);
  nh_ptr->param<double>("/pacman_vision/processor/filter_ymin", ymin, -100);
  nh_ptr->param<double>("/pacman_vision/processor/filter_ymax", ymax, 100);
  nh_ptr->param<double>("/pacman_vision/processor/filter_zmin", zmin, -100);
  nh_ptr->param<double>("/pacman_vision/processor/filter_zmax", zmax, 100);
  nh_ptr->param<double>("/pacman_vision/processor/downsampling_leaf_size", leaf, 0.005);
}

bool Processor::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scene_processed, msg);
  res.scene = msg; 
  ROS_INFO("[Processor][%s] Sent processed scene to service response", __func__);
  return true;
}

void Processor::cb_openni(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  PointCloud<PointXYZRGBA>::Ptr tmp (new PointCloud<PointXYZRGBA>);
  fromROSMsg (*message, *tmp);

  //check if we need to filter scene
  if (filter)
  {
    PassThrough<PointXYZRGBA> pass;
    //check if we need to maintain scene organized
    if (keep_organized)
      pass.setKeepOrganized(true);
    else
      pass.setKeepOrganized(false);
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (zmin, zmax);
    pass.filter (*tmp);
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ymin, ymax);
    pass.filter (*tmp);
    pass.setInputCloud (tmp);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xmin, xmax);
    pass.filter (*tmp);
  }
  //check if we need to downsample scene
  if (downsample) //cannot keep organized cloud after voxelgrid
  {
    VoxelGrid<PointXYZRGBA> vg;
    vg.setInputCloud (tmp);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter (*tmp);
  }
  pcl::copyPointCloud(*tmp, *scene_processed);
}

