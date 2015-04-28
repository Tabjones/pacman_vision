#include "pacman_vision/processor.h"
///////////////////
//Processor Class//
///////////////////

//Constructor
Processor::Processor(ros::NodeHandle &n)
{
  this->nh = ros::NodeHandle(n,"processor");
  PointCloud<PointXYZRGBA> a;
  scene_processed = a.makeShared();

  //service callbacks
  srv_get_scene = nh.advertiseService("get_scene", &Processor::cb_get_scene, this);

  //subscribe to depth_registered pointclouds topic
  std::string topic = "/camera/depth_registered/points";
  sub_openni = nh.subscribe(topic, 10, &Processor::cb_openni, this);
  pub_scene = nh.advertise<sensor_msgs::PointCloud2> ("scene", 10);

  //init filter params
  filter = downsample = keep_organized = false;
  xmin = ymin = zmin = -0.5;
  xmax = ymax = zmax = 0.5;
  leaf = 0.005;
  //dynamic reconfigure
  //this->dyn_srv.setCallback(boost::bind(&Processor::cb_reconfigure, this, _1, _2));
}

Processor::~Processor()
{
}
/*
void Processor::cb_reconfigure(pacman_vision::processorConfig &config, uint32_t level)
{
  ROS_WARN("Processor dyn recon callback!!");
  filter = config.filter;
}
*/
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
  std::cout<<"cb "<<std::endl;
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
  copyPointCloud(*tmp, *scene_processed);
  std::cout<<"cb end"<< std::endl;
}

