// ROS headers
#include "scene_processing/scene_processing.h"
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// ROS generated headers
#include "scene_filter_node/acquire_scene.h" 

//general utilities
#include <string>
#include <stdlib.h>

class sceneAcquirer
{
  public:
    sceneAcquirer();
    //Node handle
    ros::NodeHandle nh;
  private:

    //Service Server
    ros::ServiceServer srv_acquire_;
    
    //Message Subscriber
    ros::Subscriber sub_stream_;
    
    //Message Publisher
    ros::Publisher pub_stream_;
    
    //Service callback, gets executed when service is called
    bool acquireScene(scene_filter_node::acquire_scene::Request& req, scene_filter_node::acquire_scene::Response& res);

    //Message callback, gets executed when a new message is available on topic
    void new_cloud_in_stream(const sensor_msgs::PointCloud2::ConstPtr& message);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_stream_;

    //parameters
    bool filter_, downsample_, keep_organized_;
    double xmin,xmax,ymin,ymax,zmin,zmax,leaf_;
};

//Constructor
sceneAcquirer::sceneAcquirer()
{
  nh = ros::NodeHandle("scene_filter_node");
  pcl::PointCloud<pcl::PointXYZRGBA> a;
  scene_stream_ = a.makeShared();

  //service callbacks
  srv_acquire_ = nh.advertiseService("acquire_scene", &sceneAcquirer::acquireScene, this);
  
  //subscribe to depth_registered pointclouds topic
  std::string topic = nh.resolveName("/camera/depth_registered/points");
  sub_stream_ = nh.subscribe(topic, 1, &sceneAcquirer::new_cloud_in_stream, this);

  pub_stream_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("/scene_filter/scene",1);

  //load parameters
  nh.param<bool>("/scene_filter/filter", filter_, "false");
  nh.param<bool>("/scene_filter/downsample", downsample_, "false");
  nh.param<bool>("/scene_filter/keep_organized", keep_organized_, "false");
  nh.param<double>("/scene_filter/xmin", xmin, -100);
  nh.param<double>("/scene_filter/xmax", xmax, 100);
  nh.param<double>("/scene_filter/ymin", ymin, -100);
  nh.param<double>("/scene_filter/ymax", ymax, 100);
  nh.param<double>("/scene_filter/zmin", zmin, -100);
  nh.param<double>("/scene_filter/zmax", zmax, 100);
  nh.param<double>("/scene_filter/leaf_s", leaf_, 0.005);
}

void sceneAcquirer::new_cloud_in_stream(const sensor_msgs::PointCloud2::ConstPtr& message)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  //constantly copy cloud from stream into class scene_stream_ to be accessible for service callback
  pcl::fromROSMsg (*message, *tmp);

  //check if we need to filter stream
  nh.getParam("/scene_filter/filter", filter_);
  if (filter_)
  {
    nh.getParam("/scene_filter/xmin", xmin);
    nh.getParam("/scene_filter/xmax", xmax);
    nh.getParam("/scene_filter/ymin", ymin);
    nh.getParam("/scene_filter/zmin", zmin);
    nh.getParam("/scene_filter/ymax", ymax);
    nh.getParam("/scene_filter/zmax", zmax);
    nh.getParam("/scene_filter/keep_organized", keep_organized_);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    if (keep_organized_)
    {
      pass.setKeepOrganized(true);
    }
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
    pass.filter (*scene_stream_);
    pcl::copyPointCloud(*scene_stream_ , *tmp);
  }
  //check if we need to downsample stream
  nh.getParam("/scene_filter/downsample", downsample_);
  if (downsample_)
  {
    nh.getParam("/scene_filter/leaf_s", leaf_);
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud (tmp);
    vg.setLeafSize(leaf_, leaf_, leaf_);
    vg.filter (*scene_stream_);
  }
  pub_stream_.publish( *scene_stream_ ); //republish the modified scene
}

bool sceneAcquirer::acquireScene (scene_filter_node::acquire_scene::Request& req, scene_filter_node::acquire_scene::Response& res)
{
  if (req.save.compare("false") != 0)
  {
    //user requested the scene to be saved on disk, lets comply him
    std::string home = std::getenv( "HOME" );
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed ((home + "/" + req.save + ".pcd").c_str(), *scene_stream_);
    ROS_INFO("[sceneFilter] Scene saved to %s", (home + "/" + req.save + ".pcd").c_str() );
  }
  //also send it to service response
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scene_stream_, msg);
  res.cloud = msg; 
  ROS_INFO("[sceneFilter] Sent scene to service response");
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scene_filter_node");
    sceneAcquirer node;
    ROS_INFO("[sceneAcquirer] Node is ready");
    ros::Rate rate(50); //go at 50hz
    while (node.nh.ok())
    {
      ros::spinOnce(); 
      rate.sleep();
    }
    return 0;
}
