#ifndef _INCL_SCENE_PROCESS

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// ROS generated headers
#include "vision_communications/get_scene.h" 

//general utilities
#include <string>
#include <stdlib.h>

namespace scene_processing
{
  class SceneProcess
  {
  public:
    SceneProcess(ros::NodeHandle &nh);
  private:
    //passed node handle
    ros::NodeHandle nh;
    //Service Server
    ros::ServiceServer srv_get_scene;
    //Message Subscriber
    ros::Subscriber sub_openni;
    //Message Publisher
    ros::Publisher pub_scene;
    //pointer to processed point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_processed;
    
    //Service callback for srv_ger_scene
    bool cb_get_scene(vision_communications::get_scene::Request& req, vision_communications::get_scene::Response& res);
    //Message callback, for sub_openni 
    void cb_openni(const sensor_msgs::PointCloud2::ConstPtr& message);

    //filter parameters
    bool filter, downsample, keep_organized;
    double xmin,xmax,ymin,ymax,zmin,zmax,leaf;
  };
}

#define _INCL_SCENE_PROCESS
#endif
