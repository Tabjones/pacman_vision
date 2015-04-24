#ifndef _INCL_MODULES

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
#include "pacman_vision_comm/get_scene.h" 

//general utilities
#include <string>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>

using namespace pcl;

typedef boost::shared_ptr<ros::NodeHandle> NHPtr;

class VisionNode;
class Processor;
class Estimator;
class Tracker;

class Processor
{
  friend class VisionNode;
  friend class Estimator;
  friend class Tracker;
  
  public:
    Processor(NHPtr nhptr);
    PointCloud<PointXYZRGBA>::Ptr get_processed_scene() {return this->scene_processed;}
  private:
    //passed shared node handle ptr
    NHPtr nh_ptr;
    //Service Server
    ros::ServiceServer srv_get_scene;
    //Message Subscriber
    ros::Subscriber sub_openni;
    //Message Publisher
    ros::Publisher pub_scene;
    //pointer to processed point cloud
    PointCloud<PointXYZRGBA>::Ptr scene_processed;

    //Service callback for srv_ger_scene
    bool cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res);
    //Message callback, for sub_openni 
    void cb_openni(const sensor_msgs::PointCloud2::ConstPtr& message);

    //filter parameters
    bool filter, downsample, keep_organized;
    double xmin,xmax,ymin,ymax,zmin,zmax,leaf;
};

#define _INCL_MODULES
#endif
