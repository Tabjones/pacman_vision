#ifndef _INCL_PROCESSOR

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <dynamic_reconfigure/server.h>
//PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
// ROS generated headers
#include "pacman_vision_comm/get_scene.h"
//#include "pacman_vision/processorConfig.h"

//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>


using namespace pcl;


class VisionNode;

class Processor
{
  friend class VisionNode;
  
  public:
    Processor(ros::NodeHandle &n);
    ~Processor();
  private:
    ros::NodeHandle nh;
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

    //dynamic reconfigure
   // dynamic_reconfigure::Server<pacman_vision::processorConfig> dyn_srv;
   // void cb_reconfigure(pacman_vision::processorConfig &config, uint32_t level);
};
#define _INCL_PROCESSOR
#endif
