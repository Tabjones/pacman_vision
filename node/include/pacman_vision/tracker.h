#ifndef _INCL_TRACKER

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
//PCL
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
// ROS generated headers
#include "pacman_vision_comm/track_object.h"
//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

typedef pcl::PointXYZRGBA PT;
typedef pcl::PointCloud<PT> PC;

class VisionNode;

class Tracker
{
  friend class VisionNode;

  public:
    Tracker(ros::NodeHandle &n);
    ~Tracker();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    //Service Server
    ros::ServiceServer srv_track_object;
    //tracker transform
    Eigen::Matrix4f transform;
    //name and id of object to be tracked
    std::string name;
    std::string id;
    //pool of estimated objects from estimator
    std::vector<std::string> names;
    std::vector<Eigen::Matrix4f> estimations;
    //actual scene
    PC::Ptr scene;
    //actual model
    PC::Ptr model;
    //TODO add params
    
    //track_object service callback  
    bool cb_track_object(pacman_vision_comm::track_object::Request& req, pacman_vision_comm::track_object::Response& res);
    
    //custom spin method
    void spin_once();
};
#define _INCL_TRACKER
#endif
