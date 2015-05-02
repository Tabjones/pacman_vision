#ifndef _INCL_BROADCASTER
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <Eigen/Dense>

class VisionNode;

class Broadcaster
{
  friend class VisionNode;

  public:
    Broadcaster(ros::NodeHandle &n);
    ~Broadcaster();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    
    //bools to control what to broadcast
    bool tf, rviz_markers;

    //what to broadcast from estimator and/or tracker
    /////////////////////////////////////////////////
    //eigen transforms 
    std::vector<Eigen::Matrix4f> estimated;
    //tf transforms
    std::vector<tf::Transform> transforms;
    //rviz markers
    visualization_msgs::MarkerArray markers;
    //naming and id-ing of estimator objects
    std::vector<std::string> names;
    std::vector<std::string> ids;
    
    //tf broadcaster
    tf::TransformBroadcaster tf_broadcaster;
    //rviz publisher
    ros::Publisher rviz_markers_pub;

    //custom spinner
    void spin_once();
    //method to broadcast
    void broadcast_once();
    void compute_transforms();
    
};
#define _INCL_BROADCASTER
#endif
