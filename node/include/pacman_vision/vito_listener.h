#ifndef _INCL_LISTENER
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
//generated
#include "pacman_vision_comm/pe.h"
#include "pacman_vision_comm/peArray.h"
#include "pacman_vision_comm/grasp_verification.h"
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

class Listener
{
  friend class VisionNode;

  public:
    Listener(ros::NodeHandle &n);
    ~Listener();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    //service server
    ros::ServiceServer srv_grasp;
    
    //eigen transform 
    Eigen::Matrix4f left, right, table;
    //tf transforms
    tf::StampedTransform left_tf, right_tf, table_tf;
    
    //tf listener
    tf::TransformListener tf_listener;

    //custom spinner
    void spin_once();

    //method to listen to vito transforms
    void listen_once();
    //method to listen to table transform
    void listen_table();

    //grasp_verification service callback
    bool cb_grasp(pacman_vision_comm::grasp_verification::Request& req, pacman_vision_comm::grasp_verification::Response& res);
    
};
#define _INCL_LISTENER
#endif
