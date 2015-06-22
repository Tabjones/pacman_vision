#ifndef _INCL_LISTENER
#define _INCL_LISTENER
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
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
//Storage
#include "pacman_vision/storage.h"

class VisionNode;

class Listener
{
  friend class VisionNode;

  public:
    Listener(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~Listener();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    boost::shared_ptr<Storage> storage;

    //eigen transform
    Eigen::Matrix4f left_2, left_3, left_4, left_5, left_6, left_7,
                    right_2, right_3, right_4, right_5, right_6, right_7,
                    table, left_hand, right_hand;
    //tf transforms
    tf::StampedTransform left_tf_2, left_tf_3, left_tf_4, left_tf_5, left_tf_6, left_tf_7,
                         right_tf_2, right_tf_3, right_tf_4, right_tf_5, right_tf_6, right_tf_7,
                         table_tf, left_tf_hand, right_tf_hand;

    //tf listener
    tf::TransformListener tf_listener;

    //custom spinner
    void spin_once();

    //method to listen to vito transforms
    void listen_once();
    //method to listen to table transform
    void listen_table();


};
#endif
