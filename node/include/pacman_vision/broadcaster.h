#ifndef _INCL_BROADCASTER
#define _INCL_BROADCASTER
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
//Storage
#include "pacman_vision/storage.h"

class VisionNode;

class Broadcaster
{
  friend class VisionNode;

  public:
    Broadcaster(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~Broadcaster();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    boost::shared_ptr<Storage> storage;

    //bools to control what to broadcast
    bool tf, rviz_markers;

    //what to broadcast from estimator results and/or tracker
    /////////////////////////////////////////////////
    //eigen transforms
    boost::shared_ptr<std::vector<Eigen::Matrix4f> > estimated;
    //tf transforms
    std::vector<tf::Transform> transforms;
    //rviz markers
    visualization_msgs::MarkerArray markers;
    //naming and id-ing of estimator objects
    boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > names; //names/id

    //tf broadcaster
    tf::TransformBroadcaster tf_broadcaster;
    //rviz publisher
    ros::Publisher rviz_markers_pub;

    //custom spinner
    void spin_once();
    //method to broadcast
    void broadcast_once();
    void compute_transforms();
    void create_box_marker(visualization_msgs::Marker &box);

};
#endif
