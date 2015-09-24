#ifndef _INCL_BROADCASTER
#define _INCL_BROADCASTER

#include <pacman_vision/config.h>
//Utility
#include <pacman_vision/utility.h>
// ROS headers
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
//Storage
#include <pacman_vision/storage.h>

class VisionNode;

class Broadcaster
{
  friend class VisionNode;

  public:
    Broadcaster(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~Broadcaster();
    //Eigen Alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    boost::shared_ptr<Storage> storage;

    //bools to control what to broadcast
    bool obj_tf, obj_markers, pass_limits, tracker_bb;

    //what to broadcast from estimator results and/or tracker
    /////////////////////////////////////////////////////////
    //eigen transforms
    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > estimated;
    //tf transforms
    std::vector<tf::Transform> transforms;
    //naming and id-ing of estimator objects
    boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > names; //names/id

    // ALL rviz markers to publish
    visualization_msgs::MarkerArray markers;
    //tf broadcaster
    tf::TransformBroadcaster tf_broadcaster;
    //rviz publisher
    ros::Publisher rviz_markers_pub;

    //custom spinner
    void spin_once();
    //method to broadcast
    void broadcast_once();
    void elaborate_estimated_objects();
    bool create_box_marker(visualization_msgs::Marker &box, boost::shared_ptr<Box> &limits);

};
#endif
