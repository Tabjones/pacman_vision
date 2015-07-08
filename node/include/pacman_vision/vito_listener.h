#ifndef _INCL_LISTENER
#define _INCL_LISTENER
//Utility
#include "pacman_vision/utility.h"
// ROS headers
#include <tf/transform_listener.h>
//Storage
#include "pacman_vision/storage.h"

class VisionNode;

class Listener
{
  friend class VisionNode;

  public:
    Listener(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~Listener();
    //Eigen Alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    boost::shared_ptr<Storage> storage;

    std::vector<std::string> arm_naming;
    //eigen transform
    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > left_arm, right_arm ;
    boost::shared_ptr<Eigen::Matrix4f> left_hand, right_hand, table;
    //tf transforms
    std::vector<tf::StampedTransform> left_arm_tf, right_arm_tf;
    tf::StampedTransform left_hand_tf, right_hand_tf, table_tf;

    //tf listener
    tf::TransformListener tf_listener;

    //behaviour
    bool listen_left_arm, listen_right_arm, listen_left_hand, listen_right_hand;
    //custom spinner
    void spin_once();

    //method to listen to vito transforms
    void listen_once();
    //method to listen to table transform
    void listen_table();


};
#endif
