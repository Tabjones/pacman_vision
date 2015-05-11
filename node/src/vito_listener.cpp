#include "pacman_vision/vito_listener.h"
#include "pacman_vision/utility.h"

//////////////
// Listener //
//////////////
Listener::Listener(ros::NodeHandle &n)
{
  this->nh = ros::NodeHandle (n, "listener");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
}
Listener::~Listener()
{
  this->nh.shutdown();
}

void Listener::listen_table()
{
  try
  {
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/workbench_plate_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/workbench_plate_link", ros::Time(0), table_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  geometry_msgs::Pose pose;
  fromTF(table_tf, table, pose);
  return;
}

void Listener::listen_once()
{
  try
  {
    //arms
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_arm_2_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_arm_2_link", ros::Time(0), left_tf_2);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_arm_3_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_arm_3_link", ros::Time(0), left_tf_3);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_arm_4_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_arm_4_link", ros::Time(0), left_tf_4);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_arm_5_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_arm_5_link", ros::Time(0), left_tf_5);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_arm_6_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_arm_6_link", ros::Time(0), left_tf_6);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_arm_7_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_arm_7_link", ros::Time(0), left_tf_7);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_arm_2_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_arm_2_link", ros::Time(0), right_tf_2);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_arm_3_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_arm_3_link", ros::Time(0), right_tf_3);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_arm_4_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_arm_4_link", ros::Time(0), right_tf_4);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_arm_5_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_arm_5_link", ros::Time(0), right_tf_5);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_arm_6_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_arm_6_link", ros::Time(0), right_tf_6);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_arm_7_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_arm_7_link", ros::Time(0), right_tf_7);
    //hands
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_hand_palm_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_hand_palm_link", ros::Time(0), right_tf_hand);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_hand_palm_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_hand_palm_link", ros::Time(0), left_tf_hand);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  geometry_msgs::Pose pose;
  fromTF(left_tf_2, left_2, pose);
  fromTF(left_tf_3, left_3, pose);
  fromTF(left_tf_4, left_4, pose);
  fromTF(left_tf_5, left_5, pose);
  fromTF(left_tf_6, left_6, pose);
  fromTF(left_tf_7, left_7, pose);
  fromTF(right_tf_2, right_2, pose);
  fromTF(right_tf_3, right_3, pose);
  fromTF(right_tf_4, right_4, pose);
  fromTF(right_tf_5, right_5, pose);
  fromTF(right_tf_6, right_6, pose);
  fromTF(right_tf_7, right_7, pose);
  fromTF(right_tf_hand, right_hand, pose);
  fromTF(left_tf_hand, left_hand, pose);
  return;
}
void Listener::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}
