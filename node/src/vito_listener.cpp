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
  this->srv_grasp = nh.advertiseService("grasp_verification", &Listener::cb_grasp, this);
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
  catch (tf::TransformException)
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
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/left_hand_palm_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/left_hand_palm_link", ros::Time(0), left_tf);
    tf_listener.waitForTransform("/camera_rgb_optical_frame", "/right_hand_palm_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/camera_rgb_optical_frame", "/right_hand_palm_link", ros::Time(0), right_tf);
  }
  catch (tf::TransformException)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  geometry_msgs::Pose pose;
  fromTF(left_tf, left, pose);
  fromTF(right_tf, right, pose);
  return;
}

void Listener::cb_grasp(pacman_vision_comm::grasp_verification::Request& req, pacman_vision_comm::grasp_verification::Response& res)
{
  //TODO
}

void Listener::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}
