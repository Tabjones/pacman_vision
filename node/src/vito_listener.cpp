#include "pacman_vision/vito_listener.h"

//////////////
// Listener //
//////////////
//TODO Find a way to reactivate listening of specific parts after a certain time after it was disabled
Listener::Listener(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor)
{
  this->nh = ros::NodeHandle (n, "listener");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->storage = stor;
  nh.param("/pacman_vision/crop_left_arm", listen_left_arm, false);
  //TODO = listen_right_arm = listen_left_hand = listen_right_hand = true;
  //initializing arm naming
  arm_naming.resize(6);
  arm_naming[0]= "_arm_2_link";
  arm_naming[1]= "_arm_3_link";
  arm_naming[2]= "_arm_4_link";
  arm_naming[3]= "_arm_5_link";
  arm_naming[4]= "_arm_6_link";
  arm_naming[5]= "_arm_7_link";
  left_arm.reset(new std::vector<Eigen::Matrix4f>);
  right_arm.reset(new std::vector<Eigen::Matrix4f>);
  left_hand.reset(new Eigen::Matrix4f);
  right_hand.reset(new Eigen::Matrix4f);
  table.reset(new Eigen::Matrix4f);
  left_arm->resize(arm_naming.size());
  right_arm->resize(arm_naming.size());
  left_arm_tf.resize(arm_naming.size());
  right_arm_tf.resize(arm_naming.size());
}

Listener::~Listener()
{
  this->nh.shutdown();
}

void Listener::listen_table()
{
  try
  {
    tf_listener.waitForTransform("/kinect2_rgb_optical_frame", "/workbench_plate_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform("/kinect2_rgb_optical_frame", "/workbench_plate_link", ros::Time(0), table_tf);
    geometry_msgs::Pose pose;
    fromTF(table_tf, *table, pose);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("[Listener][%s] Can not find Table Transformation...", __func__);
    return;
  }
  this->storage->write_table(this->table);
  return;
}

void Listener::listen_once()
{
  geometry_msgs::Pose pose;
  if (listen_left_arm)
  {
    std::string left = "left";
    try
    {
      for (int i=0; i<arm_naming.size(); ++i)
      {
        tf_listener.waitForTransform("/kinect2_rgb_optical_frame", (left+arm_naming[i]).c_str(), ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform("/kinect2_rgb_optical_frame", (left+arm_naming[i]).c_str(), ros::Time(0), left_arm_tf[i]);
        fromTF(left_arm_tf[i], left_arm->at(i), pose);
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      ROS_WARN("[Listener][%s] Can not find Vito Left Arm Transforms, disabling lookup...", __func__);
      listen_left_arm = false;
    }
    this->storage->write_left_arm(this->left_arm);
  }
  if (listen_right_arm)
  {
    std::string right = "right";
    try
    {
      for (int i=0; i<arm_naming.size(); ++i)
      {
        tf_listener.waitForTransform("/kinect2_rgb_optical_frame", (right+arm_naming[i]).c_str(), ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform("/kinect2_rgb_optical_frame", (right+arm_naming[i]).c_str(), ros::Time(0), right_arm_tf[i]);
        fromTF(right_arm_tf[i], right_arm->at(i), pose);
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      ROS_WARN("[Listener][%s] Can not find Vito Right Arm Transforms, disabling lookup...", __func__);
      listen_right_arm = false;
    }
    this->storage->write_right_arm(this->right_arm);
  }
  if (listen_left_hand)
  {
   try
   {
     tf_listener.waitForTransform("/kinect2_rgb_optical_frame", "/left_hand_palm_link", ros::Time(0), ros::Duration(2.0));
     tf_listener.lookupTransform("/kinect2_rgb_optical_frame", "/left_hand_palm_link", ros::Time(0), left_hand_tf);
     fromTF(left_hand_tf, *left_hand, pose);
   }
   catch (tf::TransformException& ex)
   {
     ROS_WARN("%s", ex.what());
     ROS_WARN("[Listener][%s] Can not find Vito Left Hand Transformation, disabling lookup...", __func__);
     listen_left_hand = false;
   }
   this->storage->write_left_hand(this->left_hand);
  }
  if (listen_right_hand)
  {
   try
   {
     tf_listener.waitForTransform("/kinect2_rgb_optical_frame", "/right_hand_palm_link", ros::Time(0), ros::Duration(2.0));
     tf_listener.lookupTransform("/kinect2_rgb_optical_frame", "/right_hand_palm_link", ros::Time(0), right_hand_tf);
     fromTF(right_hand_tf, *right_hand, pose);
   }
   catch (tf::TransformException& ex)
   {
     ROS_WARN("%s", ex.what());
     ROS_WARN("[Listener][%s] Can not find Vito Right Hand Transformation, disabling lookup...", __func__);
     listen_right_hand = false;
   }
   this->storage->write_right_hand(this->right_hand);
  }
  return;
}
void Listener::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}
