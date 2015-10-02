#include <pacman_vision/vito_listener.h>
#include <boost/filesystem.hpp>

//////////////
// Listener //
//////////////
Listener::Listener(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor)
{
  this->nh = ros::NodeHandle (n, "listener");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->storage = stor;
  this->srv_get_cloud = nh.advertiseService("get_cloud_in_hand", &Listener::cb_get_cloud_in_hand, this);
  nh.param<bool>("/pacman_vision/crop_left_arm", listen_left_arm, false);
  nh.param<bool>("/pacman_vision/crop_right_arm", listen_right_arm, false);
  nh.param<bool>("/pacman_vision/crop_left_hand", listen_left_hand, false);
  nh.param<bool>("/pacman_vision/crop_right_hand", listen_right_hand, false);
  //initializing arm naming
  arm_naming.resize(7);
  arm_naming[0]= "_arm_1_link";
  arm_naming[1]= "_arm_2_link";
  arm_naming[2]= "_arm_3_link";
  arm_naming[3]= "_arm_4_link";
  arm_naming[4]= "_arm_5_link";
  arm_naming[5]= "_arm_6_link";
  arm_naming[6]= "_arm_7_link";
  detailed_hand_naming.resize(21);
  detailed_hand_naming[0]= "_softhand_base";
  detailed_hand_naming[1]= "_palm_link";
  detailed_hand_naming[2]= "_index_knuckle_link";
  detailed_hand_naming[3]= "_index_proximal_link";
  detailed_hand_naming[4]= "_index_middle_link";
  detailed_hand_naming[5]= "_index_distal_link";
  detailed_hand_naming[6]= "_little_knuckle_link";
  detailed_hand_naming[7]= "_little_proximal_link";
  detailed_hand_naming[8]= "_little_middle_link";
  detailed_hand_naming[9]= "_little_distal_link";
  detailed_hand_naming[10]= "_middle_knuckle_link";
  detailed_hand_naming[11]= "_middle_proximal_link";
  detailed_hand_naming[12]= "_middle_middle_link";
  detailed_hand_naming[13]= "_middle_distal_link";
  detailed_hand_naming[14]= "_ring_knuckle_link";
  detailed_hand_naming[15]= "_ring_proximal_link";
  detailed_hand_naming[16]= "_ring_middle_link";
  detailed_hand_naming[17]= "_ring_distal_link";
  detailed_hand_naming[18]= "_thumb_knuckle_link";
  detailed_hand_naming[19]= "_thumb_proximal_link";
  detailed_hand_naming[20]= "_thumb_distal_link";
  left_arm.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
  right_arm.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
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
  std::string sens_ref_frame;
  this->storage->read_sensor_ref_frame(sens_ref_frame);
  try
  {
    tf_listener.waitForTransform(sens_ref_frame.c_str(), "/workbench_plate_link", ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform(sens_ref_frame.c_str(), "/workbench_plate_link", ros::Time(0), table_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("[Listener][%s] Can not find Table Transformation. Using identity...", __func__);
    table_tf.setIdentity();
  }
  geometry_msgs::Pose pose;
  fromTF(table_tf, *table, pose);
  this->storage->write_table(this->table);
  return;
}

void Listener::listen_once()
{
  geometry_msgs::Pose pose;
  std::string sens_ref_frame;
  this->storage->read_sensor_ref_frame(sens_ref_frame);
  if (listen_left_arm)
  {
    std::string left = "left";
    try
    {
      for (int i=0; i<arm_naming.size(); ++i)
      {
        tf_listener.waitForTransform(sens_ref_frame.c_str(), (left+arm_naming[i]).c_str(), ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform(sens_ref_frame.c_str(), (left+arm_naming[i]).c_str(), ros::Time(0), left_arm_tf[i]);
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
        tf_listener.waitForTransform(sens_ref_frame.c_str(), (right+arm_naming[i]).c_str(), ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform(sens_ref_frame.c_str(), (right+arm_naming[i]).c_str(), ros::Time(0), right_arm_tf[i]);
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
     tf_listener.waitForTransform(sens_ref_frame.c_str(), "/left_hand_palm_link", ros::Time(0), ros::Duration(2.0));
     tf_listener.lookupTransform(sens_ref_frame.c_str(), "/left_hand_palm_link", ros::Time(0), left_hand_tf);
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
     tf_listener.waitForTransform(sens_ref_frame.c_str(), "/right_hand_palm_link", ros::Time(0), ros::Duration(2.0));
     tf_listener.lookupTransform(sens_ref_frame.c_str(), "/right_hand_palm_link", ros::Time(0), right_hand_tf);
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

bool Listener::cb_get_cloud_in_hand(pacman_vision_comm::get_cloud_in_hand::Request& req, pacman_vision_comm::get_cloud_in_hand::Response& res)
{
  PC::Ptr cloud (new PC); //gets progressively overwritten
  PC::Ptr cloud_original (new PC);
  PC::Ptr obj (new PC);
  PC::Ptr piece (new PC);
  this->storage->read_scene_processed(cloud);
  pcl::copyPointCloud(*cloud, *cloud_original);
  //listen right or left hand based on req.right
  for (size_t i=0; i<detailed_hand_naming.size(); ++i)
  {
    if (i == 0)
      listen_and_extract_detailed_hand_piece(req.right, i, cloud_original, obj);
    listen_and_extract_detailed_hand_piece(req.right, i, cloud_original, piece);
    listen_and_crop_detailed_hand_piece(req.right, i, cloud);
    *obj += *piece;
  }
  sensor_msgs::PointCloud2 msg, msg2;
  if (req.save.compare("false") != 0)
  {
    boost::filesystem::path save_dir (req.save);
    if (boost::filesystem::exists(save_dir) && boost::filesystem::is_directory(save_dir))
    {
      pcl::io::savePCDFile( (save_dir.string() + "/obj.pcd").c_str(), *cloud );
      pcl::io::savePCDFile( (save_dir.string() + "/hand.pcd").c_str(), *obj );
    }
    else
    {
      ROS_ERROR ("[Listener][%s] Invalid save directory passed to service: %s",__func__,save_dir.c_str());
      return false;
    }
  }
  pcl::toROSMsg(*cloud, msg);
  pcl::toROSMsg(*obj, msg2);
  res.obj = msg;
  res.hand = msg2;
  return true;
}

void Listener::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}

void Listener::listen_and_crop_detailed_hand_piece(bool right, size_t idx, PC::Ptr& cloud)
{
  std::string sens_ref_frame, hand, piece;
  this->storage->read_sensor_ref_frame(sens_ref_frame);
  tf::StampedTransform tf_piece;
  piece = detailed_hand_naming[idx];
  Eigen::Matrix4f trans;
  geometry_msgs::Pose pose;
  if (right)
    hand = "right_hand";
  else
    hand = "left_hand";
  try
  {
    tf_listener.waitForTransform(sens_ref_frame.c_str(), (hand+piece).c_str(), ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform(sens_ref_frame.c_str(), (hand+piece).c_str(), ros::Time(0), tf_piece);
    fromTF(tf_piece, trans, pose);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("[Listener][%s] Can not find %s Transformation, setting identity",__func__,(hand+piece).c_str() );
    trans.setIdentity();
  }
  //crop it
  PC::Ptr out (new PC);
  if (idx==0)
  {
    Box outer_box(-0.25, -0.25, 0, 0.25, 0.25, 0.5);
    crop_a_box(cloud, out, trans, outer_box, false, true);
    pcl::copyPointCloud(*out, *cloud);
  }
  if (right)
    crop_a_box(cloud, out, trans, soft_hand_right[idx], true, true);
  else
    crop_a_box(cloud, out, trans, soft_hand_left[idx], true, true);
  pcl::copyPointCloud(*out, *cloud);
}

void Listener::listen_and_extract_detailed_hand_piece(bool right, size_t idx, PC::Ptr& cloud, PC::Ptr& piece)
{
  std::string sens_ref_frame, hand, hand_piece;
  this->storage->read_sensor_ref_frame(sens_ref_frame);
  tf::StampedTransform tf_piece;
  hand_piece = detailed_hand_naming[idx];
  Eigen::Matrix4f trans, inv;
  geometry_msgs::Pose pose;
  if (right)
    hand = "right_hand";
  else
    hand = "left_hand";
  try
  {
    tf_listener.waitForTransform(sens_ref_frame.c_str(), (hand+hand_piece).c_str(), ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform(sens_ref_frame.c_str(), (hand+hand_piece).c_str(), ros::Time(0), tf_piece);
    fromTF(tf_piece, trans, pose);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("[Listener][%s] Can not find %s Transformation, setting identity",__func__,(hand+hand_piece).c_str() );
    trans.setIdentity();
  }
  //crop it
  if (right)
    crop_a_box(cloud, piece, trans, soft_hand_right[idx], false, true);
  else
    crop_a_box(cloud, piece, trans, soft_hand_left[idx], false, true);
}
