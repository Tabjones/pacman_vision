#ifndef _INCL_MODULE
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
//convert from eigen matrix to tf and pose
void fromEigen(Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);
//converto from pose to eigen matrix and tf
void fromPose(geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);
//convert from tf to eigen matrix and pose
void fromTF(tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);
void fromTF(tf::StampedTransform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);
#define _INCL_MODULE
#endif
