#ifndef _INCL_MODULE
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

void fromEigen(Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);

void fromPose(geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);

#define _INCL_MODULE
#endif
