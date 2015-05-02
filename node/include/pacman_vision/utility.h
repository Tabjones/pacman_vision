#ifndef _INCL_UTILITY

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>

void fromEigen(Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);

void fromPose(geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);
#define _INCL_UTILITY
#endif
