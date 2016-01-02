#ifndef _COMMON_ROS_H_
#define _COMMON_ROS_H_
// ROS headers
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <common/box.h>
/*! \file common_ros.h
    \brief Bunch of global functions and typedefs from ROS, used within Pacman Vision
*/

namespace pacv
{
/// Convert from eigen 4x4 matrix to tf and pose
void fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);
/// Convert from eigen 4x4 matrix to pose
void fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest);
/// convert from eigen 4x4 matrix to tf
void fromEigen(const Eigen::Matrix4f &source, tf::Transform &dest);
/// Convert from pose to eigen 4x4 matrix and tf
void fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);
/// Convert from pose to eigen 4x4 matrix
void fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest);
/// Convert from pose to tf
void fromPose(const geometry_msgs::Pose &source, tf::Transform &dest);
/// Convert  from  tf  to  eigen  4x4   matrix  and  pose
void fromTF(const tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);
/// Convert  from  tf  to  eigen  4x4   matrix
void fromTF(const tf::Transform &source, Eigen::Matrix4f &dest);
/// Convert  from  tf  to pose
void fromTF(const tf::Transform &source, geometry_msgs::Pose &dest);

/*! \brief Create  a box  marker out  of a  Box object.
 *
 * \param[in] lim Box object containing the limits the marker will have.
 * \param[out] marker Corresponding marker created from lim.
 * \param[in] cube_type if true create a semitransparent cube instead of a collection of lines.
 */
void create_box_marker(const Box lim, visualization_msgs::Marker &marker, const bool cube_type=false);
}
#endif

