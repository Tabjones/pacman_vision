#ifndef _INCL_UTILITY
#define _INCL_UTILITY
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
//convert from eigen matrix to tf and pose
void fromEigen(Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);
//converto from pose to eigen matrix and tf
void fromPose(geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);
//convert from tf to eigen matrix and pose
void fromTF(tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);
void fromTF(tf::StampedTransform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);

// Convenient Typedefs
typedef pcl::PointXYZRGB PT; //default point type
typedef pcl::PointCloud<PT> PC; //default point cloud with default point type

typedef pcl::PointXYZ PX; //point type without color
typedef pcl::PointCloud<PX> PXC; //point cloud with PX type

typedef pcl::PointNormal PN; //point normal type
typedef pcl::PointCloud<PN> PNC; //point normal cloud

#define D2R 0.017453293  //deg to rad conversion

typedef boost::strict_lock<boost::mutex> LOCK;

#endif
