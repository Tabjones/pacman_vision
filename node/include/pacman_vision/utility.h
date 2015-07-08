#ifndef _INCL_UTILITY
#define _INCL_UTILITY
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
// General Utils
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/timer.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/range/algorithm.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#define D2R 0.017453293  //deg to rad conversion

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

typedef pcl::Normal NT; //point type with only normals
typedef pcl::PointCloud<NT> NC; //Normal cloud

typedef boost::lock_guard<boost::mutex> LOCK; //default lock type

//Data structure for box, defined by bounduaries
struct Box{
  double x1,x2;
  double y1,y2;
  double z1,z2;
};

#endif
