#ifndef _COMMON_H_
#define _COMMON_H_
// ROS headers
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// General Utils
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <utility>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#define D2R M_PI/180  //deg to rad conversion
#define R2D 180/M_PI  //rad to deg conversion

//convert from eigen 4x4 matrix to tf and pose
void fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);
void fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest);
void fromEigen(const Eigen::Matrix4f &source, tf::Transform &dest);
//converto from pose to eigen 4x4 matrix and tf
void fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);
void fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest);
void fromPose(const geometry_msgs::Pose &source, tf::Transform &dest);
//convert  from  tf  to  eigen  4x4   matrix  and  pose,  will  also  work  with
//TransformStamped, since it is derived from Transform
void fromTF(const tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);
void fromTF(const tf::Transform &source, Eigen::Matrix4f &dest);
void fromTF(const tf::Transform &source, geometry_msgs::Pose &dest);

// Convenient Typedefs
typedef pcl::PointXYZRGB PT; //default point type
typedef pcl::PointCloud<pcl::PointXYZRGB> PTC; //default point cloud with default point type

typedef pcl::PointXYZ PX; //point type without color
typedef pcl::PointCloud<pcl::PointXYZ> PXC; //point cloud with PX type

typedef pcl::PointNormal PN; //point normal type
typedef pcl::PointCloud<pcl::PointNormal> PNC; //point normal cloud

typedef pcl::Normal NT; //point type with only normals
typedef pcl::PointCloud<pcl::Normal> NTC; //Normal cloud

typedef std::lock_guard<std::mutex> LOCK; //default lock type

//Data structure for box, defined by bounduaries and some basic arithmetics
struct Box
{
    typedef std::shared_ptr<Box> Ptr;
    double x1,y1,z1;
    double x2,y2,z2;
    //ctors
    Box(){}
    Box(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
        : x1(xmin), y1(ymin), z1(zmin), x2(xmax), y2(ymax), z2(zmax) {}
    Box(const Box& other) : x1(other.x1),  y1(other.y1), z1(other.z1),
        x2(other.x2), y2(other.y2), z2(other.z2) {}
    Box(Box&& other) : x1(std::move(other.x1)), y1(std::move(other.y1)), z1(std::move(other.z1)),
        x2(std::move(other.x2)),  y2(std::move(other.y2)), z2(std::move(other.z2)) {}
    //dtor
    ~Box(){}
    Box& operator= (const Box& other);
    Box& operator= (Box&& other);
    Box operator* (const float scale) const;
};

//Crop a source point cloud into dest,  where cropbox is defined by the Box lim.
//Optionally  remove whats  inside  the  box, rather  than  keep it.  Optionally
//transform the box with trans before  applying the crop. Finally keep the point
//cloud structure organized, if requested.
//
//WARN: dest pointer gets modified or initialized if empty
void crop_a_box(const PTC::ConstPtr source, PTC::Ptr &dest, const Box lim,
        const bool remove_inside=false,
        const Eigen::Matrix4f& trans=Eigen::Matrix4f::Identity(),
        const bool keep_organized=false);

//Create  a box  marker out  of a  Box object,  if cube  type is  true create  a
//semitransparent cube instead of lines
void create_box_marker(const Box lim, visualization_msgs::Marker &marker, bool cube_type);
#endif

