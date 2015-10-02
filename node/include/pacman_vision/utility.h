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
#include <tf/transform_broadcaster.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
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
#include <utility>

#define D2R 0.017453293  //deg to rad conversion

using namespace pcl;

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
class Box
{
  public:
    double x1,x2;
    double y1,y2;
    double z1,z2;
    Box(){}
    Box(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax) : x1(xmin), y1(ymin),
      z1(zmin), x2(xmax), y2(ymax), z2(zmax) {}
    Box(const Box& other) : x1(other.x1), x2(other.x2), y1(other.y1), y2(other.y2), z1(other.z1), z2(other.z2) {}
    Box(Box&& other) : x1(std::move(other.x1)), x2(std::move(other.x2)), y1(std::move(other.y1)),
      y2(std::move(other.y2)), z1(std::move(other.z1)), z2(std::move(other.z2)) {}
    ~Box(){}
    Box& operator= (const Box& other)
    {
      x1=other.x1;
      x2=other.x2;
      y1=other.y1;
      y2=other.y2;
      z1=other.z1;
      z2=other.z2;
      return *this;
    }
    Box& operator= (Box&& other)
    {
      x1= std::move(other.x1);
      x2= std::move(other.x2);
      y1= std::move(other.y1);
      y2= std::move(other.y2);
      z1= std::move(other.z1);
      z2= std::move(other.z2);
      return *this;
    }
};

//Crop a source point cloud into dest, previously transforming it with trans
void crop_a_box(PC::Ptr source, PC::Ptr& dest,const Eigen::Matrix4f& trans, const Box lim, bool crop_inside=false, bool organized=true);

#endif

