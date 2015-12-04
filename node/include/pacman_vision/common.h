#ifndef _INCL_COMMON_
#define _INCL_COMMON_
// ROS headers
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
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

using namespace pcl;

//convert from eigen matrix to tf and pose
void
fromEigen(Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);
//converto from pose to eigen matrix and tf
void
fromPose(geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);
//convert from tf to eigen matrix and pose
void
fromTF(tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);
void
fromTF(tf::StampedTransform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);

// Convenient Typedefs for point clouds
typedef pcl::PointXYZRGB PT; //default point type
typedef pcl::PointCloud<PT> PC; //default point cloud with default point type

typedef pcl::PointXYZ PX; //point type without color
typedef pcl::PointCloud<PX> PXC; //point cloud with PX type

typedef pcl::PointNormal PN; //point normal type
typedef pcl::PointCloud<PN> PNC; //point normal cloud

typedef pcl::Normal NT; //point type with only normals
typedef pcl::PointCloud<NT> NC; //Normal cloud

typedef std::lock_guard<std::mutex> LOCK; //default lock type

//Data structure for box, defined by bounduaries
class Box
{
    public:
        double x1,y1,z1;
        double x2,y2,z2;
        //ctors
        Box(){}
        Box(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
            : x1(xmin), y1(ymin), z1(zmin), x2(xmax), y2(ymax), z2(zmax) {}
        Box(const Box& other) : x1(other.x1), x2(other.x2), y1(other.y1), y2(other.y2), z1(other.z1), z2(other.z2) {}
        Box(Box&& other) : x1(std::move(other.x1)), x2(std::move(other.x2)), y1(std::move(other.y1)),y2(std::move(other.y2)), z1(std::move(other.z1)), z2(std::move(other.z2)) {}
        //dtor
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
        const Box operator* (const float scale) const
        {
            return (Box(this->x1*scale, this->y1*scale, this->z1*scale, this->x2*scale, this->y2*scale, this->z2*scale));
        }
};

//Crop a source point cloud into dest, previously transforming it with trans
void crop_a_box(PC::Ptr source, PC::Ptr& dest,const Eigen::Matrix4f& trans, const Box lim, bool crop_inside=false, bool organized=true);

#endif

