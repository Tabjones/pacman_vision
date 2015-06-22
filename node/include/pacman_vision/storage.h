#ifndef _INCL_STORAGE
#define _INCL_STORAGE

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//PCL
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//general utilities
#include <cmath>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>
#include <string>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

class VisionNode;
class Estimator;
class Broadcaster;
class Tracker;
class Supervoxels;

class Storage
{
  friend class VisionNode;
  friend class Estimator;
  friend class Broadcaster;
  friend class Tracker;
  friend class Supervoxels;

  public:
    Storage();
  private:
    //mutexes
    boost::mutex clouds;
    boost::mutex objects;
    boost::mutex tracked;

    //////////////// Protected by mutex clouds /////////////
    //untouched scene from openni
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene;
    //scene after passthrough
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_passthrough;
    //scene after downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsample;
    //scene after plane segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_plane;
    //scene processed after all filters (excluding clustering)
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_processed;
    float leaf_size; //leaf size used (tracker is interested in this to downsample model with same density as scene)

    ///////////// Protected by mutex objects
    //cluster of objects found on scene
    std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters;
    //Estimated transform from estimator
    std::vector<Eigen::Matrix4f> estimations;
    //naming and id-ing of estimated objects from estimator
    std::vector<std::pair<std::string, std::string> > names; //name,ID
    //path to PEL database on disk
    boost::filesystem::path pel_db_path;

    //////////// Protected by mutex tracked
    //tracker actual transform
    Eigen::Matrix4f tracked_transform;
    std::string tracked_id;

};
#endif
