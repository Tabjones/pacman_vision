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

#include "pacman_vision/utility.h"

class Storage
{
  public:
    Storage();

    //Read and write methods
    void read_scene (PC::Ptr &cloud);
    void write_scene (PC::Ptr &cloud);
    void read_scene_processed (PC::Ptr &cloud);
    void write_scene_processed (PC::Ptr &cloud);
  private:
    //mutexes
    boost::mutex scenes;
    boost::mutex objects;
    boost::mutex tracked;
    //////////////// Protected by mutex scenes /////////////
    //untouched scene from kinect
    PC::Ptr scene;
    //scene after processing
    PC::Ptr scene_processed;
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
