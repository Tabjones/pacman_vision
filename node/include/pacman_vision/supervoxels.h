#ifndef _INCL_SUPERVOXELS
// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/features/normal_3d_omp.h>
//Ros generated
#include "pacman_vision_comm/clusterize.h"
//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <Eigen/Dense>

class VisionNode;

class Supervoxels
{
  typedef pcl::PointXYZRGB PT; //default point type
  typedef pcl::PointCloud<PT> PC; //default point cloud
  typedef pcl::PointNormal PN; //default point normal type
  typedef pcl::PointCloud<PN> PCN; //default point normal cloud

  friend class VisionNode;

  public:
    Supervoxels(ros::NodeHandle &n);
    ~Supervoxels();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;

    PC::Ptr scene;
    PC::Ptr clustered_scene;
    pcl::SupervoxelClustering<PT> svc;
    pcl::NormalEstimationOMP<PT, PN> ne;

    //class behaviour
    bool serviced;
    float voxel_res, seed_res, color_imp, normal_imp, spatial_imp;
    int num_iterations;

    std::map<uint32_t, pcl::Supervoxel<PT>::Ptr > clusters;

    //Service Server
    ros::ServiceServer srv_clusterize;

    //Publisher for clusterized scene
    ros::Publisher pub_clusterized_scene;

    //custom spinner
    void spin_once();

    //method to perform clustering
    void clustering();

    //clusterize service callback
    bool cb_clusterize(pacman_vision_comm::clusterize::Request& req, pacman_vision_comm::clusterize::Response& res);

    
};
#define _INCL_SUPERVOXELS
#endif
