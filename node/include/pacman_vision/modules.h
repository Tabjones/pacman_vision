#ifndef _INCL_MODULES

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//PCL
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// ROS generated headers
#include "pacman_vision_comm/get_scene.h" 

//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

#define D2R 0.017453293  //deg to rad conversion

using namespace pcl;

typedef boost::shared_ptr<ros::NodeHandle> NHPtr;

class VisionNode;
class Processor;
class Estimator;
class Tracker;

class Processor
{
  friend class VisionNode;
  
  public:
    Processor(NHPtr nhptr);
    PointCloud<PointXYZRGBA>::Ptr get_processed_scene() {return this->scene_processed;}
  private:
    //passed shared node handle ptr
    NHPtr nh_ptr;
    //Service Server
    ros::ServiceServer srv_get_scene;
    //Message Subscriber
    ros::Subscriber sub_openni;
    //Message Publisher
    ros::Publisher pub_scene;
    //pointer to processed point cloud
    PointCloud<PointXYZRGBA>::Ptr scene_processed;

    //Service callback for srv_ger_scene
    bool cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res);
    //Message callback, for sub_openni 
    void cb_openni(const sensor_msgs::PointCloud2::ConstPtr& message);

    //filter parameters
    bool filter, downsample, keep_organized;
    double xmin,xmax,ymin,ymax,zmin,zmax,leaf;
};

class Estimator
{
  friend class VisionNode;

  public:
    Estimator(NHPtr nhptr);
  private:
    //passed shared node handle ptr
    NHPtr nh_ptr;
    //Service Server
    ros::ServiceServer srv_estimate;
    //estimated transforms
    std::vector<Eigen::Matrix4f> estimations;
    //object clusters found on scene
    std::vector<PointCloud<PointXYZRGBA> > clusters;
    //naming and id-ing of estimated objects
    std::vector<std::string> names;
    std::vector<std::string> ids;
    
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table;
    //path to pel database
    boost::filesystem::path db_path;

    /*
    bool visualize_;
    bool stop_thread_;
*/
    //method to extract clusters of objects in a table top scenario
    int extract_clusters(PointCloud<PointXYZRGBA>::Ptr scene);
    //method to perform pose estimation on each found cluster
    void pose_estimate()
    //service callback  
    bool cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res);
    //void broadcast_thread_body();
};

#define _INCL_MODULES
#endif
