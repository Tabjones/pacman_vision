#ifndef _INCL_ESTIMATOR

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
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
// ROS generated headers
#include "pacman_vision_comm/estimate.h"
#include "pacman_vision_comm/pe.h"
#include "pacman_vision_comm/peArray.h"
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
//PEL
#include <pel.h> //also gets typedefs of PEL

#define D2R 0.017453293  //deg to rad conversion

class VisionNode;

class Estimator
{
  friend class VisionNode;

  public:
    Estimator(ros::NodeHandle &n);
    ~Estimator();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    //Service Server
    ros::ServiceServer srv_estimate;
    //estimated transforms
    std::vector<Eigen::Matrix4f> estimations;
    //object clusters found on scene
    std::vector<PC> clusters;
    //naming and id-ing of estimated objects
    std::vector<std::string> names;
    std::vector<std::string> ids;
    //actual scene
    PC::Ptr scene;
    //path to pel database
    boost::filesystem::path db_path;

    //class behaviour
    bool calibration;
    int iterations, neighbors;
    double clus_tol;
    int downsampling;
    bool busy, up_broadcaster, up_tracker;

    //PEL object
    PoseEstimation pe;

    //method to extract clusters of objects in a table top scenario
    int extract_clusters();
    //perform estimation
    void estimate();
    //estimate service callback  
    bool cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res);
    
    //custom spin method
    void spin_once();
};
#define _INCL_ESTIMATOR
#endif
