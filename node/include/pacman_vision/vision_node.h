#ifndef _INCL_NODE

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS generated headers
#include "pacman_vision_comm/get_scene.h"
#include "pacman_vision/pacman_visionConfig.h"

//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/timer.hpp>

#include "pacman_vision/estimator.h"
#include "pacman_vision/broadcaster.h"
#include "pacman_vision/vito_listener.h"
#include "pacman_vision/tracker.h"

using namespace pcl;

class VisionNode
{
  typedef pcl::PointXYZRGB PT; //default point type
  typedef pcl::PointCloud<PT> PC; //default point cloud with default point type

  public:
    VisionNode();
    //custom spin method
    void spin_once();
    //method to say goodbye
    void shutdown();
    //node handle
    ros::NodeHandle nh;
  private:
    //bools to control modules
    bool en_estimator, en_tracker, en_broadcaster, en_listener;
    //bool to initialize rqt_reconfigure with user parameters
    bool rqt_init;

    // use kinect2 hd(1920x1080) <2>, qhd(960x540) <1>, or sd(530x270) <0>
    int kinect2_resolution;

    //transforms to crop out vito arms and hands
    //only use them if vito listener is active
    Eigen::Matrix4f left_2,left_3, left_4, left_5, left_6, left_7,
                    right_2, right_3, right_4, right_5, right_6, right_7,
                    left_hand, right_hand;
    //table transform from vito listener
    Eigen::Matrix4f table_trans;
    //crop or not
    bool crop_r_arm, crop_l_arm, crop_r_hand, crop_l_hand;

    //Service Server to retrieve processed scene
    ros::ServiceServer srv_get_scene;
    //Message Subscriber to read from openni2
    ros::Subscriber sub_openni;
    //Message Publisher to republish processed scene
    ros::Publisher pub_scene;
    //pointer to processed and acquired point cloud
    PC::Ptr scene_processed;
    PC::Ptr scene;

    //Shared pointers of modules
    boost::shared_ptr<Estimator> estimator_module;
    boost::shared_ptr<Broadcaster> broadcaster_module;
    boost::shared_ptr<Listener> listener_module;
    boost::shared_ptr<Tracker> tracker_module;
    //slave spinner threads for modules
    //estimator
    boost::thread estimator_driver;
    void spin_estimator();
    //broadcaster
    boost::thread broadcaster_driver;
    void spin_broadcaster();
    //tracker
    boost::thread tracker_driver;
    void spin_tracker();
    //listenerr
    boost::thread listener_driver;
    void spin_listener();

    //Service callback for srv_get_scene
    bool cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res);
    //Message callback, for sub_openni
    void cb_openni(const sensor_msgs::PointCloud2::ConstPtr& message);

    //filter parameters
    bool filter, downsample, keep_organized, plane;
    double xmin,xmax,ymin,ymax,zmin,zmax,leaf,plane_tol;

    //Dynamic Reconfigure//
    //Server
    dynamic_reconfigure::Server<pacman_vision::pacman_visionConfig> dyn_srv;
    //Callback
    void cb_reconfigure(pacman_vision::pacman_visionConfig &config, uint32_t level);

    //boost mutexes to protect intra-module copy
    boost::mutex mtx_scene;
    boost::mutex mtx_estimator;
    boost::mutex mtx_broadcaster;
    boost::mutex mtx_tracker;
    boost::mutex mtx_listener;

    //method to enable/disable modules
    void check_modules();

};

#define _INCL_NODE
#endif
