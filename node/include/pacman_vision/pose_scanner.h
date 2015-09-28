#ifndef _INCL_POSE_SCANNER
#define _INCL_POSE_SCANNER

#include <pacman_vision/config.h>
//Utility
#include <pacman_vision/utility.h>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
// ROS generated headers
#include <turn_table_interface/setPos.h>
#include <turn_table_interface/getPos.h>
#include <pacman_vision_comm/acquire.h>
//general utilities
#include <ctime>
#include <algorithm>
#include <boost/filesystem/path.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
//Storage
#include <pacman_vision/storage.h>

class VisionNode;

class PoseScanner
{
  friend class VisionNode;

  public:
    PoseScanner(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~PoseScanner();
    //Eigen alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    boost::shared_ptr<Storage> storage;
    //Service Server
    ros::ServiceServer srv_acquire;

    //subscriber to clickedpoints
    ros::Subscriber sub_clicked;

    //method to move turn table
    bool set_turn_table_pos(float pos);
    //method to read turn table position
    float get_turn_table_pos();

    //table transform
    boost::shared_ptr<Eigen::Matrix4f> table_transform;

    //Save location informations
    boost::filesystem::path work_dir;
    boost::filesystem::path session_dir;
    boost::posix_tim::ptime timestamp;

    //Scene processed
    PC::Ptr scene;

    //table pass in degrees
    int table_pass;

    //acquire service callback
    bool cb_acquire(pacman_vision_comm::acquire::Request& req, pacman_vision_comm::acquire::Response& res);

    //custom spin method
    void spin_once();
};
#endif
