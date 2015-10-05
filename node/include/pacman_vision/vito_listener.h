#ifndef _INCL_LISTENER
#define _INCL_LISTENER

#include <pacman_vision/config.h>
//Utility
#include <pacman_vision/utility.h>
// ROS headers
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
//Storage
#include <pacman_vision/storage.h>
//Ros generated
#include <pacman_vision_comm/get_cloud_in_hand.h>
#include <pacman_vision/vito_geometry.h>

class VisionNode;

class Listener
{
  friend class VisionNode;

  public:
    Listener(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~Listener();
    //Eigen Alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    ros::NodeHandle nh;
    ros::ServiceServer srv_get_cloud;
    //callback get_cloud service
    bool cb_get_cloud_in_hand(pacman_vision_comm::get_cloud_in_hand::Request& req, pacman_vision_comm::get_cloud_in_hand::Response& res);
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    boost::shared_ptr<Storage> storage;

    double box_scale;
    std::vector<std::string> arm_naming;
    std::vector<std::string> detailed_hand_naming;
    //eigen transform
    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > left_arm, right_arm ;
    boost::shared_ptr<Eigen::Matrix4f> left_hand, right_hand, table;
    //tf transforms
    std::vector<tf::StampedTransform> left_arm_tf, right_arm_tf;
    tf::StampedTransform left_hand_tf, right_hand_tf, table_tf;

    //tf listener
    tf::TransformListener tf_listener;

    //behaviour
    bool listen_left_arm, listen_right_arm, listen_left_hand, listen_right_hand;
    //custom spinner
    void spin_once();

    //method to listen to vito transforms
    void listen_once();
    //method to listen to table transform
    void listen_table();
    //method to listen to a piece of soft hand, and crop out its bb.
    void listen_and_crop_detailed_hand_piece(bool right, size_t idx, PC::Ptr& cloud);
    //method to listen to a piece of soft hand, and extract its content.
    void listen_and_extract_detailed_hand_piece(bool right, size_t idx, PC::Ptr& cloud, PC::Ptr& piece);
};
#endif
