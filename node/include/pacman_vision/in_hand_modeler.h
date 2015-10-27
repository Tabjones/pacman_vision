#ifndef _INCL_POSE_SCANNER
#define _INCL_POSE_SCANNER

#include <pacman_vision/config.h>
//Utility
#include <pacman_vision/utility.h>
//ROS
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
// ROS generated headers
#include <pacman_vision_comm/acquire.h>
//general utilities
#include <ctime>
#include <algorithm>
#include <boost/filesystem/path.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
//Storage
#include <pacman_vision/storage.h>

class VisionNode;

class InHandModeler
{
    friend class VisionNode;

    public:
    InHandModeler(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~InHandModeler(){};
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

        //publisher of model
        ros::Publisher pub_model;

        //transform broadcaster and listener
        tf::TransformBroadcaster tf_table_trans;
        tf::TransformListener tf_listener;

        //model transforms
        Eigen::Matrix4f T_km, T_mk;
        //has a model trasform ?
        bool has_transform;

        //Acquired model so far
        PC::Ptr model;

        //Save location informations
        boost::filesystem::path work_dir;
        boost::filesystem::path session_dir;
        boost::posix_time::ptime timestamp;

        //Scene processed
        PC::Ptr scene;

        //ignore accidentally clicked points
        bool ignore_clicked_point;

        bool
        computeModelTransform(PT pt, float nx, float ny, float nz);
        //save acquired model to disk
        bool
        saveModel();
        //acquire service callback
        bool
        cb_acquire(pacman_vision_comm::acquire::Request& req,
                                pacman_vision_comm::acquire::Response& res);
        //Callback from clicked_point
        void
        cb_clicked(const geometry_msgs::PointStamped::ConstPtr& msg);

        //custom spin method
        void
        spin_once();
};
#endif
