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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/visualization/pcl_visualizer.h>
// ROS generated headers
#include <pacman_vision_comm/start_modeler.h>
#include <pacman_vision_comm/stop_modeler.h>
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
        ros::ServiceServer srv_start;
        ros::ServiceServer srv_stop;

        //subscriber to clickedpoints
        ros::Subscriber sub_clicked;

        //publisher of model
        ros::Publisher pub_model;

        //transform broadcaster and listener
        ////TODO move this broadcasting to broadcaster module !
        tf::TransformBroadcaster tf_broadcaster;
        tf::TransformListener tf_listener;

        //model transforms
        Eigen::Matrix4f T_km, T_mk;
        //has a model trasform ?
        bool has_transform;
        //needs to iterate ?
        bool do_iterations;

        //pointclouds
        //actual scene in kinect and model reference frame
        PC::Ptr actual_k, actual_m;
        //previous scene in kinect and model reference frame
        PC::Ptr previous_k, previous_m;
        //model and downsampled model
        PC::Ptr model, model_ds;

        //Voxelgrid downsampling
        pcl::VoxelGrid<PT> vg;

        //ICP object
        pcl::IterativeClosestPoint<PT, PT, float> icp;
        pcl::registration::CorrespondenceRejectorDistance::Ptr crd;
        pcl::registration::TransformationEstimationDualQuaternion<PT, PT, float>::Ptr teDQ;

        //Dynamic reconfigurable parameters
        //ignore accidentally clicked points
        bool ignore_clicked_point;
        //Save location informations
        boost::filesystem::path work_dir;
        //Model leaf size
        double model_ls;

        //Methods
        bool
        computeModelTransform(PT pt, float nx, float ny, float nz);
        //save acquired model to disk
        bool
        saveModel();
        //acquire service callback
        bool
        cb_start(pacman_vision_comm::start_modeler::Request& req, pacman_vision_comm::start_modeler::Response& res);
        //stop service callback
        bool
        cb_stop(pacman_vision_comm::stop_modeler::Request& req, pacman_vision_comm::stop_modeler::Response& res);
        //Callback from clicked_point
        void
        cb_clicked(const geometry_msgs::PointStamped::ConstPtr& msg);
        //Do one step of iterations
        void
        iterate_once();

        //custom spin method
        void
        spin_once();
};
#endif
