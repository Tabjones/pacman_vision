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
// #include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/correspondence.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
// #include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/visualization/pcl_visualizer.h>
// ROS generated headers
#include <pacman_vision_comm/start_modeler.h>
#include <pacman_vision_comm/stop_modeler.h>
//general utilities
#include <ctime>
#include <list>
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

        //pointers to queue
        std::list<PC>::iterator align_it, fuse_it;
        //leaf size for frames fusion and frame alignment
        const float leaf_f, leaf;

        //subscriber to clickedpoints
        ros::Subscriber sub_clicked;

        //publisher of model
        ros::Publisher pub_model;

        //transform broadcaster and listener
        tf::TransformBroadcaster tf_broadcaster;
        tf::TransformListener tf_listener;

        //model transforms
        Eigen::Matrix4f T_km, T_mk;
        //has a model trasform ?
        bool has_transform;
        //is acquiring clouds ? also start acquisition
        bool do_acquisition;
        //is aligning clouds?
        bool do_alignment, done_alignment;
        //is fusing clouds ?
        bool do_frame_fusion, done_frame_fusion;
        //count how many frames have been acquired and not fused
        int frames, not_fused;

        //pointclouds sequence
        //in kinect reference frame
        std::list<PC> cloud_sequence;
        //model and downsampled model in model reference frame
        PC::Ptr model, model_ds;

        //Voxelgrid downsampling
        pcl::VoxelGrid<PT> vg;

        //Registration
        // pcl::SampleConsensusPrerejective<PT, PT, pcl::FPFHSignature33> alignment;
        pcl::MultiscaleFeaturePersistence<PT,pcl::FPFHSignature33> persistance;
        pcl::registration::TransformationEstimationDualQuaternion<PT,PT,float>::Ptr teDQ;
        pcl::NormalEstimationOMP<PT, NT> ne;
        pcl::FPFHEstimationOMP<PT, NT, pcl::FPFHSignature33>::Ptr fpfh;

        //Octrees
        pcl::octree::OctreePointCloudAdjacency<PT> oct_adj;
        pcl::octree::OctreePointCloudChangeDetector<PT> oct_cd_frames;
        pcl::octree::OctreePointCloudChangeDetector<PT> oct_cd;

        //Feature comparation
        typedef search::FlannSearch<pcl::FPFHSignature33, flann::L2<float>> SearchT;
        typedef typename SearchT::FlannIndexCreatorPtr CreatorT;
        typedef typename SearchT::KdTreeMultiIndexCreator IndexT;
        typedef typename SearchT::PointRepresentationPtr RepT;

        //Threads stuff
        boost::thread fusion_driver, alignment_driver;
        ros::NodeHandle nh_fusion, nh_alignment;
        boost::shared_ptr<ros::CallbackQueue> queue_fusion, queue_alignment;
        boost::mutex mtx_seq, mtx_model;


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
        //Perform alignment of sequence clouds
        void
        alignSequence();
        //discard too similar frames
        void
        fuseSimilarFrames();

        //custom spin method
        void
        spin_once();
};
#endif
