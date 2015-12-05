#ifndef _VISION_NODE_H_
#define _VISION_NODE_H_

#include <pacman_vision/config.h>
#include <pacman_vision/common.h>
#include <pacman_vision/storage.h>
// ROS headers
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
//PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
// ROS generated headers
#include <pacman_vision_comm/get_scene.h>
#include <pacman_vision/pacman_visionConfig.h>

//TODO from here
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
#include <pacman_vision/kinect2_processor.h>
#endif

//Modules
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
#include <pacman_vision/estimator.h>
#include <pacman_vision/tracker.h>
#endif

#include <pacman_vision/broadcaster.h>
#include <pacman_vision/vito_listener.h>
#include <pacman_vision/supervoxels.h>
#include <pacman_vision/in_hand_modeler.h>

class VisionNode
{
    struct SensorParams
    {
        /*
         * Sensor reference frame. Either external kinect2, asus xtion
         * or internal kinect2 processor
         */
        std::string ref_frame;
        /*
         * which sensor to use: 0 internal, 1 external kinect2_bridge,
         * 2 external openni2
         */
        int type;
        /*
         * use EXTERNAL kinect2 hd(1920x1080) <2>, qhd(960x540) <1>,
         * or sd(512x424) <0>  (i.e. kinect2_bridge)
         * this only has a meaning if type == 1
         */
        int resolution;
        //sensor subscribers need update
        bool needs_update;
    };

    public:
    VisionNode();
    //custom spin method
    void
    spin_once();
    //method to say goodbye
    void
    shutdown();
    //node handle
    ros::NodeHandle nh;
    //Takes care of Eigen Alignment on Fixed-Size Containers
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        //Globally disable all functionalities
        bool master_disable;
        //bools to control modules
        bool en_estimator, en_tracker, en_broadcaster, en_listener;
        bool en_supervoxels, en_modeler;
        /*
         * bool to initialize rqt_reconfigure with user parameters
         * instead of those written in cfg file
         */
        bool rqt_init;
        //sensor information
        SensorParams sensor;

        //transforms to crop out vito arms and hands
        //only use them if vito listener is active
        boost::shared_ptr<std::vector<Eigen::Matrix4f,
            Eigen::aligned_allocator<Eigen::Matrix4f>>> left_arm, right_arm;
        boost::shared_ptr<Eigen::Matrix4f> left_hand, right_hand;
        //table transform from vito listener
        boost::shared_ptr<Eigen::Matrix4f> table_trans;
        //crop or not
        bool crop_r_arm, crop_l_arm, crop_r_hand, crop_l_hand, detailed_hand_crop;
        //use table transformation to apply passthrough or not
        bool use_table_trans;
        //global geometry scale (robot meshes and filters)
        double box_scale;
        //tells which hand is grasping an object
        bool grasp_with_right;

        //Service Server to retrieve processed scene
        ros::ServiceServer srv_get_scene;
        //Message Subscriber to read from kinect
        ros::Subscriber sub_kinect;
        //Message Publisher to republish processed scene
        ros::Publisher pub_scene;
        //pointer to processed and acquired point cloud
        PC::Ptr scene_processed;
        PC::Ptr scene;
        //tf broadcaster for sesor reference frame (used if internal processor in enabled)
        tf::TransformBroadcaster tf_sensor_ref_frame_brcaster;
        //Shared pointer of Storage (to be shared to modules)
        boost::shared_ptr<Storage> storage;
#ifdef PACMAN_VISION_WITH_KINECT2_SUPPORT
        //Shared pointer of Kinect2Processor
        boost::shared_ptr<Kinect2Processor> kinect2;
#endif
#ifdef PACMAN_VISION_WITH_PEL_SUPPORT
        //Shared pointers of modules
        boost::shared_ptr<Estimator> estimator_module;
        boost::shared_ptr<Tracker> tracker_module;
        //spinners
        //estimator
        boost::thread estimator_driver;
        void
        spin_estimator();
        //tracker
        boost::thread tracker_driver;
        void
        spin_tracker();
#endif
        boost::shared_ptr<Broadcaster> broadcaster_module;
        boost::shared_ptr<Listener> listener_module;
        boost::shared_ptr<Supervoxels> supervoxels_module;
        boost::shared_ptr<InHandModeler> modeler_module;
        //slave spinner threads for modules
        //broadcaster
        boost::thread broadcaster_driver;
        void
        spin_broadcaster();
        //in hand modeler
        boost::thread modeler_driver;
        void
        spin_modeler();
        //listener
        boost::thread listener_driver;
        void
        spin_listener();
        //supervoxels
        boost::thread supervoxels_driver;
        void
        spin_supervoxels();

        //Service callback for srv_get_scene
        bool
        cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res);
        //Message callback, for sub_kinect
        void
        cb_kinect(const sensor_msgs::PointCloud2::ConstPtr& message);

        //filter parameters
        bool filter, downsample, keep_organized, plane;
        boost::shared_ptr<Box> limits;
        double leaf,plane_tol;

        //Dynamic Reconfigure//
        //Server
        dynamic_reconfigure::Server<pacman_vision::pacman_visionConfig> dyn_srv;
        //Callback
        void
        cb_reconfigure(pacman_vision::pacman_visionConfig &config, uint32_t level);

        //method to enable/disable modules
        void
        check_modules();
        //Check which sensor to use
        void
        check_sensor();

        //Defined into node utils//
        //Publish scene processed
        void
        publish_scene_processed();
        //Process scene method (read scene -> write scene_processed)
        void
        process_scene();
        //Create a box marker
        void
        create_arm_box_marker(Eigen::Matrix4f& t, visualization_msgs::Marker &marker, const Box lim, int i, bool right=true);
        void
        create_hand_box_marker(Eigen::Matrix4f& t, visualization_msgs::Marker &marker, const Box lim, bool right=true);
        //Crop out a vito arm
        void
        crop_arm(PC::Ptr source, PC::Ptr& dest, bool right=true);
        //Crop out a softhand, approximately with one bounding box
        void
        crop_hand(PC::Ptr source, PC::Ptr& dest, bool right=true);
};

#endif
