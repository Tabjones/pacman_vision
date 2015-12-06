#ifndef _BASIC_NODE_HPP_
#define _BASIC_NODE_HPP_

#include <pacman_vision/config.h>
#include <pacman_vision/common.h>
#include <pacman_vision/storage.h>
#include <pacman_vision/dynamic_modules.hpp>
// ROS headers
// #include <dynamic_reconfigure/server.h>
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

class BasicNode: public Module<BasicNode>
{
    friend class Module<BasicNode>;
    // TODO: Lets this  become a class and let it  handle subscribers, then save
    // scene into storage  basic node then just has to  read scene from storage,
    // this will of course incorporate the kinect2 processor (tabjones on Sunday
    // 06/12/2015)
    /*
     * struct SensorParams
    {
        std::string ref_frame;
        int type;
        int resolution;
        //sensor subscribers need update
        bool needs_update;
    };
    */

    public:
        BasicNode()=delete;
        BasicNode(const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate):
            is_running(false), spin_rate(rate), disabled(false)
        {}
        //Takes care of Eigen Alignment on Fixed-Size Containers
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        //Service Server to retrieve processed scene
        ros::ServiceServer srv_get_scene;
        /* TODO move into sensor class
         * //Message Subscriber to read from kinect
         * ros::Subscriber sub_kinect;
         */
        //Message Publisher to republish processed scene
        ros::Publisher pub_scene;
        //pointer to processed and acquired point cloud
        PC::Ptr scene_processed;
        PC::Ptr scene;
        /* TODO move into sensor class
         * //tf broadcaster for sensor reference frame (used if internal processor in enabled)
         * tf::TransformBroadcaster tf_sensor_ref_frame_brcaster;
         */
        //Service callback for srv_get_scene
        bool
        cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res);
        /*
         * //Message callback, for sub_kinect
         * void
         * cb_kinect(const sensor_msgs::PointCloud2::ConstPtr& message);
         */

        //filter parameters
        bool crop, downsample, keep_organized, plane;
        Box::Ptr limits; //cropbox limits
        double leaf, plane_tol;

        //Publish scene processed
        void
        publish_scene_processed();
        //Process scene method (read scene -> write scene_processed)
        void
        process_scene();
        //Create a box marker
        /* TODO move into listener, handle realtime cropping with services OR conditional variable
         * void
         * create_arm_box_marker(Eigen::Matrix4f& t, visualization_msgs::Marker &marker, const Box lim, int i, bool right=true);
         * void
         * create_hand_box_marker(Eigen::Matrix4f& t, visualization_msgs::Marker &marker, const Box lim, bool right=true);
         * //Crop out a vito arm
         * void
         * crop_arm(PC::Ptr source, PC::Ptr& dest, bool right=true);
         * //Crop out a softhand, approximately with one bounding box
         * void
         * crop_hand(PC::Ptr source, PC::Ptr& dest, bool right=true);
         */
};

#endif
