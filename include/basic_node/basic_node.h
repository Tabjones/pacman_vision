#ifndef _BASIC_NODE_H_
#define _BASIC_NODE_H_

#include <pacv_config.h>
#include <common/dynamic_modules.hpp>
#include <common/box.h>
#include <common/common_pcl.h>
#include <basic_node/basic_node_config.hpp>
// ROS headers
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
//PCL
#include <pcl/io/pcd_io.h>
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

#ifdef PACV_LISTENER_SUPPORT
#include <listener/listener_config.hpp>
#endif

namespace pacv
{
class BasicNode: public Module<BasicNode>
{
    friend class Module<BasicNode>;
    public:
        BasicNode()=delete;
        BasicNode(const std::string ns, const Storage::Ptr stor);
        typedef std::shared_ptr<BasicNode> Ptr;
        //update current configuration with new one
        // void updateIfNeeded(const BasicNode::ConfigPtr conf, bool reset=false);
        //Get current config
        BasicConfig::Ptr getConfig() const;
        //Takes care of Eigen Alignment on Fixed-Size Containers
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ///Updates marker to publish, when changed from gui
        void update_markers();
        ///Load config from rosparams
        void setConfigFromRosparams();
#ifdef PACV_LISTENER_SUPPORT
        void setListenerConfig(ListenerConfig::Ptr config)
        {
            list_config = config;
        }
#endif
    private:
        //Configuration
        BasicConfig::Ptr config;
#ifdef PACV_LISTENER_SUPPORT
        ListenerConfig::Ptr list_config;
#endif
        //Message Publisher to republish processed scene
        ros::Publisher pub_scene;
        //publisher for markers
        ros::Publisher pub_markers;
        //marker to publish
        std::shared_ptr<visualization_msgs::MarkerArray> marks; //,mark_plane;
        //server for get_scene_processed
        ros::ServiceServer srv_get_scene;
        PTC::Ptr scene_processed;
        //Service callback for srv_get_scene
        bool cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res);
        //init with ros params
        void init();
        void deInit();
        //Publish scene processed
        void publish_scene_processed() const;
        //Process scene method (read scene -> write scene_processed)
        void process_scene();
        //redefine spin and spinOnce
        void spin();
        void spinOnce();
        void downsamp_scene(const PTC::ConstPtr source, PTC::Ptr &dest);
        void segment_scene(const PTC::ConstPtr source, PTC::Ptr &dest);
        void publish_markers();
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
}//namespace
#endif
