#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <common/dynamic_modules.hpp>
#include <common/common_pcl.h>
#include <common/common_ros.h>
#include <recognition/tracker_config.hpp>
#include <basic_node/basic_node_config.hpp> //needs to know downsampling leaf
//PCL
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/registration/correspondence_rejection_distance.h>
// #include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
// ROS generated headers
#include <pacman_vision_comm/track_object.h>
#include <pacman_vision_comm/stop_track.h>
#include <pacman_vision_comm/grasp_verification.h>
//ROS
#include <visualization_msgs/MarkerArray.h>

namespace pacv
{
class Tracker: public Module<Tracker>
{
    friend class Module<Tracker>;
    public:
        Tracker()=delete;
        Tracker(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor);
        typedef std::shared_ptr<Tracker> Ptr;
        TrackerConfig::Ptr getConfig() const;
        //Eigen alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        //Configuration
        TrackerConfig::Ptr config;
        BasicConfig::Ptr basic_config;
        //init with ros param
        void init();
        //deinit to free memory
        void deInit();
        //Service Server
        ros::ServiceServer srv_track_object, srv_grasp, srv_stop;
        //marker broadcaster
        ros::Publisher pub_markers;
        //marker
        std::shared_ptr<visualization_msgs::MarkerArray> marks;
        //tracker transform
        std::shared_ptr<Eigen::Matrix4f> transform;
        Eigen::Matrix4f T_rotx, T_rotz, T_roty; //for disturbances
        //name and id of tracker object
        std::pair<std::string, std::string> obj_name; //name/id
        //index of tracked object in storage
        int index;
        //actual scene
        PXC::Ptr scene;
        //model
        PXC::Ptr model;
        //loaded model
        PXC::Ptr orig_model;
        //model centroid
        PX model_centroid;
        //internal state
        bool started, lost_it; //, manual_disturbance;
        //bb factor
        float factor;
        //leaf size to model downsampling
        double leaf;
        //counter
        int error_count,centroid_counter;
        //transform type (unused)
        int type;
        //BBs
        std::shared_ptr<Box> bounding_box;
        //rejector distance
        float rej_distance;
        //ICP fitness
        double fitness;
        //icp object
        pcl::IterativeClosestPoint<PX, PX,float> icp;
        //correspondences
        pcl::registration::CorrespondenceRejectorDistance::Ptr crd;
        pcl::registration::TransformationEstimationDualQuaternion<PX,PX,float>::Ptr teDQ;
        //filters
        pcl::VoxelGrid<PX> vg;
        //spin once
        void spinOnce();
        //publish markers
        void publish_markers();
        //create new markers from scratch
        void create_markers();
        //create a bounding_box marker
        void create_bb_marker(geometry_msgs::Pose pose);
        //update current marker transform
        void update_markers();
        //track_object service callback
        bool cb_track_object(pacman_vision_comm::track_object::Request& req, pacman_vision_comm::track_object::Response& res);
        //stop tracker service callback
        bool cb_stop_tracker(pacman_vision_comm::stop_track::Request& req, pacman_vision_comm::stop_track::Response& res);
        //grasp_verification service callback
        bool cb_grasp(pacman_vision_comm::grasp_verification::Request& req, pacman_vision_comm::grasp_verification::Response& res);
        //tracker methods
        void track();
        void find_object_in_scene();
        void setBasicNodeConfig(BasicConfig::Ptr config);
};
}//namespace
#endif
