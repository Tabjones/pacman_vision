#ifndef _LISTENER_H_
#define _LISTENER_H_

#include <common/dynamic_modules.hpp>
#include <common/common_pcl.h>
#include <common/common_ros.h>
#include <listener/listener_config.hpp>
#include <listener/vito_geometry.h>
// ROS generated headers
#include <pacman_vision_comm/get_cloud_in_hand.h>
//ROS
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

namespace pacv
{
class Listener: public Module<Listener>
{
    friend class Module<Listener>;
    public:
        Listener()=delete;
        Listener(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor);
        typedef std::shared_ptr<Listener> Ptr;
        ListenerConfig::Ptr getConfig() const;
        //update rosparams
        void updateRosparams();
        //Eigen alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        //Configuration
        ListenerConfig::Ptr config;
        //init with ros param
        void init();
        //deinit to free memory
        void deInit();
        //Service Server
        ros::ServiceServer srv_get_in_hand;
        //marker broadcaster
        ros::Publisher pub_markers;
        //tf
        tf::TransformListener tf_listener;
        //marker
        std::shared_ptr<visualization_msgs::MarkerArray> marks;
        //listened transforms
        std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> left_arm, right_arm, left_hand, right_hand;
        //get cloud in hand service callback
        bool cb_get_in_hand(pacman_vision_comm::get_cloud_in_hand::Request& req, pacman_vision_comm::get_cloud_in_hand::Response& res);
        //spin once
        void spinOnce();
        //create new markers from transforms
        void create_markers();
        //publish markers
        void publish_markers();
        //listen arm or hand
        void listen(std::string which, std::string component);
};
}//namespace
#endif
