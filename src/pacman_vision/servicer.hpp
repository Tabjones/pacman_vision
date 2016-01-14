#ifndef _SERVICER_HPP_
#define _SERVICER_HPP_

#include <common/common_ros.h>
#include <common/common_std.h>
// ROS services
#include <pacman_vision_comm/estimate.h>
#include <pacman_vision_comm/stop_track.h>
#include <pacman_vision_comm/track_object.h>
#include <pacman_vision_comm/get_scene.h>
#include <pacman_vision_comm/grasp_verification.h>
#include <pacman_vision_comm/get_cloud_in_hand.h>
#include <pacman_vision_comm/start_modeler.h>
#include <pacman_vision_comm/stop_modeler.h>

#include <thread>


namespace pacv
{
class Servicer
{
    public:
        Servicer()=delete;
        Servicer(const ros::NodeHandle n, const std::string ns): busy(false),
                name_space(ns)
        {
            father_nh = std::make_shared<ros::NodeHandle>(n);
        }
        typedef std::shared_ptr<Servicer> Ptr;
        inline bool isBusy() const
        {
            return busy;
        }
        template <typename Service>
        bool spawn(Service &srv, std::string topic)
        {
            nh = std::make_shared<ros::NodeHandle>(*father_nh, name_space);
            client = nh.serviceClient<Service>(topic);
            caller
        }
    private:
        bool busy;
        std::shared_ptr<ros::NodeHandle> father_nh, nh;
        std::string name_space;
        ros::ServiceClient client;
        std::thread caller;
        //init with ros param
        void init()
        {
        }
        template <typename Service>
        void call(Service &srv, std::string topic)
        {
            busy = true;
            if (client.call(srv))
                //success
        }
};
}//namespace
#endif
