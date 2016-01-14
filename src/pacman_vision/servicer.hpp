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

        template <typename Service>
        void spawn(Service &srv, std::string topic)
        {
            busy = true;
            nh = std::make_shared<ros::NodeHandle>(*father_nh, name_space);
            client = nh->serviceClient<Service>(topic);
            //actually easier to use a lambda than monkeying around with function
            //pointers and templates!
            caller = std::thread( [&]{this->call(srv);} );
        }
        bool postCallClean()
        {
            if(!busy && caller.joinable()){
                //means the thread was spawnd and it has finished
                caller.join();
                nh->shutdown();
                nh.reset();
                return true;
            }
            return false;
        }
    private:
        bool busy;
        std::shared_ptr<ros::NodeHandle> father_nh, nh;
        std::string name_space;
        ros::ServiceClient client;
        std::thread caller;

        template <typename Service>
        void call(Service &srv)
        {
            //this is blocking call
            if (!client.call(srv))
                //failed
                ROS_ERROR("[Servicer::%s]\tFailed to call service", __func__);
            busy = false;
        }
};
}//namespace
#endif
