#ifndef _SERVICER_HPP_
#define _SERVICER_HPP_

#include <common/dynamic_modules.hpp>
#include <common/common_ros.h>
// ROS services
#include <pacman_vision_comm/estimate.h>
#include <pacman_vision_comm/stop_track.h>
#include <pacman_vision_comm/track_object.h>
#include <pacman_vision_comm/get_scene.h>
#include <pacman_vision_comm/grasp_verification.h>
#include <pacman_vision_comm/get_cloud_in_hand.h>
#include <pacman_vision_comm/start_modeler.h>
#include <pacman_vision_comm/stop_modeler.h>


namespace pacv
{
class Servicer: public Module<Servicer>
{
    friend class Module<Servicer>;
    public:
        Servicer()=delete;
        Servicer(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor):
            Module<Servicer>(n,ns,stor), busy(false)
        {}
        typedef std::shared_ptr<Servicer> Ptr;
        inline bool isBusy() const
        {
            return busy;
        }
    private:
        bool busy;
        ros::ServiceClient client;
        //init with ros param
        void init()
        {
            if(!nh){
                ROS_ERROR("[Servicer::%s]\tNode Handle not initialized, Module must call spawn() first",__func__);
                return;
            }
            client = nh.serviceClient<pacman_vision_comm::estimate>("/pacman_vsion/estimator/estimate");
        }
        //deinit to free memory
        void deInit()
        {
        }
        //spin once
        void spinOnce()
        {}
        void call()
        {
            pacman_vision_comm::estimate srv;
            busy = true;
            if (client.call(srv))
                //success
        }
};
}//namespace
#endif
