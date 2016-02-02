// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
