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
#ifndef _SENSOR_PROCESSOR_H_
#define _SENSOR_PROCESSOR_H_

#include <pacv_config.h>
#include <common/dynamic_modules.hpp>
#include <basic_node/sensor_config.hpp>
//ROS
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>

#ifdef PACV_KINECT2_SUPPORT
#include <basic_node/kinect2_processor.h>
#endif

////////////////////////////////////////////////////////////////////////////////
namespace pacv
{
class SensorProcessor: public Module<SensorProcessor>
{
    friend class Module<SensorProcessor>;
    public:
        SensorProcessor()=delete;
        SensorProcessor(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor);
        typedef std::shared_ptr<SensorProcessor> Ptr;
        SensorConfig::Ptr getConfig() const;
        //update behaviour based on config
        void update();
        //set configutation from ROS Params
        void setConfigFromRosparams();
        //update configuration to ROS params
        void updateRosparams();
    private:
        SensorConfig::Ptr config;
        //init with ros param
        void init();
        void deInit();
        //external subscriber to recieve a cloud
        ros::Subscriber sub_cloud;
        //internal kinect2 name handler (if present) and or
        //identity broadcaster
        tf::TransformBroadcaster kinect2_ref_brcaster;
        //subscribed topic
        std::string topic;
        //associated callback
        void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        //identity broadcaster at request
        void broadcast_identity();

        void spinOnce();
#ifdef PACV_KINECT2_SUPPORT
        //kinect2 processor
        Kinect2 kinect2;
#endif
};
}
#endif
