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
#include <basic_node/sensor_processor.h>
#include <common/common_pcl.h>
#include <common/common_ros.h>
#include <pacv_config.h> //cmake generated

#ifdef PACV_KINECT2_SUPPORT
#include <basic_node/kinect2_processor.h>
#endif

namespace pacv
{
SensorProcessor::SensorProcessor(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor):
    Module<SensorProcessor>(n,ns,stor)
{
    //default to asus xtion
    config = std::make_shared<SensorConfig>();
    topic = "/camera/depth_registered/points";
    //tmp set param to dump into default
    // nh.setParam("internal", false);
    // nh.setParam("topic", "/camera/depth_registered/points");
    // nh.setParam("name", "kinect2_optical_frame");
    ////////////////////////////////////
}
void
SensorProcessor::deInit()
{
    //nothing to do actually this is to suppress annoying warning
}

void
SensorProcessor::updateRosparams()
{
    for (const auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if (!config->get(key, val))
            ROS_WARN("[SensorProcessor::%s]\tFailed to get key:%s from Config",__func__,key.c_str());
        else
            nh->setParam(key, val);
    }
}

void
SensorProcessor::setConfigFromRosparams()
{
    //init node params
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh->getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[SensorProcessor::%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[SensorProcessor::%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
}

void
SensorProcessor::init()
{
    if(!nh){
        ROS_ERROR("[SensorProcessor::%s]\tNode Handle not initialized, Module must call spawn() first",__func__);
        return;
    }
    setConfigFromRosparams();
    update();
    // //Fire subscriber or kinect2
    // bool internal;
    // config->get("internal", internal);
    // if(internal)
    // {
    //     kinect2.initDevice();
    //     kinect2.start();
    //     return;
    // }
    // config->get("topic", topic);
    // //fire the subscriber
    // sub_cloud = nh->subscribe(nh->resolveName(topic), 5, &SensorProcessor::cb_cloud, this);
}

SensorConfig::Ptr
SensorProcessor::getConfig() const
{
    return config;
}

///This needs to be called every time subscription or type changes,
///also when enable/disable is toggled. PacmanVision takes care of it
void SensorProcessor::update()
{
    if (isDisabled()){
#ifdef PACV_KINECT2_SUPPORT
        if (kinect2.isStarted() || kinect2.isInitialized()){
            kinect2.stop();
            kinect2.close();
        }
#endif
        sub_cloud.shutdown();
        return;
    }
#ifdef PACV_KINECT2_SUPPORT
    bool internal;
    config->get("internal", internal);
    if(internal){
        //Use internal kinect2, no subscriber needed
        sub_cloud.shutdown();
        if (!kinect2.isInitialized())
            kinect2.initDevice();
        if (!kinect2.isStarted())
            kinect2.start();
        return;
    }
    //internal is off, we use external subscriber
    //stop the internal kinect2
    if (kinect2.isStarted() || kinect2.isInitialized()){
        kinect2.stop();
        kinect2.close();
    }
#endif
    config->get("topic", topic);
    //fire the subscriber
    sub_cloud = nh->subscribe(nh->resolveName(topic), 5, &SensorProcessor::cb_cloud, this);
}

void SensorProcessor::spinOnce()
{
    queue_ptr->callAvailable(ros::WallDuration(0));
#ifdef PACV_KINECT2_SUPPORT
    if (kinect2.isStarted()){
        //get a cloud from  kinect2, we also need to publish a  tf of the sensor
        //and write also the ref frame inside the point cloud we send downstream
        kinect2.processData();
        PTC::Ptr scene;
        kinect2.computePointCloud(scene);
        std::string ref_name;
        config->get("name", ref_name);
        scene->header.frame_id = ref_name;
        pcl_conversions::toPCL(ros::Time::now(), scene->header.stamp);
        //Send it to good riddance downstream!
        storage->writeScene(scene);
        tf::Vector3 t(0.0385,0,0);
        tf::Quaternion q;
        q.setRPY(0,0,0);
        tf::Transform T(q,t);
        kinect2_ref_brcaster.sendTransform(tf::StampedTransform(T, ros::Time::now(),"kinect2_link", ref_name.c_str()));
        storage->writeSensorFrame(ref_name);
    }
#endif
    //nothing to do for point clouds if we dont use internal kinect2,
    //callback takes care of it all

    //broadcast identity between cameras if requested
    broadcast_identity();
}

void SensorProcessor::cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PTC::Ptr scene = boost::make_shared<PTC>();
    pcl::fromROSMsg (*msg, *scene);
    // Save untouched scene into storage bye bye
    storage->writeScene(scene);
    storage->writeSensorFrame(scene->header.frame_id);
}

void
SensorProcessor::broadcast_identity()
{
    bool val;
    config->get("broadcast_identity_tf", val);
    if (val){
        tf::Transform T;
        T.setIdentity();
        kinect2_ref_brcaster.sendTransform(tf::StampedTransform(T, ros::Time::now(), "kinect2_link", "camera_link"));
    }
}

}
