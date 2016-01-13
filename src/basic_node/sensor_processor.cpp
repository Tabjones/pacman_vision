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
    Module<SensorProcessor>(n,ns,stor), was_disabled(false)
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
SensorProcessor::init()
{
    if(!nh){
        ROS_ERROR("[SensorProcessor::%s]\tNode Handle not initialized, Module must call spawn() first",__func__);
        return;
    }
    //init node params
#ifndef PACV_KINECT2_SUPPORT
    //we just need to read which topic to subscribe
    if(nh->getParam("topic",topic)){
            if(!config->set("topic", topic))
                ROS_WARN("[SensorProcessor::%s]\tFailed to set key:topic into Config",__func__);
    }
    else
        ROS_WARN("[SensorProcessor::%s]\tKey:topic not found on parameter server",__func__);
#endif
#ifdef PACV_KINECT2_SUPPORT
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh.getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[SensorProcessor::%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[SensorProcessor::%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
    //Fire subscriber or kinect2
    bool internal;
    config->get("internal", internal);
    if(internal)
    {
        kinect2.initDevice();
        kinect2.start();
        return;
    }
#endif
    config->get("topic", topic);
    //fire the subscriber
    sub_cloud = nh->subscribe(nh->resolveName(topic), 5, &SensorProcessor::cb_cloud, this);
}

SensorConfig::Ptr
SensorProcessor::getConfig() const
{
    return config;
}

void SensorProcessor::update()
{
    if (this->isDisabled()){
#ifdef PACV_KINECT2_SUPPORT
        if (kinect2.isStarted() || kinect2.isInitialized()){
            kinect2.stop();
            kinect2.close();
        }
#endif
        sub_cloud.shutdown();
        was_disabled = true;
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
        if(was_disabled)
            was_disabled=false;
        return;
    }
    //then we use external subscriber
    //stop the internal kinect2
    if (kinect2.isStarted() || kinect2.isInitialized()){
        kinect2.stop();
        kinect2.close();
    }
#endif
    if (was_disabled){
        config->get("topic", topic);
        //fire the subscriber
        sub_cloud = nh->subscribe(nh->resolveName(topic), 5, &SensorProcessor::cb_cloud, this);
        was_disabled = false;
        return;
    }
    config->get("topic", topic);
    if (topic.compare(sub_cloud.getTopic()) != 0){
        //we need to change subscriber, topic changed
        sub_cloud = nh->subscribe(nh->resolveName(topic), 5, &SensorProcessor::cb_cloud, this);
        return;
    }
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
        storage->write_scene(scene);
        tf::Vector3 t(0.0385,0,0);
        tf::Quaternion q;
        q.setRPY(0,0,0);
        tf::Transform T(q,t);
        kinect2_ref_brcaster.sendTransform(tf::StampedTransform(T, ros::Time::now(),"kinect2_anchor", ref_name.c_str()));
        storage->write_sensor_ref_frame(ref_name);
    }
#endif
    //nothing to do if we dont use internal kinect2, callback takes care of it all
}

void SensorProcessor::cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PTC::Ptr scene = boost::make_shared<PTC>();
    pcl::fromROSMsg (*msg, *scene);
    // Save untouched scene into storage bye bye
    storage->write_scene(scene);
    storage->write_sensor_ref_frame(scene->header.frame_id);
}
}
