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
    actual_topic = nh.resolveName("/camera/depth_registered/points");
    config = std::make_shared<SensorConfig>();
    //tmp set param to dump into default
    // nh.setParam("internal", false);
    // nh.setParam("topic", "/camera/depth_registered/points");
    // nh.setParam("name", "kinect2_optical_frame");
    ////////////////////////////////////
    init();
}

void
SensorProcessor::init()
{
    //init node params
#ifndef PACV_KINECT2_SUPPORT
    //we just need to read which topic to subscribe
    std::string tp;
    if(nh.getParam("topic",tp)){
            if(!config->set("topic", tp))
                ROS_WARN("[%s]\tFailed to set key:topic into Config",__func__);
    }
    else
        ROS_WARN("[%s]\tKey:topic not found on parameter server",__func__);
    return;
#endif
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh.getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
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
        config->get("topic", actual_topic);
        //fire the subscriber
        sub_cloud = nh.subscribe(actual_topic, 5, &SensorProcessor::cb_cloud, this);
        was_disabled = false;
        return;
    }
    std::string topic;
    config->get("topic", topic);
    if (topic.compare(actual_topic) != 0){
        //we need to change subscriber
        sub_cloud = nh.subscribe(topic, 5, &SensorProcessor::cb_cloud, this);
        actual_topic = topic;
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
    }
#endif
    //nothing to do if we dont use internal kinect2, callback takes care of it all
}

void SensorProcessor::cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PTC::Ptr scene;
    pcl::fromROSMsg (*msg, *scene);
    // Save untouched scene into storage bye bye
    storage->write_scene(scene);
}
}
