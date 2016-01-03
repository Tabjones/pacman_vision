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
    private:
        SensorConfig::Ptr config;
        //init with ros param
        void init();
        //external subscriber to recieve a cloud
        ros::Subscriber sub_cloud;
        //subscribed topic
        std::string topic;
        //and track when we were disableb
        bool was_disabled;
        //associated callback
        void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

        void spinOnce();
#ifdef PACV_KINECT2_SUPPORT
        //internal kinect2 name handler (if present)
        tf::TransformBroadcaster kinect2_ref_brcaster;
        //kinect2 processor
        Kinect2 kinect2;
#endif
};
}
#endif
