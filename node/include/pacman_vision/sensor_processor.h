#ifndef _SENSOR_PROCESSOR_H_
#define _SENSOR_PROCESSOR_H_

#include <pacman_vision/config.h>
#include <pacman_vision/module_config.h>
#include <pacman_vision/dynamic_modules.hpp>
//libfreenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
//ROS
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>

class Kinect2
{
    public:
        Kinect2();
        virtual ~Kinect2();
        //init Kinect2 Device
        bool initDevice();
        //Start Kinect2
        bool start();
        //Stop Kinect2
        bool stop();
        //close kinect2
        bool close();
        bool isStarted() const;
        bool isInitialized() const;
        //Recieve and process data
        void processData();
        //Compute a PointCloud from processed data
        void computePointCloud(PTC::Ptr &out_cloud);
    private:
        //internal kinect2 handling
        bool started, initialized;
        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *device;
        libfreenect2::PacketPipeline *packetPipeline;
        libfreenect2::Registration *registration;
        libfreenect2::SyncMultiFrameListener *listener_color;
        libfreenect2::SyncMultiFrameListener *listener_depth;
        libfreenect2::Freenect2Device::ColorCameraParams colorParams;
        libfreenect2::Freenect2Device::IrCameraParams irParams;
        libfreenect2::Frame *colorFrame, *irFrame, *depthFrame;
        libfreenect2::Frame *undistorted, *registered;
        libfreenect2::FrameMap frames_c, frames_d;
};


////////////////////////////////////////////////////////////////////////////////
class SensorProcessor: public Module<SensorProcessor>
{
    friend class Module<SensorProcessor>;
    public:
        SensorProcessor()=delete;
        SensorProcessor(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor, const ros::Rate rate);
        typedef std::shared_ptr<SensorProcessorConfig> ConfigPtr;
        typedef std::shared_ptr<SensorProcessor> Ptr;
        void updateIfNeeded (const SensorProcessor::ConfigPtr conf, bool reset=false);
        SensorProcessor::ConfigPtr getConfig() const;
    private:
        //config protection
        std::mutex mtx_config;
        SensorProcessor::ConfigPtr config;
        //init with ros param
        void init();
        //external subscriber to recieve a cloud
        ros::Subscriber sub_cloud;
        //associated callback
        void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        //internal kinect2 handler
        Kinect2 kinect2;
        tf::TransformBroadcaster kinect2_ref_brcaster;

        void spinOnce();
};
#endif
