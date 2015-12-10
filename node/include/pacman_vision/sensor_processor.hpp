#ifndef _SENSOR_PROCESSOR_HPP_
#define _SENSOR_PROCESSOR_HPP_

#include <pacman_vision/config.h>
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
        bool start()
        {
            device->start();
            started = true;
            return started;
        }
        //Stop Kinect2
        bool stop()
        {
            device->stop();
            started = false;
            return started;
        }
        //close kinect2
        bool close()
        {
            device->close();
            initialized=false;
            return initialized;
        }
        inline bool isStarted() const
        {
            return started;
        }
        inline bool isInitialized() const
        {
            return initialized;
        }
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

Kinect2::Kinect2(): started(false), initialized(false), device(0), packetPipeline(0),
    registration(0), listener_color(0), listener_depth(0)
{
    undistorted = new libfreenect2::Frame(512, 424, 4);
    registered = new libfreenect2::Frame(512, 424, 4);
}
Kinect2::~Kinect2()
{
    delete colorFrame;
    delete irFrame;
    delete depthFrame;
    delete undistorted;
    delete registered;
    delete packetPipeline;
    delete listener_depth;
    delete listener_color;
    delete registration;
    delete device;
}

bool
Kinect2::initDevice()
{
    if (freenect2.enumerateDevices() <= 0){
        ROS_ERROR("[Kinect2][%s]\tNo Kinect2 devices found",__func__);
        return (false);
    }
    //no support for multiple kinects as of now... just get default one.
    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    //Use OpenCL Packet pipeline processor
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    if (!packetPipeline)
        packetPipeline = new libfreenect2::OpenCLPacketPipeline();
#else
    ROS_ERROR("[Kinect2][%s]\tLibFreenect2 does not have OpenCL support. Please recompile it with OpenCL support",__func__);
    return (false);
#endif

    //Open kinect2
    device = freenect2.openDevice(serial, packetPipeline);
    if (device == 0){
        ROS_ERROR("[Kinect2][%s]\tFailed to open Kinect2 with serial %s",__func__, serial.c_str());
        return (false);
    }

    //create the listener
    listener_depth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    listener_color = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
    //and set them
    device->setColorFrameListener(listener_color);
    device->setIrAndDepthFrameListener(listener_depth);
    //listen to camera parameters
    device->start();
    colorParams = device->getColorCameraParams();
    irParams = device->getIrCameraParams();
    device->stop();

    //init registration
    registration = new libfreenect2::Registration(irParams, colorParams);
    initialized = true;
    return (true);
}
void
Kinect2::processData()
{
    //assume kinect2 is started and initialized
    if(started && initialized)
    {
        listener_color->waitForNewFrame(frames_c);
        colorFrame = frames_c[libfreenect2::Frame::Color];
        listener_depth->waitForNewFrame(frames_d);
        depthFrame = frames_d[libfreenect2::Frame::Depth];
        //  irFrame = frames_d[libfreenect2::Frame::Ir];
        registration->apply(colorFrame, depthFrame, undistorted, registered);
        listener_color->release(frames_c);
        listener_depth->release(frames_d);
    }
}

void
Kinect2::computePointCloud(PTC::Ptr& out_cloud)
{
    if(started && initialized)
    {
        //assume registration was performed
        out_cloud.reset(new PTC);
        out_cloud->width = 512;
        out_cloud->height = 424;
        out_cloud->is_dense = false;
        out_cloud->sensor_origin_.setZero();
        out_cloud->sensor_orientation_.setIdentity();
        out_cloud->points.resize(out_cloud->width * out_cloud->height);
        for (uint32_t r=0; r<out_cloud->height; ++r)
            for (uint32_t c=0; c<out_cloud->width; ++c)
                registration->getPointXYZRGB(undistorted, registered, r,c, out_cloud->points[512*r+c].x, out_cloud->points[512*r+c].y, out_cloud->points[512*r+c].z, out_cloud->points[512*r+c].rgb);
    }
}

////////////////////////////////////////////////////////////////////////////////
class SensorProcessor: public Module<SensorProcessor>
{
    friend class Module<SensorProcessor>;
    public:
        SensorProcessor()=delete;
        virtual ~SensorProcessor()=default;
        SensorProcessor(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor, const ros::Rate rate);
        struct Config
        {
            //Use the internal kinect2 processor, or a subscriber
            bool internal;
            //on which topic to listen if !internal
            std::string topic;
            //name of the internal processor
            std::string name;
        };
        typedef std::shared_ptr<SensorProcessor::Config> ConfigPtr;
        void update (const SensorProcessor::ConfigPtr conf);
        inline SensorProcessor::ConfigPtr getConfig() const
        {
            return config;
        }
    private:
        SensorProcessor::ConfigPtr config;
        //external subscriber to recieve a cloud
        ros::Subscriber sub_cloud;
        //associated callback
        void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        //internal kinect2 handler
        Kinect2 kinect2;
        tf::TransformBroadcaster kinect2_ref_brcaster;

        void spinOnce();
};

SensorProcessor::SensorProcessor(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor, const ros::Rate rate):
    Module<SensorProcessor>(n,ns,stor,rate)
{
    //default to asus xtion
    config.reset(new SensorProcessor::Config);
    config->internal = false;
    config->topic = nh.resolveName("/camera/depth_registered/points");
}

void SensorProcessor::update(const SensorProcessor::ConfigPtr conf)
{
    if (conf){
        config->internal = conf->internal;
        if (config->internal){
            //Use internal kinect2
            config->name = conf->name;
            //Use internal sensor processor, no subscriber needed
            sub_cloud.shutdown();
            if (!kinect2.isInitialized())
                kinect2.initDevice();
            if (!kinect2.isStarted())
                kinect2.start();
            return;
        }
        else if (!config->internal){
            //stop the internal kinect2
            if (kinect2.isStarted() || kinect2.isInitialized()){
                kinect2.stop();
                kinect2.close();
            }
            if (config->topic.compare(conf->topic) != 0){
                config->topic = conf->topic;
                //fire the subscriber
                sub_cloud = nh.subscribe(config->topic, 5, &SensorProcessor::cb_cloud, this);
            }
            return;
        }
    }
}

void SensorProcessor::spinOnce()
{
    queue_ptr->callAvailable(ros::WallDuration(0));
    if (config->internal){
        //get a cloud from  kinect2, we also need to publish a  tf of the sensor
        //and write also the ref frame inside the point cloud we send downstream
        kinect2.processData();
        PTC::Ptr scene;
        kinect2.computePointCloud(scene);
        scene->header.frame_id = config->name;
        pcl_conversions::toPCL(ros::Time::now(), scene->header.stamp);
        //Send it to good riddance downstream!
        storage->write_scene(scene);
        tf::Vector3 t(0.0385,0,0);
        tf::Quaternion q;
        q.setRPY(0,0,0);
        tf::Transform T(q,t);
        kinect2_ref_brcaster.sendTransform(tf::StampedTransform(T, ros::Time::now(),"kinect2_anchor", config->name.c_str()));
    }
    //nothing to do if external, callback takes care of it
}

void SensorProcessor::cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PTC::Ptr scene;
    pcl::fromROSMsg (*msg, *scene);
    // Save untouched scene into storage bye bye
    storage->write_scene(scene);
}
#endif
