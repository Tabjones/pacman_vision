#include <basic_node/kinect2_processor.h>

namespace pacv
{
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
Kinect2::start()
{
    device->start();
    started = true;
    return started;
}
bool
Kinect2::stop()
{
    device->stop();
    started = false;
    return started;
}
bool
Kinect2::close()
{
    device->close();
    initialized=false;
    return initialized;
}
bool
Kinect2::isStarted() const
{
    return started;
}
bool
Kinect2::isInitialized() const
{
    return initialized;
}
bool
Kinect2::initDevice()
{
    if (freenect2.enumerateDevices() <= 0){
        ROS_ERROR("[Kinect2::%s]\tNo Kinect2 devices found",__func__);
        return (false);
    }
    //no support for multiple kinects as of now... just get default one.
    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    //Use OpenCL Packet pipeline processor
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    if (!packetPipeline)
        packetPipeline = new libfreenect2::OpenCLPacketPipeline();
#else
    ROS_ERROR("[Kinect2::%s]\tLibFreenect2 does not have OpenCL support. Please recompile it with OpenCL support",__func__);
    return (false);
#endif

    //Open kinect2
    device = freenect2.openDevice(serial, packetPipeline);
    if (device == 0){
        ROS_ERROR("[Kinect2::%s]\tFailed to open Kinect2 with serial %s",__func__, serial.c_str());
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
}
