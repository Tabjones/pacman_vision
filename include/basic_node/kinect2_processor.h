#ifndef _KINECT2_PROCESSOR_H_
#define _KINECT2_PROCESSOR_H_

//libfreenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <common/common_ros.h>
#include <common/common_pcl.h>

namespace pacv
{
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
}
#endif
