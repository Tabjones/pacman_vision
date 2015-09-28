#ifndef _INCL_KINECT2
#define _INCL_KINECT2

#include <pacman_vision/config.h>
//utility
#include <pacman_vision/utility.h>
// libfreenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

class Kinect2Processor
{
  public:
    Kinect2Processor ();
    ~Kinect2Processor ()
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
    bool started, initialized;
    //init Kinect2 Device
    bool
    initDevice ();
    //Start Kinect2
    void
    start ()
    {
      device->start();
      started = true;
    }
    //Stop Kinect2
    void
    stop ()
    {
      device->stop();
      started = false;
    }
    //close kinect2
    void
    close ()
    {
      device->close();
      initialized=false;
    }
    //Recive and process data
    void
    processData();
    //Compute a PointCloud from processed data
    void
    computePointCloud(PC::Ptr& out_cloud);
  private:
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *device;
    libfreenect2::SyncMultiFrameListener *listener_color;
    libfreenect2::SyncMultiFrameListener *listener_depth;
    libfreenect2::PacketPipeline *packetPipeline;
    libfreenect2::Registration *registration;
    libfreenect2::Freenect2Device::ColorCameraParams colorParams;
    libfreenect2::Freenect2Device::IrCameraParams irParams;
    libfreenect2::Frame *colorFrame, *irFrame, *depthFrame;
    libfreenect2::Frame *undistorted, *registered;
    libfreenect2::FrameMap frames_c, frames_d;
};

#endif
