#ifndef _INCL_KINECT2
#define _INCL_KINECT2

// libfreenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
//opencv2
#include <opencv2/opencv.hpp>

#include <ros/console.h>
#include <string>

class Kinect2Processor
{
  public:
    Kinect2Processor ();
    ~Kinect2Processor ()
    {
      delete packetPipeline;
      delete device;
      delete listener;
      delete registration;
      delete colorFrame;
      delete irFrame;
      delete depthFrame;
    }
    //init Kinect2 Device
    bool
    initDevice ();
    //Start Kinect2
    void
    start ()
    {
      device->start();
    }
    //Stop Kinect2
    void
    stop ()
    {
      device->stop();
    }
    //Recive and process data
    void
    processData();
  private:
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *device;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::PacketPipeline *packetPipeline;
    libfreenect2::Registration *registration;
    libfreenect2::Freenect2Device::ColorCameraParams colorParams;
    libfreenect2::Freenect2Device::IrCameraParams irParams;
    libfreenect2::Frame *colorFrame, *irFrame, *depthFrame;
    libfreenect2::FrameMap frames;
};

#endif
