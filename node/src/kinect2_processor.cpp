#include "pacman_vision/kinect2_processor.h"

Kinect2Processor::Kinect2Processor () : device(0), packetPipeline(0), registration(0),
  listenerColor(0), listenerIrDepth(0)
{
}

bool
Kinect2Processor::initDevice ()
{
  if (freenect2.enumerateDevices() == 0)
  {
    ROS_ERROR("[Kinect2] No Kinect2 devices found");
    return (false);
  }
  //no support for multiple kinects as of now... just get default one.
  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  //Use OpenCL Packet pipeline processor
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
  if (!packetPipeline)
    packetPipeline = new libfreenect2::OpenCLPacketPipeline();
#else
  ROS_ERROR("[Kinect2] LibFreenect2 does not have OpenCL support. Please recompile it with OpenCL support");
  return (false);
#endif

  //Open kinect2
  device = freenect2.openDevice(serial, packetPipeline);
  if (device == 0)
  {
    ROS_ERROR("[Kinect2] Failed to open Kinect2 with serial %s", serial.c_str());
    return (false);
  }

  //create the listener
  listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  //and set them
  device->setColorFrameListener(listener);
  device->setIrAndDepthFrameListener(listener);
  //listen to camera parameters
  device->start();
  colorParams = device->getColorCameraParams();
  irParams = device->getIrCameraParams();
  device->stop();

  //init registration
  registration = new libfreenect2::Registration(irParams, colorParams);
  return (true);
}

void
Kinect2Processor::processData()
{
  //assume kinect2 is started and initialized
  listener->waitForNewFrame(frames);
  colorFrame = frames[libfreenect2::Frame::Color];
//  irFrame = frames[libfreenect2::Frame::Ir];
  depthFrame = frames[libfreenect2::Frame::Depth];
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
  registration->apply(colorFrame, depthFrame, &undistorted, &registered);
  listener->release(frames);
  //TODO add pointcloud converter
}
