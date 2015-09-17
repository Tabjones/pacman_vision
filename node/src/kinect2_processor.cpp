#include "pacman_vision/kinect2_processor.h"

Kinect2Processor::Kinect2Processor () : device(0), packetPipeline(0), registration(0),
  listener(0), started(false), initialized(false)
{
  undistorted = new libfreenect2::Frame(512, 424, 4);
  registered = new libfreenect2::Frame(512, 424, 4);
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
  initialized = true;
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
  registration->apply(colorFrame, depthFrame, undistorted, registered);
  listener->release(frames);
}

void
Kinect2Processor::computePointCloud(PC::Ptr& out_cloud)
{
  //assume registration was performed
  out_cloud.reset(new PC);
  out_cloud->width = 512;
  out_cloud->height = 424;
  out_cloud->is_dense = true;
  out_cloud->sensor_origin_.setZero();
  out_cloud->sensor_orientation_.setIdentity();
  float cx(irParams.cx), cy(irParams.cy);
  float fx(irParams.fx), fy(irParams.fy);
  for (int xi=0; xi<512; ++xi)
  {
    for (int yi=0; yi<424; ++yi)
    {
      float xu = (xi + 0.5 - cx)/fx;
      float yu = (yi + 0.5 - cy)/fy;
      PT point;
      point.x = xu*undistorted->data[512*yi + xi];
      point.y = yu*undistorted->data[512*yi + xi];
      point.z = undistorted->data[512*yi + xi];
      point.rgb = registered->data[512*yi + xi];
      out_cloud->push_back(point);
    }
  }
}
