// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
