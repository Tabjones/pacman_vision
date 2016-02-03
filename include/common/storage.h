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
#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <pacv_config.h>
#include <common/common_std.h>
#include <common/common_pcl.h>
#include <common/box.h>

#include <Eigen/StdVector>
// #include <pcl/segmentation/supervoxel_clustering.h>

namespace pacv
{
class Storage
{
    public:
        typedef std::shared_ptr<Storage> Ptr;
        Storage();
        ~Storage(){}
        //Eigen Alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Read and write scene methods
        bool readScene (PTC::Ptr &cloud);
        bool writeScene (PTC::Ptr cloud);
        bool readSceneProcessed (PTC::Ptr &cloud);
        bool writeSceneProcessed (PTC::Ptr cloud);
        bool readScene (PXC::Ptr &cloud);
        bool readSceneProcessed (PXC::Ptr &cloud);
        //Read write sensor reference frame
        bool readSensorFrame (std::string& frame);
        bool writeSensorFrame (std::string frame);
        //Read and write estimated objects
        bool readObjTransforms (std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &trans);
        bool writeObjTransforms (std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> trans);
        bool readObjNames (std::shared_ptr<std::vector<std::pair<std::string, std::string>>> &n);
        bool writeObjNames (std::shared_ptr<std::vector<std::pair<std::string, std::string>>> n);
        //Search for a specific object name and return its index
        bool searchObjName (std::string n, int &idx);
        //Read and object transform by its index
        bool readObjTransformByIndex (int idx, std::shared_ptr<Eigen::Matrix4f> &trans);
        //Write and object transform to the specified index
        bool writeObjTransformByIndex (int idx, std::shared_ptr<Eigen::Matrix4f> trans);
        //Read and write arms/hands transforms
        bool readLeftArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        bool writeLeftArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> arm);
        bool readRightArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        bool writeRightArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> arm);
        bool readLeftHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &hand);
        bool writeLeftHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> hand);
        bool readRightHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &hand);
        bool writeRightHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> hand);
        //Read/write index of tracked object
        void readTrackedIndex(int &idx);
        void writeTrackedIndex(int idx);
    private:
        //untouched scene from kinect
        PTC::Ptr scene;
        std::mutex mtx_scene;
        //scene after processing
        PTC::Ptr scene_processed;
        std::mutex mtx_scene_processed;
        //sensor reference frame
        std::string sensor_ref_frame;
        std::mutex mtx_sensor_ref_frame;
        //Estimated transform from estimator
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> estimations;
        std::mutex mtx_estimations;
        //naming and id-ing of estimated objects from estimator
        std::vector<std::pair<std::string, std::string>> names; //name,ID
        std::mutex mtx_names;
        /*
         * Tracked object index (referred to vector of estimations),
         * set to -1 if not tracking
         */
        int index;
        std::mutex mtx_index;
        //Vito Left arm transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> left_arm;
        std::mutex mtx_left_arm;
        //Vito Right arm transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> right_arm;
        std::mutex mtx_right_arm;
        //Vito Left hand transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> left_hand;
        std::mutex mtx_left_hand;
        //Vito Right hand transform
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> right_hand;
        std::mutex mtx_right_hand;
};
}
#endif
