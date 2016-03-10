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
#ifndef _MODELER_H_
#define _MODELER_H_

#include <common/dynamic_modules.hpp>
#include <common/common_pcl.h>
#include <common/common_ros.h>
#include <modeler/modeler_config.hpp>
#include <pacv_config.h>
//PCL
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/correspondence.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl_ros/transforms.h>
// #include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
// #include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
//general utilities
#include <ctime>
#include <deque>
#include <algorithm>
#include <boost/filesystem/path.hpp>
//ROS
#include <tf/transform_broadcaster.h>
//ROS generated
#include <pacman_vision_comm/start_modeler.h>
#include <pacman_vision_comm/stop_modeler_recording.h>

namespace pacv
{
class Modeler: public Module<Modeler>
{
    friend class Module<Modeler>;
    public:
        Modeler()=delete;
        Modeler(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor);
        typedef std::shared_ptr<Modeler> Ptr;
        ModelerConfig::Ptr getConfig() const
        {
            return config;
        }
        //update rosparams
        void updateRosparams();
        //Eigen alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        //Configuration
        ModelerConfig::Ptr config;
        //init with ros param
        void init();
        //deinit to free memory
        void deInit();
        //Service Server
        ros::ServiceServer srv_start;
        ros::ServiceServer srv_stop;
        //marker broadcaster
        ros::Publisher pub_markers;
        //publisher of model
        ros::Publisher pub_model;
        //subscriber to clickedpoints to create a model frame
        ros::Subscriber sub_clicked;
        //marker
        // std::shared_ptr<visualization_msgs::MarkerArray> marks;
        //start_callback
        bool cb_start(pacman_vision_comm::start_modeler::Request& req, pacman_vision_comm::start_modeler::Response& res);
        //stop_callback
        bool cb_stop(pacman_vision_comm::stop_modeler_recording::Request& req, pacman_vision_comm::stop_modeler_recording::Response& res);
        //spin once
        void spinOnce();
        // //create new markers from transforms
        // void create_markers();
        // //publish markers
        // void publish_markers();

        //processing queues
        std::deque<PTC> acquisition_q;
        //transform broadcaster for model
        tf::TransformBroadcaster tf_broadcaster;
        //model transforms
        Eigen::Matrix4f T_km, T_mk;
        //incremental frame transforms
        Eigen::Matrix4f T_frames;

        //model and downsampled model
        PTC::Ptr model_c, model_ds;
        //model color
        double mean_L, mean_a, mean_b;
        double model_mean_dE, model_stddev_dE;
        double stddev_mul;

        //PCL objects
        //registration stuff
        pcl::registration::TransformationEstimationDualQuaternion<PT,PT,float>::Ptr teDQ;
        // pcl::registration::CorrespondenceRejectorDistance cr;
        pcl::IterativeClosestPoint<PT,PT> icp;
        pcl::GeneralizedIterativeClosestPoint<PT,PT> gicp;
        //Octrees
        // pcl::octree::OctreePointCloudAdjacency<PT> oct_adj;
        // pcl::octree::OctreePointCloudChangeDetector<PT> oct_cd;

        //Threads stuff
        std::thread proc_t;
        std::mutex mtx_acq, mtx_model;

        //behaviour
        bool acquiring, processing;

        //init model color for filtering, computed out of first frame
        void computeColorDistribution(const PTC &frame);
        //tell if a point is inside the color distribution
        bool colorMetricInclusion(const PT &pt);
        //processing queue thread, consume acquisition_q and builds the model
        void processQueue();
        //check if two frames are too similar to each other, return true if similar
        bool checkFramesSimilarity(PTC::Ptr current, PTC::Ptr next, float factor=0.1);
        //align frames
        Eigen::Matrix4f alignFrames(PTC::Ptr target, PTC::Ptr source, PTC::Ptr &aligned, const float dist=0.1);
        //refines frames on model
        void refineFrames(PTC::Ptr frame, PTC::Ptr &refined, const float dist=0.01);
        //publish model as it is being created
        void publishModel();
};
}//namespace
#endif
