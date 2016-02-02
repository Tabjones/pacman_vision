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
#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

#include <common/dynamic_modules.hpp>
#include <common/common_pcl.h>
#include <common/common_ros.h>
#include <recognition/estimator_config.hpp>
//PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
// ROS generated headers
#include <pacman_vision_comm/estimate.h>
#include <pacman_vision_comm/pe.h>
#include <pacman_vision_comm/peArray.h>
//Boost
#include <boost/filesystem.hpp>
//ROS
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

namespace pel
{
    namespace interface
    {
        class PEProgressiveBisection;
    }
}

namespace pacv
{
class Estimator: public Module<Estimator>
{
    friend class Module<Estimator>;
    public:
        Estimator()=delete;
        Estimator(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor);
        typedef std::shared_ptr<Estimator> Ptr;
        EstimatorConfig::Ptr getConfig() const;
        ///update rosparams
        void updateRosparams();
        //Eigen alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        //Configuration
        EstimatorConfig::Ptr config;
        //init with ros param
        void init();
        //deinit to free memory
        void deInit();
        //Service Server
        ros::ServiceServer srv_estimate;
        //marker broadcaster
        ros::Publisher pub_markers;
        //tf
        tf::TransformBroadcaster tf_brc;
        //marker
        std::shared_ptr<visualization_msgs::MarkerArray> marks;
        //estimated transforms
        std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> estimations;
        //object clusters found on scene
        std::shared_ptr<std::vector<PXC>> clusters;
        //naming and id-ing of estimated objects
        std::shared_ptr<std::vector<std::pair<std::string,std::string>>> names; //name/id pairs
        //actual scene
        PXC::Ptr scene;
        //path to pel database
        boost::filesystem::path db_path;
        //index of tracked object, if the tracker is running and tracking this is != -1
        int tracked_idx;
        int transition;

        //PEL object
        std::shared_ptr<pel::interface::PEProgressiveBisection> pe;
        //its parameters
        int iter, neigh;
        bool all_success;
        double rmse_thresh;

         // method to extract  clusters of objects in a table  top scenario with
         // table already removed
        int extract_clusters();
        //perform estimation
        bool estimate();
        //estimate service callback
        bool cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res);
        //spin once
        void spinOnce();
        //create new markers from estimation
        void create_markers();
        //publish markers
        void publish_markers();
        //publish tf transforms
        void publish_tf();
};
}//namespace
#endif
