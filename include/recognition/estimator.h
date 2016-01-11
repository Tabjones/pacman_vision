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
        //publish markers
        void publish_markers();
        //create new markers from estimation
        void create_markers();
};
}//namespace
#endif
