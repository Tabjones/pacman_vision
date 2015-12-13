#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

#include <pacman_vision/config.h>
//Utility
#include <pacman_vision/common.h>
#include <pacman_vision/dynamic_modules.hpp>
#include <pacman_vision/module_config.h>
//PCL
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS generated headers
#include <pacman_vision_comm/estimate.h>
#include <pacman_vision_comm/pe.h>
#include <pacman_vision_comm/peArray.h>
//Storage
#include <pacman_vision/storage.h>
//PEL
#include <pel/pe_progressive_bisection.h>

class Estimator : public Module<Estimator>
{
    friend class Module<Estimator>;
    public:
    Estimator()=delete;
    Estimator(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor, const ros::Rate rate);
    virtual ~Estimator()=default;
    typedef std::shared_ptr<EstimatorConfig> ConfigPtr;
    typedef std::shared_ptr<Estimator> Ptr;
    void updateIfNeeded(const Estimator::ConfigPtr conf);
    inline Estimator::ConfigPtr getConfig() const;
    //Eigen alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        //Configuration
        Estimator::ConfigPtr config;
        //Service Server
        ros::ServiceServer srv_estimate;
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

        //class behaviour
        bool calibration, disabled;
        int iterations, neighbors;
        double clus_tol;

        //PEL object
        pel::interface::PEProgressiveBisection pe;

         // method to extract  clusters of objects in a table  top scenario with
         // table already removed
        int
        extract_clusters();
        //perform estimation
        bool
        estimate();
        //estimate service callback
        bool
        cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res);
        //spin redefinition
        void spin();
};
#endif
