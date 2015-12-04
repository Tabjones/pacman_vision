#ifndef _ESTIMATOR_HPP_
#define _ESTIMATOR_HPP_

#include <pacman_vision/config.h>
//Utility
#include <pacman_vision/common.h>
#include <pacman_vision/base_ros_node_helpers.hpp>
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

using namespace pcl;

class Estimator : public Module<Estimator>
{
    friend class Module<Estimator>;
    public:
    Estimator()=delete;
    Estimator(const ros::NodeHandle n, const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate);
    virtual ~Estimator()=default;
    //Eigen alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
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

        /*
         * method to extract clusters of objects in a table top scenario with
         * table already removed
         */
        int
        extract_clusters();
        //perform estimation
        bool
        estimate();
        //estimate service callback
        bool
        cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res);
};

//Implementations

//Constructor
Estimator::Estimator(const ros::NodeHandle n, const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate)
    :Module<Estimator>(n,ns,stor,rate)
{
    scene.reset(new PXC);
    db_path = (ros::package::getPath("pacman_vision") + "/database" );
    if (!boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path))
        ROS_WARN("[Estimator][%s] Database for pose estimation does not exists!! Plese put one in /database folder, before trying to perform a pose estimation.",__func__);
    srv_estimate = nh.advertiseService("estimate", &Estimator::cb_estimate, this);
    //init params
    nh.param<bool>("/pacman_vision/object_calibration", calibration, false);
    disabled = false;
    nh.param<int>("/pacman_vison/iterations", iterations, 5);
    nh.param<int>("/pacman_vision/neighbors", neighbors, 20);
    nh.param<double>("/pacman_vision/cluster_tol", clus_tol, 0.05);
    pe.setParam("verbosity",2);
    pe.setRMSEThreshold(0.003);
    pe.setStepIterations(iterations);
    pe.setParam("lists_size",neighbors);
    pe.setParam("downsamp",0);
    pe.loadAndSetDatabase(this->db_path);
    ROS_INFO("[Estimator] Estimator module extract euclidean clusters from current scene and tries to identify each of them by matching with provided database. For the Estimator to work properly please enable at least plane segmentation during scene processing.");
}

int
Estimator::extract_clusters()
{
    storage->read_scene_processed(scene);
    if (scene->empty()){
        ROS_WARN("[Estimator][%s] Processed scene is empty, cannot continue...",__func__);
        return -1;
    }
    ROS_INFO("[Estimator][%s] Extracting object clusters with cluster tolerance of %g",__func__, clus_tol);
    //objects
    pcl::ExtractIndices<PX> extract;
    pcl::EuclideanClusterExtraction<PX> ec;
    pcl::search::KdTree<PX>::Ptr tree (new pcl::search::KdTree<PX>);
    std::vector<pcl::PointIndices> cluster_indices;
    //cluster extraction
    tree->setInputCloud(scene);
    ec.setInputCloud(scene);
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(clus_tol);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(scene->points.size());
    ec.extract(cluster_indices);
    int size = (int)cluster_indices.size();
    clusters.reset (new std::vector<PXC> );
    clusters->resize(size);
    names.reset(new std::vector<std::pair<std::string, std::string>>);
    names->resize(size);
    estimations.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);
    estimations->resize(size);
    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
    {
        PXC::Ptr object (new PXC);
        extract.setInputCloud(scene);
        extract.setIndices(boost::make_shared<PointIndices>(*it));
        extract.setNegative(false);
        extract.filter(clusters->at(j));
    }
    ROS_INFO("[Estimator][%s] Found %d clusters of possible objects.",__func__,size);
    return size;
}

bool
Estimator::cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res)
{
    if (this->disabled){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Estimator][%s] Estimator module is temporary disabled, exiting...",__func__);
        return false;
    }
    if (this->estimate()){
        geometry_msgs::Pose pose;
        tf::Transform trans;
        for (int i=0; i<estimations->size(); ++i)
        {
            fromEigen(estimations->at(i), pose, trans);
            pacman_vision_comm::pe pose_est;
            pose_est.pose = pose;
            pose_est.name = names->at(i).first;
            pose_est.id = names->at(i).second;
            pose_est.parent_frame = "/kinect2_rgb_optical_frame";
            res.estimated.poses.push_back(pose_est);
        }
        ROS_INFO("[Estimator][%s] Pose Estimation complete!", __func__);
        return true;
    }
    else{
        ROS_WARN("[Estimator][%s] Pose Estimation failed!", __func__);
        return false;
    }
}

bool
Estimator::estimate()
{
    int size = this->extract_clusters();
    if (size < 1){
        ROS_ERROR("[Estimator][%s] No object clusters found in scene, aborting pose estimation...",__func__);
        return false;
    }
    for (int i=0; i<size; ++i)
    {
        pe.setTarget(clusters->at(i).makeShared(), "object");
        pel::Candidate pest;
        pe.estimate(pest);
        std::string name = pest.getName();
        std::vector<std::string> vst;
        boost::split (vst, name, boost::is_any_of("_"), boost::token_compress_on);
        if (this->calibration)
            names->at(i).first = "object";
        else
            names->at(i).first = vst.at(0);
        estimations->at(i) = pest.getTransformation();
        names->at(i).second = vst.at(0);
        ROS_INFO("[Estimator][%s] Found %s.",__func__,name.c_str());
    }
    //first check if we have more copy of the same object in names
    for (int i=0; i<names->size(); ++i)
    {
        int count(1);
        std::string name_original = names->at(i).first;
        if (i>0){
            for (int j=0; j<i; ++j)
            {
                if (names->at(i).first.compare(names->at(j).first) == 0){
                    //i-th name is equal to j-th name
                    names->at(i).first = name_original + "_" + std::to_string(++count);
                }
            }
        }
    }
    //Save estimations in Storage
    this->storage->write_obj_clusters(this->clusters);
    this->storage->write_obj_names(this->names);
    this->storage->write_obj_transforms(this->estimations);
    return true;
}
#endif
