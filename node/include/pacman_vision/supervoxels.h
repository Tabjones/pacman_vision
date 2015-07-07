#ifndef _INCL_SUPERVOXELS
#define _INCL_SUPERVOXELS
// ROS headers
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//pcl
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>
//Ros generated
#include "pacman_vision_comm/clusterize.h"
//Utility
#include "pacman_vision/utility.h"
//Storage
#include "pacman_vision/storage.h"

class VisionNode;

class Supervoxels
{

  friend class VisionNode;

  public:
    Supervoxels(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor);
    ~Supervoxels();
  private:
    ros::NodeHandle nh;
    boost::shared_ptr<ros::CallbackQueue> queue_ptr;
    boost::shared_ptr<Storage> storage;

    PC::Ptr scene;
    PC::Ptr clustered_scene;
    //pcl::SupervoxelClustering<PT> svc;
    //pcl::NormalEstimationOMP<PT, PN> ne;

    //class behaviour
    bool serviced;
    double voxel_res, seed_res, color_imp, normal_imp, spatial_imp, normal_radius;
    int num_iterations;

    boost::shared_ptr<std::map<uint32_t, pcl::Supervoxel<PT>::Ptr > > clusters;

    //Service Server
    ros::ServiceServer srv_clusterize;

    //Publisher for clusterized scene
    ros::Publisher pub_clusterized_scene;

    //custom spinner
    void spin_once();

    //method to perform clustering
    bool clustering();

    //clusterize service callback
    bool cb_clusterize(pacman_vision_comm::clusterize::Request& req, pacman_vision_comm::clusterize::Response& res);


};
#endif
