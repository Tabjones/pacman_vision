#include "pacman_vision/supervoxels.h"

/////////////////
// Supervoxels //
/////////////////
Supervoxels::Supervoxels(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor)
{
  this->nh = ros::NodeHandle (n, "supervoxels");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->storage = stor;
  this->scene.reset(new PC);
  this->clustered_scene.reset(new PC);
  this->clusters.reset(new std::map<uint32_t, pcl::Supervoxel<PT>::Ptr>);
  this->pub_clusterized_scene = this->nh.advertise<PC> ("supervoxelled_scene", 2);
  srv_clusterize = nh.advertiseService("clusterize_scene", &Supervoxels::cb_clusterize, this);
  //Params
  nh.param<bool>("/pacman_vision/use_service", serviced, false);
  nh.param<double>("/pacman_vision/voxel_resolution", voxel_res, 0.02);
  nh.param<double>("/pacman_vision/seed_resolution", seed_res, 0.2);
  nh.param<double>("/pacman_vision/color_importance",color_imp, 0.3333);
  nh.param<double>("/pacman_vision/normal_importance", normal_imp, 0.3333);
  nh.param<double>("/pacman_vision/spatial_importance", spatial_imp ,0.3333);
  nh.param<double>("/pacman_vision/normals_search_radius", normal_radius ,0.015);
  nh.param<int>("/pacman_vision/refinement_iterations", num_iterations ,2);
}
Supervoxels::~Supervoxels()
{
  this->nh.shutdown();
}

void Supervoxels::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}

bool Supervoxels::clustering()
{
  if (!this->storage->read_scene_processed(scene))
    return false;
  pcl::NormalEstimationOMP<PT, NT> ne;
  ne.setInputCloud(scene);
  ne.useSensorOriginAsViewPoint();
  ne.setRadiusSearch(normal_radius);
  NC::Ptr normals (new NC);
  ne.compute(*normals);

  pcl::SupervoxelClustering<PT> svc(voxel_res, seed_res, true);
  svc.setInputCloud(scene);
  svc.setNormalCloud(normals);
  svc.setColorImportance(color_imp);
  svc.setSpatialImportance(spatial_imp);
  svc.setNormalImportance(normal_imp);
  //get clusters
  svc.extract(*clusters);
  svc.refineSupervoxels(num_iterations, *clusters);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  output_cloud = svc.getColoredCloud();
  //This outputs cloud with the voxelgrid of the method (voxel_res), so its further downsampled.
  //output_cloud = svc.getColoredVoxelCloud();
  pcl::copyPointCloud(*output_cloud, *clustered_scene);
  pub_clusterized_scene.publish(clustered_scene);
  this->storage->write_supervoxels_clusters(clusters);
  return true;
}

bool Supervoxels::cb_clusterize(pacman_vision_comm::clusterize::Request& req, pacman_vision_comm::clusterize::Response& res)
{
  if ( clustering() )
    return true;
  else
    return false;
}

