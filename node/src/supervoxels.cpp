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
  this->pub_clusterized_scene = this->nh.advertise<PC> ("supervoxelled_scene", 2);
  serviced = true;
  voxel_res = 0.02f;
  seed_res = 0.2f;
  color_imp = normal_imp = spatial_imp = 0.3333f;
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
  //TODO
}

bool Supervoxels::cb_clusterize(pacman_vision_comm::clusterize::Request& req, pacman_vision_comm::clusterize::Response& res)
{
  //TODO
}

