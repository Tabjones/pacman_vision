#include "pacman_vision/storage.h"

///////////////////
//Storage Class//
///////////////////

//Constructor
Storage::Storage()
{
  this->scene.reset(new PC);
  this->scene_processed.reset(new PC);
}

void Storage::read_scene(PC::Ptr &cloud)
{
  LOCK guard(scenes);
  pcl::copyPointCloud(*scene, *cloud);
  return;
}

void Storage::read_scene_processed(PC::Ptr &cloud)
{
  LOCK guard(scenes);
  pcl::copyPointCloud(*scene_processed, *cloud);
  return;
}

void Storage::write_scene(PC::Ptr &cloud)
{
  LOCK guard(scenes);
  pcl::copyPointCloud(*cloud, *scene);
  return;
}

void Storage::write_scene_processed(PC::Ptr &cloud)
{
  LOCK guard(scenes);
  pcl::copyPointCloud(*cloud, *scene_processed);
  return;
}
