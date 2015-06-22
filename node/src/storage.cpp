#include "pacman_vision/storage.h"

///////////////////
//Storage Class//
///////////////////

//Constructor
Storage::Storage()
{
  this->scene.reset(new pcl::PointCloud<pcl::PointXYZ>);
  this->scene_passthrough.reset(new pcl::PointCloud<pcl::PointXYZ>);
  this->scene_downsample.reset(new pcl::PointCloud<pcl::PointXYZ>);
  this->scene_plane.reset(new pcl::PointCloud<pcl::PointXYZ>);
  this->scene_processed.reset(new pcl::PointCloud<pcl::PointXYZ>);
}
