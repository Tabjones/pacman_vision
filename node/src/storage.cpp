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

void Storage::read_scene(PXC::Ptr &cloud)
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

void Storage::read_scene_processed(PXC::Ptr &cloud)
{
  LOCK guard(scenes);
  pcl::copyPointCloud(*scene_processed, *cloud);
  return;
}

void Storage::write_scene(PC::Ptr &cloud)
{
  LOCK guard(scenes);
  if (cloud)
    pcl::copyPointCloud(*cloud, *scene);
  return;
}

void Storage::write_scene_processed(PC::Ptr &cloud)
{
  LOCK guard(scenes);
  if (cloud)
    pcl::copyPointCloud(*cloud, *scene_processed);
  return;
}

void Storage::read_obj_clusters (boost::shared_ptr<std::vector<PXC> > &objs)
{
  LOCK guard(objects);
  if (!objs)
    objs.reset(new std::vector<PXC>);
  else
    objs->clear();
  if (this->clusters.empty())
  {
    ROS_WARN("[Storage][%s] Clusters from Storage are empty! Not reading anything", __func__);
    return;
  }
  for (std::vector<PXC>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    PXC temp;
    pcl::copyPointCloud(*it, temp);
    objs->push_back(temp);
  }
  return;
}
void write_obj_clusters (boost::shared_ptr<std::vector<PXC> > &objs)
{
  LOCK guard(objects);
  if (objs)
  {
    if (!objs->empty())
    {
      this->clusters.clear();
      for (std::vector<PXC>::const_iterator it = objs->begin(); it!= objs->end(); ++it)
      {
        PXC temp;
        pcl::copyPointCloud(*it, temp);
        clusters.push_back(temp);
      }
      return;
    }
  }
  ROS_WARN("[Storage][%s] Passed clusters are empty! Not writing anything", __func__);
  return;
}
void read_obj_transforms (boost::shared_ptr<std::vector<Eigen::Matrix4f> > &trans)
{
  LOCK guard(objects);
  if (!trans)
    trans.reset(new std::vector<Eigen::Matrix4f>);
  else
    trans->clear();
  if (this->estimations.empty())
  {
    ROS_WARN("[Storage][%s] Estimations from Storage are empty! Not reading anything", __func__);
    return;
  }
  for (std::vector<Eigen::Matrix4f>::const_iterator it = estimations.begin(); it != estimations.end(); ++it)
  {
    Eigen::Matrix4f temp(*it);
    trans->push_back(temp);
  }
  return;
}
void write_obj_transforms (boost::shared_ptr<std::vector<Eigen::Matrix4f> > &trans)
{
  LOCK guard(objects);
  if (trans)
  {
    if (!trans->empty())
    {
      this->estimations.clear();
      for (std::vector<Eigen::Matrix4f>::const_iterator it = trans->begin(); it!= trans->end(); ++it)
      {
        Eigen::Matrix4f temp(*it);
        this->estimations.push_back(temp);
      }
      return;
    }
  }
  ROS_WARN("[Storage][%s] Passed transformations are empty! Not writing anything", __func__);
  return;
}
void read_obj_names (boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > &n)
{
  LOCK guard(objects);
  if (!n)
    n.reset(new std::vector<std::pair<std::string, std::string> >);
  else
    n->clear();
  if (this->names.empty())
  {
    ROS_WARN("[Storage][%s] Names of objects from Storage are empty! Not reading anything", __func__);
    return;
  }
  for (std::vector<std::pair<std::string, std::string> >::const_iterator it = names.begin(); it != names.end(); ++it)
  {
    std::pair<std::string, std::string> temp(*it);
    n->push_back(temp);
  }
  return;
}
void write_obj_names (boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > &n)
{
  LOCK guard(objects);
  if (n)
  {
    if (!n->empty())
    {
      this->names.clear();
      for (std::vector<std::pair<std::string, std::string> >::const_iterator it = n->begin(); it!= n->end(); ++it)
      {
        std::pair<std::string, std::string> temp(*it);
        this->names.push_back(temp);
      }
      return;
    }
  }
  ROS_WARN("[Storage][%s] Passed names are empty! Not writing anything", __func__);
  return;
}
