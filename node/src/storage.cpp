#include "pacman_vision/storage.h"

///////////////////
//Storage Class//
///////////////////

//Constructor
Storage::Storage()
{
  this->tracked_id = "NOT TRACKING";
  this->tracked_name = "NOT TRACKING";
}

void Storage::read_scene(PC::Ptr &cloud)
{
  if (scene)
  {
    LOCK guard(mtx_scene);
    pcl::copyPointCloud(*scene, *cloud);
    return;
  }
  ROS_WARN("[Storage][%s] Scene from Storage is empty! Not reading anything", __func__);
  return;
}

void Storage::read_scene(PXC::Ptr &cloud)
{
  if (scene)
  {
    LOCK guard(mtx_scene);
    pcl::copyPointCloud(*scene, *cloud);
    return;
  }
  ROS_WARN("[Storage][%s] Scene from Storage is empty! Not reading anything", __func__);
  return;
}

void Storage::read_scene_processed(PC::Ptr &cloud)
{
  if (scene_processed)
  {
    LOCK guard(mtx_scene_processed);
    pcl::copyPointCloud(*scene_processed, *cloud);
    return;
  }
  ROS_WARN("[Storage][%s] Scene Processed from Storage is empty! Not reading anything", __func__);
  return;
}

void Storage::read_scene_processed(PXC::Ptr &cloud)
{
  if(scene_processed)
  {
    LOCK guard(mtx_scene_processed);
    pcl::copyPointCloud(*scene_processed, *cloud);
    return;
  }
  ROS_WARN("[Storage][%s] Scene Processed from Storage is empty! Not reading anything", __func__);
  return;
}

void Storage::write_scene(PC::Ptr &cloud)
{
  if (cloud)
  {
    LOCK guard(mtx_scene);
    if (!scene)
      scene.reset(new PC);
    pcl::copyPointCloud(*cloud, *scene);
    return;
  }
  ROS_WARN("[Storage][%s] Passed cloud is empty! Not writing anything in Storage", __func__);
  return;
}

void Storage::write_scene_processed(PC::Ptr &cloud)
{
  if (cloud)
  {
    LOCK guard(mtx_scene_processed);
    if (!scene_processed)
      scene_processed.reset(new PC);
    pcl::copyPointCloud(*cloud, *scene_processed);
    return;
  }
  ROS_WARN("[Storage][%s] Passed cloud is empty! Not writing anything in Storage", __func__);
  return;
}

void Storage::read_obj_clusters (boost::shared_ptr<std::vector<PXC> > &objs)
{
  LOCK guard(mtx_clusters);
  if (!objs)
    objs.reset(new std::vector<PXC>);
  else
    objs->clear();
  if (this->clusters.empty())
  {
    ROS_WARN("[Storage][%s] Clusters from Storage are empty! Not reading anything", __func__);
    return;
  }
  boost::copy(clusters, back_inserter(*objs));
  return;
}
void Storage::write_obj_clusters (boost::shared_ptr<std::vector<PXC> > &objs)
{
  if (objs)
  {
    if (!objs->empty())
    {
      LOCK guard(mtx_clusters);
      this->clusters.clear();
      boost::copy(*objs, back_inserter(clusters));
      return;
    }
  }
  ROS_WARN("[Storage][%s] Passed clusters are empty! Not writing anything", __func__);
  return;
}
void Storage::read_obj_transforms (boost::shared_ptr<std::vector<Eigen::Matrix4f> > &trans)
{
  LOCK guard(mtx_estimations);
  if (!trans)
    trans.reset(new std::vector<Eigen::Matrix4f>);
  else
    trans->clear();
  if (this->estimations.empty())
  {
    ROS_WARN("[Storage][%s] Estimations from Storage are empty! Not reading anything", __func__);
    return;
  }
  boost::copy(estimations, back_inserter(*trans));
  return;
}
void Storage::write_obj_transforms (boost::shared_ptr<std::vector<Eigen::Matrix4f> > &trans)
{
  if (trans)
  {
    if (!trans->empty())
    {
      LOCK guard(mtx_estimations);
      this->estimations.clear();
      boost::copy(*trans, back_inserter(estimations));
      return;
    }
  }
  ROS_WARN("[Storage][%s] Passed transformations are empty! Not writing anything", __func__);
  return;
}
void Storage::read_obj_names (boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > &n)
{
  LOCK guard(mtx_names);
  if (!n)
    n.reset(new std::vector<std::pair<std::string, std::string> >);
  else
    n->clear();
  if (this->names.empty())
  {
    ROS_WARN("[Storage][%s] Names of objects from Storage are empty! Not reading anything", __func__);
    return;
  }
  boost::copy(names, back_inserter(*n));
  return;
}
void Storage::write_obj_names (boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > &n)
{
  if (n)
  {
    if (!n->empty())
    {
      LOCK guard(mtx_names);
      this->names.clear();
      boost::copy(*n, back_inserter(names));
      return;
    }
  }
  ROS_WARN("[Storage][%s] Passed names are empty! Not writing anything", __func__);
  return;
}

void Storage::search_obj_name(std::string n, int &idx)
{
  LOCK guard(mtx_names);
  idx = -1;
  for (int i=0; i<names.size(); ++i)
  {
    if (names[i].first.compare(n) == 0)
    {
      idx = i;
      return;
    }
  }
  return;
}

bool Storage::read_obj_transform_by_index(int idx, boost::shared_ptr<Eigen::Matrix4f> &trans)
{
  if (!trans)
    trans.reset(new Eigen::Matrix4Xf);
  LOCK guard(mtx_estimations);
  for (int i=0; i<estimations.size(); ++i)
  {
    if ( i == idx)
    {
      *trans = estimations[i];
      return true;
    }
  }
  return false;
}
void Storage::read_tracked_transform(boost::shared_ptr<Eigen::Matrix4f> &transf)
{
  if (!transf)
    transf.reset(new Eigen::Matrix4f);
  LOCK guard(mtx_tracked_transform);
  *transf = tracked_transform;
  return;
}

void Storage::write_tracked_transform(boost::shared_ptr<Eigen::Matrix4f> &transf)
{
  if (!transf)
  {
    ROS_WARN("[Storage][%s] Passed transform is empty, not writing anything...", __func__);
    return;
  }
  LOCK guard(mtx_tracked_transform);
  this->tracked_transform = *transf;
  return;
}

void Storage::read_tracked_name(std::string &n)
{
  LOCK guard(mtx_tracked_name);
  n = this->tracked_name;
  return;
}

void Storage::write_tracked_name(std::string &n)
{
  LOCK guard(mtx_tracked_name);
  this->tracked_name = n;
  return;
}
void Storage::read_tracked_id(std::string &id)
{
  LOCK guard(mtx_tracked_id);
  id = this->tracked_id;
  return;
}
void Storage::write_tracked_id(std::string &id)
{
  LOCK guard(mtx_tracked_id);
  this->tracked_id = id;
  return;
}

void Storage::read_left_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm)
{
}
    void write_left_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm);
    void read_right_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm);
    void write_right_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm);
    void read_left_hand(boost::shared_ptr<Eigen::Matrix4f> &hand);
    void write_left_hand(boost::shared_ptr<Eigen::Matrix4f> &hand);
    void read_right_hand(boost::shared_ptr<Eigen::Matrix4f> &hand);
    void write_right_hand(boost::shared_ptr<Eigen::Matrix4f> &hand);
