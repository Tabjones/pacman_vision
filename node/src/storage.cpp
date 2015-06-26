#include "pacman_vision/storage.h"

///////////////////
//Storage Class//
///////////////////

//Constructor
Storage::Storage()
{
  this->index = -1;
}

bool Storage::read_scene(PC::Ptr &cloud)
{
  if (scene)
  {
    LOCK guard(mtx_scene);
    pcl::copyPointCloud(*scene, *cloud);
    return true;
  }
  ROS_WARN("[Storage][%s] Scene from Storage is empty! Not reading anything", __func__);
  return false;
}

bool Storage::read_scene(PXC::Ptr &cloud)
{
  if (scene)
  {
    LOCK guard(mtx_scene);
    pcl::copyPointCloud(*scene, *cloud);
    return true;
  }
  ROS_WARN("[Storage][%s] Scene from Storage is empty! Not reading anything", __func__);
  return false;
}

bool Storage::read_scene_processed(PC::Ptr &cloud)
{
  if (scene_processed)
  {
    LOCK guard(mtx_scene_processed);
    pcl::copyPointCloud(*scene_processed, *cloud);
    return true;
  }
  ROS_WARN("[Storage][%s] Scene Processed from Storage is empty! Not reading anything", __func__);
  return false;
}

bool Storage::read_scene_processed(PXC::Ptr &cloud)
{
  if(scene_processed)
  {
    LOCK guard(mtx_scene_processed);
    pcl::copyPointCloud(*scene_processed, *cloud);
    return true;
  }
  ROS_WARN("[Storage][%s] Scene Processed from Storage is empty! Not reading anything", __func__);
  return false;
}

bool Storage::write_scene(PC::Ptr &cloud)
{
  if (cloud)
  {
    LOCK guard(mtx_scene);
    if (!scene)
      scene.reset(new PC);
    pcl::copyPointCloud(*cloud, *scene);
    return true;
  }
  ROS_WARN("[Storage][%s] Passed cloud is empty! Not writing anything in Storage", __func__);
  return false;
}

bool Storage::write_scene_processed(PC::Ptr &cloud)
{
  if (cloud)
  {
    LOCK guard(mtx_scene_processed);
    if (!scene_processed)
      scene_processed.reset(new PC);
    pcl::copyPointCloud(*cloud, *scene_processed);
    return true;
  }
  ROS_WARN("[Storage][%s] Passed cloud is empty! Not writing anything in Storage", __func__);
  return false;
}

bool Storage::read_obj_clusters (boost::shared_ptr<std::vector<PXC> > &objs)
{
  LOCK guard(mtx_clusters);
  if (!objs)
    objs.reset(new std::vector<PXC>);
  else
    objs->clear();
  if (this->clusters.empty())
  {
    ROS_WARN("[Storage][%s] Clusters from Storage are empty! Not reading anything", __func__);
    return false;
  }
  boost::copy(clusters, back_inserter(*objs));
  return true;
}
bool Storage::write_obj_clusters (boost::shared_ptr<std::vector<PXC> > &objs)
{
  if (objs)
  {
    if (!objs->empty())
    {
      LOCK guard(mtx_clusters);
      this->clusters.clear();
      boost::copy(*objs, back_inserter(clusters));
      return true;
    }
  }
  ROS_WARN("[Storage][%s] Passed clusters are empty! Not writing anything", __func__);
  return false;
}
bool Storage::read_obj_transforms (boost::shared_ptr<std::vector<Eigen::Matrix4f> > &trans)
{
  LOCK guard(mtx_estimations);
  if (!trans)
    trans.reset(new std::vector<Eigen::Matrix4f>);
  else
    trans->clear();
  if (this->estimations.empty())
  {
    ROS_WARN("[Storage][%s] Estimations from Storage are empty! Not reading anything", __func__);
    return false;
  }
  boost::copy(estimations, back_inserter(*trans));
  return true;
}
bool Storage::write_obj_transforms (boost::shared_ptr<std::vector<Eigen::Matrix4f> > &trans)
{
  if (trans)
  {
    if (!trans->empty())
    {
      LOCK guard(mtx_estimations);
      this->estimations.clear();
      boost::copy(*trans, back_inserter(estimations));
      return true;
    }
  }
  ROS_WARN("[Storage][%s] Passed transformations are empty! Not writing anything", __func__);
  return false;
}
bool Storage::read_obj_names (boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > &n)
{
  LOCK guard(mtx_names);
  if (!n)
    n.reset(new std::vector<std::pair<std::string, std::string> >);
  else
    n->clear();
  if (this->names.empty())
  {
    ROS_WARN("[Storage][%s] Names of objects from Storage are empty! Not reading anything", __func__);
    return false;
  }
  boost::copy(names, back_inserter(*n));
  return true;
}
bool Storage::write_obj_names (boost::shared_ptr<std::vector<std::pair<std::string, std::string> > > &n)
{
  if (n)
  {
    if (!n->empty())
    {
      LOCK guard(mtx_names);
      this->names.clear();
      boost::copy(*n, back_inserter(names));
      return true;
    }
  }
  ROS_WARN("[Storage][%s] Passed names are empty! Not writing anything", __func__);
  return false;
}

bool Storage::search_obj_name(std::string n, int &idx)
{
  LOCK guard(mtx_names);
  idx = -1;
  for (int i=0; i<names.size(); ++i)
  {
    if (names[i].first.compare(n) == 0)
    {
      idx = i;
      return true;
    }
  }
  return false;
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
bool Storage::write_obj_transform_by_index(int idx, boost::shared_ptr<Eigen::Matrix4f> &trans)
{
  if (!trans)
  {
    ROS_WARN("[Storage][%s] Passed transform is empty! Not writing anything", __func__);
    return false;
  }
  LOCK guard(mtx_estimations);
  if (idx >=0 && idx < (estimations.size()-1))
  {
    estimations.at(idx) = *trans;
    return true;
  }
  ROS_WARN("[Storage][%s] Passed index is invalid, or estimations are not available", __func__);
  return false;
}

bool Storage::read_left_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm)
{
  LOCK guard(mtx_left_arm);
  if (!arm)
    arm.reset(new std::vector<Eigen::Matrix4f>);
  else
    arm->clear();
  if (this->left_arm.empty())
  {
    ROS_WARN("[Storage][%s] Vito Left Arm transforms from Storage are empty! Not reading anything", __func__);
    return false;
  }
  boost::copy(left_arm, back_inserter(*arm));
  return true;
}
bool Storage::write_left_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm)
{
  if (arm)
  {
    if (!arm->empty())
    {
      LOCK guard(mtx_left_arm);
      left_arm.clear();
      boost::copy(*arm, back_inserter(left_arm));
      return true;
    }
  }
  ROS_WARN("[Storage][%s] Passed Arm transforms are empty! Not writing anything in Storage", __func__);
  return false;
}
bool Storage::read_right_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm)
{
  LOCK guard(mtx_right_arm);
  if (!arm)
    arm.reset(new std::vector<Eigen::Matrix4f>);
  else
    arm->clear();
  if (this->right_arm.empty())
  {
    ROS_WARN("[Storage][%s] Vito Right Arm transforms from Storage are empty! Not reading anything", __func__);
    return false;
  }
  boost::copy(right_arm, back_inserter(*arm));
  return true;
}
bool Storage::write_right_arm(boost::shared_ptr<std::vector<Eigen::Matrix4f> > &arm)
{
  if (arm)
  {
    if (!arm->empty())
    {
      LOCK guard(mtx_right_arm);
      right_arm.clear();
      boost::copy(*arm, back_inserter(right_arm));
      return true;
    }
  }
  ROS_WARN("[Storage][%s] Passed Arm transforms are empty! Not writing anything in Storage", __func__);
  return false;
}
void Storage::read_left_hand(boost::shared_ptr<Eigen::Matrix4f> &hand)
{
  LOCK guard(mtx_left_hand);
  if (!hand)
   hand.reset(new Eigen::Matrix4f);
  *hand = this->left_hand;
  return;
}

bool Storage::write_left_hand(boost::shared_ptr<Eigen::Matrix4f> &hand)
{
  if (hand)
  {
    LOCK guard(mtx_left_hand);
    this->left_hand = *hand;
    return true;
  }
  ROS_WARN("[Storage][%s] Passed Hand transformation is empty! Not writing anything in Storage", __func__);
  return false;
}
void Storage::read_right_hand(boost::shared_ptr<Eigen::Matrix4f> &hand)
{
  LOCK guard(mtx_right_hand);
  if (!hand)
    hand.reset(new Eigen::Matrix4f);
  *hand = this->right_hand;
  return;
}
bool Storage::write_right_hand(boost::shared_ptr<Eigen::Matrix4f> &hand)
{
  if (hand)
  {
    LOCK guard(mtx_right_hand);
    this->right_hand = *hand;
    return true;
  }
  ROS_WARN("[Storage][%s] Passed Hand transformation is empty! Not writing anything in Storage", __func__);
  return false;
}
void Storage::read_table(boost::shared_ptr<Eigen::Matrix4f> &t)
{
  LOCK guard(mtx_table);
  if (!t)
    t.reset(new Eigen::Matrix4f);
  *t = this->table;
  return;
}
bool Storage::write_table(boost::shared_ptr<Eigen::Matrix4f> &t)
{
  if (t)
  {
    LOCK guard(mtx_table);
    this->table = *t;
    return true;
  }
  ROS_WARN("[Storage][%s] Passed Table transformation is empty! Not writing anything in Storage", __func__);
  return false;
}
void Storage::read_tracked_index(int &idx)
{
  LOCK guard(mtx_index);
  idx = this->index;
  return;
}
void Storage::write_tracked_index(int idx)
{
  LOCK guard(mtx_index);
  this->index = idx;
  return;
}
