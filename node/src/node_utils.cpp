#include <pacman_vision/vision_node.h>


void VisionNode::create_arm_box_marker(Eigen::Matrix4f& t, visualization_msgs::Marker &marker, const Box lim, int i, bool right)
{
  this->broadcaster_module->create_box_marker(marker, lim);
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  geometry_msgs::Pose pose;
  tf::Transform tf;
  fromEigen(t, pose, tf);
  marker.pose = pose;
  if (right)
    marker.ns = "Right Arm Boxes";
  else
    marker.ns = "Left Arm Boxes";
  marker.id = i+1;
}

void VisionNode::crop_arm(PC::Ptr source, PC::Ptr& dest, bool right)
{
  if(!source)
    return;
  if(!dest)
    dest.reset(new PC);
  if (right)
  {
    if(! this->storage->read_right_arm(right_arm) )
    {
      right_arm.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
      right_arm->resize(6);
      for (auto& x: *right_arm)
        x.setIdentity();
    }
    for(int i=0; i<right_arm->size(); ++i)
    {
      crop_a_box(source, dest, right_arm->at(i), lwr_arm[i], true, false);
      pcl::copyPointCloud(*dest, *source);
    }
  }
  else
  {
    if(! this->storage->read_left_arm(left_arm) )
    {
      left_arm.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
      left_arm->resize(6);
      for (auto& x: *left_arm)
        x.setIdentity();
    }
    for(int i=0; i<left_arm->size(); ++i)
    {
      crop_a_box(source, dest, left_arm->at(i), lwr_arm[i], true, false);
      pcl::copyPointCloud(*dest, *source);
    }
  }
}

void VisionNode::process_scene()
{
  PC::Ptr source (new PC);
  PC::Ptr dest;
  source = this->scene;
  //check if we need to crop scene
  if (filter)
  {
    //check if have a table transform
    if (use_table_trans)
    {
      this->storage->read_table(table_trans);
      if (table_trans)
      {
        //hardcoded and unused
        Box table_limits(0.1, -1.15, -0.1, 0.825, 0.1, 1.5);
        //just use old limits but transformed
        crop_a_box(source, dest, *table_trans, *limits, false, keep_organized);
      }
    }
    else
      crop_a_box(source, dest, Eigen::Matrix4f::Identity(), *limits, false, keep_organized);
  }
  //check if we need to downsample scene
  if (downsample) //cannot keep organized cloud after voxelgrid
  {
    VoxelGrid<PT> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setDownsampleAllData(true);
    if (dest)
      pcl::copyPointCloud(*dest, *source);
    else
      dest.reset(new PC);
    vg.setInputCloud (source);
    vg.filter (*dest);
  }
  if (plane)
  {
    pcl::SACSegmentation<PT> seg;
    pcl::ExtractIndices<PT> extract;
    //coefficients
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //plane segmentation
    if (dest)
      pcl::copyPointCloud(*dest, *source);
    else
      dest.reset(new PC);
    seg.setInputCloud(source);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (this->plane_tol);
    seg.segment(*inliers, *coefficients);
    //extract what's on top of plane
    extract.setInputCloud(seg.getInputCloud());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*dest);
  }
  //crop arms if listener is active and we set it
  if ( (crop_r_arm || crop_l_arm || crop_r_hand || crop_l_hand) && this->en_listener && this->listener_module )
  {
    if (crop_l_arm)
    {
      if (dest)
        pcl::copyPointCloud(*dest, *source);
      crop_arm(source, dest, false);
    }
    if (crop_r_arm)
    {
      if (dest)
        pcl::copyPointCloud(*dest, *source);
      crop_arm(source, dest, true);
    }
    if (crop_r_hand)
    {
      //right hand //TODO
    }
    if (crop_l_hand)
    {
      //left_hand //TODO
    }
  }
  //Save into storage
  if (dest)
  {
    pcl::copyPointCloud(*dest, *scene_processed);
    this->storage->write_scene_processed(this->scene_processed);
  }
  else
  {
    this->storage->write_scene_processed(this->scene);
  }
}

void VisionNode::publish_scene_processed()
{
  //republish processed cloud
  if (scene_processed && scene)
  {
    if (!scene_processed->empty() && !scene->empty())
    {
      /* |passt   | voxelgrid   |segment | | arms or hands croppings                                 */
      if ((filter || downsample || plane || crop_l_arm || crop_r_arm || crop_r_hand || crop_l_hand ) && (pub_scene.getNumSubscribers()>0))
        pub_scene.publish(*scene_processed);
      else if (pub_scene.getNumSubscribers()>0)
        pub_scene.publish(*scene);
    }
  }
}
