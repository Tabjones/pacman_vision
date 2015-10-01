#include <pacman_vision/vision_node.h>

void VisionNode::crop_a_box(PC::Ptr source, PC::Ptr& dest, const Eigen::Matrix4f& trans, boost::shared_ptr<Box> &lim, bool inside)
{
  if(!source)
    return;
  if(!dest)
    dest.reset(new PC);
  pcl::CropBox<PT> cb;
  //check if we need to maintain cloud organized
  if (keep_organized)
    cb.setKeepOrganized(true);
  else
    cb.setKeepOrganized(false);
  cb.setInputCloud (source);
  Eigen::Vector4f min,max;
  min << lim->x1, lim->y1, lim->z1, 1;
  max << lim->x2, lim->y2, lim->z2, 1;
  cb.setMin(min);
  cb.setMax(max);
  //Note this transform is applied to the box, not the cloud
  Eigen::Matrix3f Rot = trans.block<3,3>(0,0); //3x3 block starting at 0,0
  Eigen::Vector3f angles = Rot.eulerAngles(0,1,2);
  Eigen::Vector3f translation( trans(0,2), trans(1,2), trans(2,2));
  cb.setTranslation(translation);
  cb.setRotation(angles);

  cb.setNegative(inside);
  cb.filter (*dest);
}

void VisionNode::crop_arm(PC::Ptr source, PC::Ptr& dest, bool right)
{
  if(!source)
    return;
  if(!dest)
    dest.reset(new PC);
  boost::shared_ptr<Box> lim (new Box);
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
      if ( i == 0)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.06;
        lim->z1 = -0.06;
        lim->x2 = 0.06;
        lim->y2 = 0.094;
        lim->z2 = 0.192;
      }
      if ( i == 1)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.06;
        lim->z1 = 0;
        lim->x2 = 0.06;
        lim->y2 = 0.094;
        lim->z2 = 0.269;
      }
      if ( i == 2)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.094;
        lim->z1 = -0.06;
        lim->x2 = 0.06;
        lim->y2 = 0.06;
        lim->z2 = 0.192;
      }
      if ( i == 3)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.056;
        lim->z1 = 0;
        lim->x2 = 0.06;
        lim->y2 = 0.06;
        lim->z2 = 0.269;
      }
      if ( i == 4)
      {
        lim->x1 = -0.071;
        lim->y1 = -0.056;
        lim->z1 = -0.071;
        lim->x2 = 0.071;
        lim->y2 = 0.08;
        lim->z2 = 0.057;
      }
      if ( i == 5)
      {
        lim->x1 = -0.04;
        lim->y1 = -0.04;
        lim->z1 = -0.031;
        lim->x2 = 0.04;
        lim->y2 = 0.04;
        lim->z2 = 0;
      }
      crop_a_box(source, dest, right_arm->at(i), lim, true);
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
      if ( i == 0)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.06;
        lim->z1 = -0.06;
        lim->x2 = 0.06;
        lim->y2 = 0.094;
        lim->z2 = 0.192;
      }
      if ( i == 1)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.06;
        lim->z1 = 0;
        lim->x2 = 0.06;
        lim->y2 = 0.094;
        lim->z2 = 0.269;
      }
      if ( i == 2)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.094;
        lim->z1 = -0.06;
        lim->x2 = 0.06;
        lim->y2 = 0.06;
        lim->z2 = 0.192;
      }
      if ( i == 3)
      {
        lim->x1 = -0.06;
        lim->y1 = -0.056;
        lim->z1 = 0;
        lim->x2 = 0.06;
        lim->y2 = 0.06;
        lim->z2 = 0.269;
      }
      if ( i == 4)
      {
        lim->x1 = -0.071;
        lim->y1 = -0.056;
        lim->z1 = -0.071;
        lim->x2 = 0.071;
        lim->y2 = 0.08;
        lim->z2 = 0.057;
      }
      if ( i == 5)
      {
        lim->x1 = -0.04;
        lim->y1 = -0.04;
        lim->z1 = -0.031;
        lim->x2 = 0.04;
        lim->y2 = 0.04;
        lim->z2 = 0;
      }
      crop_a_box(source, dest, left_arm->at(i), lim, true);
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
        boost::shared_ptr<Box> table_limits (new Box);
        //hardcoded for now and unused
        table_limits->x1 = -0.1;
        table_limits->y1 = -1.15;
        table_limits->z1 = -0.1;
        table_limits->x2 = 0.825;
        table_limits->y2 = 0.1;
        table_limits->z2 = 1.5;
        //just use old limits but transformed
        crop_a_box(source, dest, *table_trans, limits, true);
      }
    }
    else
      crop_a_box(source, dest, Eigen::Matrix4f::Identity(), limits, true);
  }
  //check if we need to downsample scene
  if (downsample) //cannot keep organized cloud after voxelgrid
  {
    VoxelGrid<PT> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setDownsampleAllData(true);
    if (dest)
      vg.setInputCloud (dest);
    else
    {
      vg.setInputCloud (source);
      dest.reset(new PC);
    }
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
      seg.setInputCloud(dest);
    else
    {
      seg.setInputCloud(source);
      dest.reset(new PC);
    }
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
    this->scene_processed = dest;
    this->storage->write_scene_processed(this->scene_processed);
  }
  else
  {
    this->storage->write_scene_processed(this->scene);
  }
}
