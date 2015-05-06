#include "pacman_vision/estimator.h"
#include "pacman_vision/utility.h"

///////////////////
//Estimator Class//
///////////////////

//Constructor
Estimator::Estimator(ros::NodeHandle &n)
{
  this->scene.reset(new PEC);
  this->nh = ros::NodeHandle (n, "estimator");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->db_path = (ros::package::getPath("pacman_vision") + "/database" );
  if (!boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path))
    ROS_WARN("[Estimator][%s] Database for pose estimation does not exists!! Plese put one in /database folder, before trying to perform a pose estimation.",__func__);
  this->srv_estimate = nh.advertiseService("estimate", &Estimator::cb_estimate, this);
  //init params
  calibration = false;
  iterations = 10;
  neighbors = 10;
  clus_tol = 0.05;
  downsampling = 1;
  no_segment=false;
  busy = false;
  pe.setParam("verbosity",2);
  pe.setParam("progItera",iterations);
  pe.setParam("icpReciprocal",1);
  pe.setParam("kNeighbors",neighbors);
  pe.setParam("downsampling",downsampling);
  pe.setDatabase(db_path);
}
Estimator::~Estimator()
{
  this->nh.shutdown();
}

int Estimator::extract_clusters()
{
  if (scene->empty())
    return -1;
  ROS_INFO("[Estimator][%s] Extracting object clusters with cluster tolerance of %g",__func__,clus_tol);
  //objects
  pcl::SACSegmentation<PET> seg;
  pcl::ExtractIndices<PET> extract;
  pcl::EuclideanClusterExtraction<PET> ec;
  pcl::search::KdTree<PET>::Ptr tree (new pcl::search::KdTree<PET>);
  PEC::Ptr table_top (new PEC);
  //coefficients
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  std::vector<pcl::PointIndices> cluster_indices;
  //plane segmentation
  //disable interruption during clustering, will be restored when di is destroyed
  boost::this_thread::disable_interruption di;
  if (!no_segment)
  {
    seg.setInputCloud(scene);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    seg.segment(*inliers, *coefficients);
    //extract what's on top of plane
    extract.setInputCloud(scene);
    extract.setIndices(inliers);
    extract.setNegative(true); 
    extract.filter(*table_top);
  }
  else
  {
    pcl::copyPointCloud(*scene,*table_top);
  }
  //cluster extraction
  tree->setInputCloud(table_top);
  ec.setInputCloud(table_top);
  ec.setSearchMethod(tree);
  ec.setClusterTolerance(clus_tol);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(table_top->points.size());
  ec.extract(cluster_indices);
  int size = (int)cluster_indices.size();
  clusters.clear();
  clusters.resize(size);
  names.clear();
  names.resize(size);
  ids.clear();
  ids.resize(size);
  estimations.clear();
  estimations.resize(size);
  int j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
  {
    PEC::Ptr object (new PEC);
    extract.setInputCloud(table_top);
    extract.setIndices(boost::make_shared<PointIndices>(*it));
    extract.setNegative(false);
  /*  
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      object->points.push_back(table_top->points[*pit]);
    }
    object->width = object->points.size();
    object->height = 1;
    object->is_dense = true;
    pcl::copyPointCloud(*object, clusters[j]);
    */
    extract.filter(clusters[j]);
  }
  ROS_INFO("[Estimator][%s] Found %d clusters of possible objects.",__func__,size);
  return size;
}

bool Estimator::cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res)
{
  this->estimate();
  geometry_msgs::Pose pose;
  tf::Transform trans;
  for (int i=0; i<estimations.size(); ++i)
  {
    fromEigen(estimations[i], pose, trans);
    pacman_vision_comm::pe pose_est;
    pose_est.pose = pose;
    pose_est.name = names[i];
    pose_est.id = ids[i];
    pose_est.parent_frame = "/camera_rgb_optical_frame";
    res.estimated.poses.push_back(pose_est);
  }
  this->busy = false;
  this->up_broadcaster = true;
  this->up_tracker = true;
  ROS_INFO("[Estimator][%s] Pose Estimation complete!", __func__);
  return true;
}

void Estimator::estimate()
{
  this->busy = true; //tell other modules that estimator is computing new estimations
  int size = this->extract_clusters();
  if (size < 1)
  {
    ROS_ERROR("[Estimator][%s] No object clusters found in scene, aborting pose estimation...",__func__);
    this->busy = false;
    return;
  }
  pe.setParam("downsampling",this->downsampling);
  pe.setDatabase(db_path);
  for (int i=0; i<size; ++i)
  {
    pcl::PointCloud<pcl::PointXYZRGBA> query;
    pcl::copyPointCloud(clusters[i], query);
    pe.setQuery("object", query);
    pe.generateLists();
    pe.refineCandidates();
    boost::shared_ptr<Candidate> pest (new Candidate);
    pe.getEstimation(pest);
    std::string name;
    pest->getName(name);
    std::vector<std::string> vst;
    boost::split (vst, name, boost::is_any_of("_"), boost::token_compress_on);
    if (this->calibration)
      names[i] = "object";
    else
      names[i] = vst.at(0);
    pe.getEstimationTransformation(estimations[i]);
    ids[i] = vst.at(0);
    ROS_INFO("[Estimator][%s] Found %s.",__func__,name.c_str());
  }
  //first check if we have more copy of the same object in names
  for (int i=0; i<names.size(); ++i)
  {
    int count(1);
    string name_original = names[i];
    if (i>0)
    {
      for (int j=0; j<i; ++j)
      {
        if (names[i].compare(names[j]) == 0)
        { //i-th name is equal to j-th name
         names[i] = name_original + "_" + std::to_string(++count);
        }
      }
    }
  }
}

void Estimator::spin_once()
{
  //process this module callbacks
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}

