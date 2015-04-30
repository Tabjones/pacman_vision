#include "pacman_vision/estimator.h"
#include "pacman_vision/utility.hpp"

#include <pcl/visualization/pcl_visualizer.h>
///////////////////
//Estimator Class//
///////////////////

//Constructor
Estimator::Estimator(ros::NodeHandle &n)
{
  this->scene.reset(new PointCloud<PointXYZRGBA> );
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
  pe.setParam("verbosity",1);
  pe.setParam("progItera",iterations);
  pe.setParam("icpReciprocal",1);
  pe.setParam("kNeighbors",neighbors);
  pe.setParam("downsampling",1);
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
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
  std::vector<pcl::PointIndices> cluster_indices;
  seg.setInputCloud(scene);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.02);
  seg.segment(*inliers, *coefficients);
  //extract plane and whats on top
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  extract.setInputCloud(scene);
  extract.setIndices(inliers);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_top (new pcl::PointCloud<pcl::PointXYZRGBA>);
  extract.setNegative(true);
  extract.filter(*table_top);
  //cluster extraction
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setInputCloud(table_top);
  ec.setSearchMethod(tree);
  ec.setClusterTolerance(clus_tol);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(table_top->points.size());
  ec.extract(cluster_indices);
  cout<<"clus "<<cluster_indices.size()<<std::endl;
  cluster_indices.resize(cluster_indices.size()-1); //TODO TMP FIX
  clusters.resize(cluster_indices.size());
  names.resize(cluster_indices.size());
  ids.resize(cluster_indices.size());
  estimations.resize(cluster_indices.size());
  int j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
  {
    pcl::PointCloud<pcl::PointXYZRGBA> object;
  //  extract.setInputCloud(table_top);
  //  extract.setIndices(boost::make_shared<PointIndices>(*it));
  //  extract.setNegative(false);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      object.points.push_back(table_top->points[*pit]);
    }
    object.width = object.points.size();
    object.height = 1;
    object.is_dense = true;
    pcl::copyPointCloud(object,clusters[j]);
    //extract.filter(clusters[j]);
  }
  ROS_INFO("[Estimator][%s] Found %d clusters of possible objects.",__func__,(int)clusters.size());
  return clusters.size();
}

bool Estimator::cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res)
{
  if (this->extract_clusters() < 1)
  {
    ROS_ERROR("[Estimator][%s] No object clusters found in scene, aborting pose estimation...",__func__);
    return false;
  }
  pe.setDatabase(db_path);
  for (int i=0; i<clusters.size(); ++i)
  {
    pe.setQuery("object", clusters[i]);
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
  geometry_msgs::Pose pose;
  tf::Transform trans;
  for (int i=0; i<estimations.size(); ++i)
  {
    fromEigen(estimations[i], pose, trans);
    pacman_vision_comm::pe pose_est;
    pose_est.pose = pose;
    pose_est.name = names[i];
    pose_est.id = ids[i];
    res.estimated.poses.push_back(pose_est);
  }
  ROS_INFO("[Estimator][%s] Pose Estimation complete!", __func__);
  return true;
}

void Estimator::spin_once()
{
  //process this module callbacks
  this->queue_ptr->callAvailable(ros::WallDuration(0, 10000));
}

