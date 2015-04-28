#include "pacman_vision/estimator.h"
#include "pacman_vision/utility.hpp"
///////////////////
//Estimator Class//
///////////////////

//Constructor
Estimator::Estimator(NHPtr nhptr)
{
  PointCloud<PointXYZRGBA> a;
  this->scene = a.makeShared();
  this->nh_ptr = nhptr;
  this->db_path = (ros::package::getPath("pacman_vision") + "/database" );
  if (!boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path))
    ROS_WARN("[Estimator][%s] Database for pose estimation does not exists!! Plese put one in /database folder, before trying to perform a pose estimation.",__func__);
  this->srv_estimate = nh_ptr->advertiseService("estimate", &Estimator::cb_estimate, this);
  //init params
  nh_ptr->param<bool>("/pacman_vision/estimator/object_calibration", calibration, false);
  nh_ptr->param<int>("/pacman_vision/estimator/iterations", iterations, 10);
  nh_ptr->param<int>("/pacman_vision/estimator/neighbors", neighbors, 10);
  nh_ptr->param<double>("/pacman_vision/estimator/clus_tol", clus_tol, 0.05);
  pe.setParam("verbosity",2);
  pe.setParam("progItera",iterations);
  pe.setParam("icpReciprocal",1);
  pe.setParam("kNeighbors",neighbors);
  pe.setParam("downsampling",0); //downsampling is performed as soon a scene is acquired
  pe.setDatabase(db_path);
}

int Estimator::extract_clusters()
{
  if (scene->empty())
    return -1;
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud(scene);
  seg.segment(*inliers, *coefficients);
  //extract plane and whats on top
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  extract.setInputCloud(scene);
  extract.setIndices(inliers);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_top (new pcl::PointCloud<pcl::PointXYZRGBA>);
  extract.setNegative(true);
  extract.filter(*table_top);
  //cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance(clus_tol);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(table_top->points.size());
  ec.setInputCloud(table_top);
  ec.extract(cluster_indices);
  //lock guard
  boost::lock_guard<boost::mutex> guard(mtx);
  clusters.resize(cluster_indices.size());
  names.resize(cluster_indices.size());
  ids.resize(cluster_indices.size());
  estimations.resize(cluster_indices.size());
  int j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
  {
   // pcl::PointCloud<pcl::PointXYZRGBA> object;
    extract.setInputCloud(table_top);
    extract.setIndices(boost::make_shared<PointIndices>(*it));
    extract.setNegative(false);
    //for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    //  object.points.push_back(table_top->points[*pit]);
    //object.width = object.points.size();
    //object.height = 1;
    //object.is_dense = true;
    //pcl::copyPointCloud(object,clusters[j]);
    extract.filter(clusters[j]);
    ++j;
  }
  return clusters.size();
}

bool Estimator::cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res)
{
  estimator = boost::thread(&Estimator::estimator_thread, this);
  geometry_msgs::Pose pose;
  tf::Transform trans;
  estimator.join();
  for (int i=0; i<estimations.size(); ++i)
  {
    fromEigen(estimations[i], pose, trans);
    pacman_vision_comm::pe pose_est;
    pose_est.pose = pose;
    pose_est.name = names[i];
    pose_est.id = ids[i];
    res.estimated.poses.push_back(pose_est);
  }
  return true;
}

void Estimator::estimator_thread()
{
  if (this->extract_clusters() < 1)
  {
    ROS_ERROR("[Estimator][%s] No object clusters found in scene, aborting pose estimation...",__func__);
    return;
  }
  boost::lock_guard<boost::mutex> guard(mtx);
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
  return;
}

