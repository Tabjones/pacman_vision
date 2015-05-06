#include "pacman_vision/tracker.h"
#include "pacman_vision/utility.h"

///////////////////
//Tracker Class//
///////////////////

//Constructor
Tracker::Tracker(ros::NodeHandle &n)
{
  this->scene.reset(new PTC);
  this->model.reset(new PTC);
  this->nh = ros::NodeHandle (n, "tracker");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->srv_track_object = nh.advertiseService("track_object", &Tracker::cb_track_object, this);
  this->rviz_marker_pub = nh.advertise<visualization_msgs::Marker>("tracked_object", 1);
  started = false;
  downsample = true;
  leaf = 0.02f;
  factor = 1.1;
}
Tracker::~Tracker()
{
  this->nh.shutdown();
}

//tracker step 
void Tracker::track()
{
  Eigen::Matrix4f inv_trans;
  PTC::Ptr target (new PTC);
  PTC::Ptr tmp (new PTC);
  inv_trans = transform.inverse();
  //filter
  pcl::transformPointCloud(*scene, *tmp, inv_trans);
  pass.setInputCloud(tmp);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(factor*x1, factor*x2);
  pass.filter(*tmp);
  pass.setInputCloud(tmp);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(factor*y1, factor*y2);
  pass.filter(*tmp);
  pass.setInputCloud(tmp);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(factor*z1, factor*z2);
  pass.filter(*tmp);
  if(downsample)
  {
    PTC t;
    vg.setInputCloud (tmp);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter (t);
    pcl::transformPointCloud(t, *target, transform);
  }
  else
    pcl::transformPointCloud(*tmp, *target, transform);
  //align
  if (this->type == 0)
  {
    icp.setInputTarget(target);
    icp.setInputSource(model);
    icp.align(*tmp, transform);
    transform = icp.getFinalTransformation();
  }
  else if (this->type == 1)
  {
    gicp.setInputTarget(target);
    gicp.setInputSource(model);
    gicp.align(*tmp, transform);
    transform = gicp.getFinalTransformation();
  }
  else if (this->type == 2)
  {
    icp_nl.setInputTarget(target);
    icp_nl.setInputSource(model);
    icp_nl.align(*tmp, transform);
    transform = icp_nl.getFinalTransformation();
  }
}

bool Tracker::cb_track_object(pacman_vision_comm::track_object::Request& req, pacman_vision_comm::track_object::Response& res)
{
  if (req.name.empty())
  {
    ROS_ERROR("[Tracker][%s] You need to provide the name of the object you want to track!", __func__);
    return false;
  }
  std::string models_path (ros::package::getPath("asus_scanner_models"));
  int j = -1;
  for (int i=0; i<names.size(); ++i)
  {
    if (names[i].compare(req.name) == 0)
    {
      j = i;
      break;
    }
  }
  if (j == -1)
  {
    ROS_ERROR("[Tracker][%s] Cannot find %s from the pool of already estimated objects, check spelling or run an estimation first!", __func__, req.name.c_str());
    return false;
  }
  name = (req.name);
  std::vector<std::string> vst;
  boost::split(vst, name, boost::is_any_of("_"), boost::token_compress_on);
  id = vst.at(0);
  transform = estimations[j];
  boost::filesystem::path model_path (models_path + "/" + id + "/" + id + ".pcd");
  PTC::Ptr tmp (new PTC);
  if (boost::filesystem::exists(model_path) && boost::filesystem::is_regular_file(model_path))
  {
    if (pcl::io::loadPCDFile(model_path.c_str(), *tmp))
    {
      ROS_ERROR("[Tracker][%s] Error loading model %s",__func__, model_path.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("[Tracker][%s] Request model (%s) does not exists in asus_scanner_models package",__func__, model_path.stem().c_str());
    return false;
  }
  pcl::VoxelGrid<PTT> vg;
  vg.setInputCloud (tmp);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter (*model);
  PTT min,max;
  //Get the minimum and maximum values on each of the 3 (x-y-z) dimensions of model
  pcl::getMinMax3D(*model, min, max);
  x1 = min.x;
  y1 = min.y;
  z1 = min.z;
  x2 = max.x;
  y2 = max.y;
  z2 = max.z;
  //init icps
  icp.setUseReciprocalCorrespondences(false);
  icp.setMaximumIterations(5);
  icp.setTransformationEpsilon(1e-9);
  icp.setEuclideanFitnessEpsilon(1e-9);
  icp_nl.setUseReciprocalCorrespondences(false);
  icp_nl.setMaximumIterations(5);
  icp_nl.setTransformationEpsilon(1e-9);
  icp_nl.setEuclideanFitnessEpsilon(1e-9);
  gicp.setUseReciprocalCorrespondences(false);
  gicp.setMaximumIterations(5);
  gicp.setTransformationEpsilon(1e-9);
  gicp.setEuclideanFitnessEpsilon(1e-9);
  gicp.setCorrespondenceRandomness(10);
  gicp.setMaximumOptimizerIterations(5);
  gicp.setRotationEpsilon(1e-4);
  //init rviz marker
  marker.header.frame_id = "/camera_rgb_optical_frame";
  marker.ns = std::string(id + "_tracked").c_str();
  marker.id = 0;
  marker.scale.x=1;
  marker.scale.y=1;
  marker.scale.z=1;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  std::string mesh_path ("package://asus_scanner_models/" + id + "/" + id + ".stl");
  marker.mesh_resource = mesh_path.c_str();
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.3f;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration(1);
  //we are ready to start
  started = true;
  return true;
}

void Tracker::broadcast_tracked_object()
{
  geometry_msgs::Pose pose;
  tf::Transform trans;
  fromEigen(transform, pose, trans);
  marker.header.stamp = ros::Time();
  marker.pose = pose;
  rviz_marker_pub.publish(marker);
  tf_broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "/camera_rgb_optical_frame", std::string(name + "_tracked").c_str()));
}

void Tracker::spin_once()
{
  //process this module callbacks
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}

