#include "pacman_vision/tracker.h"
#include "pacman_vision/utility.h"

///////////////////
//Tracker Class//
///////////////////

//Constructor
Tracker::Tracker(ros::NodeHandle &n)
{
  this->scene.reset(new PC);
  this->nh = ros::NodeHandle (n, "tracker");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->srv_track_object = nh.advertiseService("track_object", &Tracker::cb_track_object, this);
  this->rviz_marker_pub = nh.advertise<visualization_msgs::Marker>("tracked_object", 1);
  started = false;
  window = 0.6;
}
Tracker::~Tracker()
{
  this->nh.shutdown();
}

//tracker v1 icp 
void Tracker::track_v1()
{
  pcl::IterativeClosestPoint<PT, PT, float> icp;
  pcl::PassThrough<PT> pass;
  pcl::VoxelGrid<PT> vg;
  icp.setUseReciprocalCorrespondences(false);
  icp.setMaximumIterations(5);
  icp.setTransformationEpsilon(1e-9);
  icp.setEuclideanFitnessEpsilon(1e-9);
  Eigen::Matrix4f inv_trans;
  pcl::PointCloud<PT>::Ptr target (new pcl::PointCloud<PT>);
  pcl::PointCloud<PT>::Ptr tmp (new pcl::PointCloud<PT>);
  inv_trans = transform.inverse();
  //filter
  pcl::transformPointCloud(*scene, *tmp, inv_trans);
  pass.setInputCloud(tmp);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-window/2, +window/2);
  pass.filter(*tmp);
  pass.setInputCloud(tmp);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-window/2, +window/2);
  pass.filter(*tmp);
  pass.setInputCloud(tmp);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-window/2, +window/2);
  pass.filter(*tmp);
  vg.setInputCloud (tmp);
  vg.setLeafSize(0.02f, 0.02f, 0.02f);
  vg.filter (*tmp);
  pcl::transformPointCloud(*tmp, *target, transform);
  //align
  icp.setInputTarget(target);
  icp.setInputSource(model);
  icp.align(*tmp, transform);
  transform = icp.getFinalTransformation();
}

bool Tracker::cb_track_object(pacman_vision_comm::track_object::Request& req, pacman_vision_comm::track_object::Response& res)
{
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
  PC::Ptr tmp (new PC);
  if (boost::filesystem::exists(model_path) && boost::filesystem::is_regular_file(model_path))
  {
    if (pcl::io::loadPCDFile(model_path.string(), *tmp))
    {
      ROS_ERROR("[Tracker][%s] Error loading model %s",__func__, model_path.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("[Tracker][%s] Request model %s does not exists in asus_scanner_models",__func__, model_path.c_str());
    return false;
  }
  pcl::VoxelGrid<PT> vg;
  vg.setInputCloud (tmp);
  vg.setLeafSize(0.02f, 0.02f, 0.02f);
  vg.filter (*model);
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

