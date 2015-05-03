#include "pacman_vision/tracker.h"

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
  //TODO add params
}
Tracker::~Tracker()
{
  this->nh.shutdown();
}

//tracker v1
void ObjectTracker::tracker_thread_body_v1()
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;
  icp.setUseReciprocalCorrespondences(false);
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-9);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr acquired (new pcl::PointCloud<pcl::PointXYZ>);
  while (nh.ok() && !stop_thread)
  {
    if (!acquire_scene(acquired))
    {
      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
      continue;
    }
    internal_lock.lock();
    icp.setInputTarget(acquired);
    icp.setInputSource(model);
    icp.align(*tmp, transform);
   /* 
    pcl::io::savePCDFile("/home/tabjones/target" + std::to_string(i)+ ".pcd", *actual); //TODO REMOVE
    pcl::io::savePCDFile("/home/tabjones/source"+std::to_string(i)+".pcd", *model); //TODO REMOVE
    pcl::io::savePCDFile("/home/tabjones/result"+std::to_string(i)+".pcd", *tmp); //TODO REMOVE
    */
    transform = icp.getFinalTransformation();
    internal_lock.unlock();
    broadcast_thread = boost::thread(&ObjectTracker::broadcast_thread_body, this);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}
//tracker v2 ultra slow (TODO find correspondences, maybe with features)
void ObjectTracker::tracker_thread_body_v2()
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  icp.setUseReciprocalCorrespondences(false);
  icp.setMaximumIterations(5);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-9);
  Eigen::Matrix4f temp_t, inv_trans;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
  while (nh.ok() && !stop_thread)
  {
    inv_trans = transform.inverse();
    temp_t = transform;
    internal_lock.lock();
    pcl::transformPointCloud(*actual, *tmp, inv_trans);
    internal_lock.unlock();
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
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter (*tmp);
    pcl::transformPointCloud(*tmp, *target, temp_t);
    icp.setInputTarget(target);

    internal_lock.lock();
    pcl::transformPointCloud(*past, *tmp, inv_trans);
    internal_lock.unlock();
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
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter (*source);
    icp.setInputSource(source);
    icp.align(*tmp, temp_t);
    
    internal_lock.lock();
    transform = icp.getFinalTransformation();
    internal_lock.unlock();
    
    broadcast_thread = boost::thread(&ObjectTracker::broadcast_thread_body, this);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
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
   
    return true;
  }
  else
    return false;
}

void Tracker::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0, 10000));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracker");
    ObjectTracker node;
    ros::Rate rate(50); //go at 50 hz
    ROS_INFO("object_tracker node started, call service to actually start tracking");
    while (node.nh.ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
///

void Tracker::spin_once()
{
  //process this module callbacks
  this->queue_ptr->callAvailable(ros::WallDuration(0, 100000));
}

