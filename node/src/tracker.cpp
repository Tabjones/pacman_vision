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
  ce.reset( new pcl::registration::CorrespondenceEstimation<PTT, PTT, float>);
  crd.reset( new pcl::registration::CorrespondenceRejectorDistance);
  cro2o.reset( new pcl::registration::CorrespondenceRejectorOneToOne);
  teDQ.reset(new pcl::registration::TransformationEstimationDualQuaternion<PTT,PTT,float>);
 // crsc.reset( new pcl::registration::CorrespondenceRejectorSampleConsensus<PTT>);

  started = false;
  downsample = true;
  leaf = 0.02f;
  factor = 1.1f;
  rej_distance = 0.025f;
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
  pcl::Correspondences corr, filt;
  ce->setInputTarget(target);
  ce->determineCorrespondences(corr);
  crd->setInputTarget<PTT>(target);
  crd->setMaximumDistance(rej_distance);
  //cro2o->getRemainingCorrespondences(corr, o2o);
  crd->getRemainingCorrespondences(corr, filt);
  double actual_corr_ratio = filt.size()/corr.size();
  //crsc->setInputTarget(target);
  icp.setInputTarget(target);
  icp.align(*tmp, transform);
  ROS_INFO("corr: %d, filt: %d, fitness: %g, actual: %g", (int)corr.size(), (int)filt.size(), fitness, icp.getFitnessScore(rej_distance));
  this->transform = icp.getFinalTransformation();
  //TODO adjust distance and factor according to fitness and rej.size
  if (icp.getFitnessScore() > 0.0008) //something is probably wrong
  {
    rej_distance +=0.003;
    factor += 0.01;
    //create a disturbance
  }
  else if (icp.getFitnessScore() < 0.0003)
  {
    if(rej_distance > 0.01)
      rej_distance -=0.003;
    if(factor >= 1.0)
      factor -=0.01;
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
  //Get the minimum and maximum values on each of the 3 (x-y-z) dimensions of model
  std::vector<float> xvec,yvec,zvec;
  for (int i=0; i<model->points.size(); ++i)
  {
    xvec.push_back(model->points[i].x);
    yvec.push_back(model->points[i].y);
    zvec.push_back(model->points[i].z);
  }
  x1 = *std::min_element(xvec.begin(), xvec.end());
  y1 = *std::min_element(yvec.begin(), yvec.end());
  z1 = *std::min_element(zvec.begin(), zvec.end());
  x2 = *std::max_element(xvec.begin(), xvec.end());
  y2 = *std::max_element(yvec.begin(), yvec.end());
  z2 = *std::max_element(zvec.begin(), zvec.end());
  //init icps
  icp.setUseReciprocalCorrespondences(false);
  icp.setMaximumIterations(50);
  icp.setTransformationEpsilon(1e-9);
  icp.setEuclideanFitnessEpsilon(1e-9);
  ce->setInputSource(model);
  icp.setCorrespondenceEstimation(ce);
  crd->setInputSource<PTT>(model);
  crd->setMaximumDistance(rej_distance);
  //crsc->setInputSource(model);
  //crsc->setInlierThreshold(0.02);
  //crsc->setMaximumIterations(5);
  //crsc->setRefineModel(true);
  //icp.addCorrespondenceRejector(cro2o);
  icp.addCorrespondenceRejector(crd);
  //icp.addCorrespondenceRejector(crsc);
  icp.setInputSource(model);
  //do one step of icp
  ce->setInputTarget(scene);
  crd->setInputTarget<PTT>(scene);
  pcl::Correspondences corr, filt;
  ce->determineCorrespondences(corr);
  crd->getRemainingCorrespondences(corr, filt);
  corr_ratio = filt.size()/corr.size();
  icp.setInputTarget(scene);
  icp.setTransformationEstimation(teDQ);
  icp.align(*tmp, transform);
  fitness = icp.getFitnessScore(rej_distance); //save fitness of correct alignment
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

