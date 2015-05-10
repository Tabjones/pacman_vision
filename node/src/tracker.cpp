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
  this->orig_model.reset(new PTC);
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

  started = lost_it = false;
  error_count=0;
  leaf = 0.02f;
  factor = 1.5f;
  rej_distance = 0.025f;
  disturbance_counter=0;
  ROS_INFO("[Tracker] Tracker module tries to follow an already estimated object around the scene. For it to work properly please enable at least downsampling");
}
Tracker::~Tracker()
{
  this->nh.shutdown();
}

//tracker step 
void Tracker::track()
{
  if (error_count >= 30)
  {
    //failed 10 times in a row
    ROS_ERROR("[Tracker][%s] Object is lost ... stopping tracker...",__func__);
    started = false;
    lost_it = true;
    return;
  }
  Eigen::Matrix4f inv_trans;
  PTC::Ptr target (new PTC);
  PTC::Ptr tmp (new PTC);
  inv_trans = transform.inverse();
  //boundingbox
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
  pass.setFilterLimits(-factor*z1, factor*z2); //z1 is positive due to model, adding a minus
  pass.filter(*tmp);
  if (tmp->points.size() <= 50)
  {
    ROS_ERROR("[Tracker][%s] Not enought points in bounding box, retryng with larger bounding box", __func__);
    factor += 0.5;
    rej_distance +=0.005;
    ++error_count;
    return;
  }
  pcl::transformPointCloud(*tmp, *target, transform);
  //align
  if (old_leaf != leaf)
  {
    vg.setInputCloud (orig_model);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter (*model);
    old_leaf = leaf;
    ce->setInputSource(model);
    crd->setInputSource<PTT>(model);
    icp.setInputSource(model);
  }
  pcl::Correspondences corr, filt, interm;
  ce->setInputTarget(target);
  ce->determineCorrespondences(corr);
  crd->setInputTarget<PTT>(target);
  crd->setMaximumDistance(rej_distance);
  std::cout<<crd->getMaximumDistance()<<std::endl<<std::flush;
  crd->getRemainingCorrespondences(corr, interm);
  cro2o->getRemainingCorrespondences(interm, filt);
  //crsc->setInputTarget(target);
  icp.setInputTarget(target);
  if (disturbance_counter >= 10)
  {
    /* TODO need to rethink a better way, maybe centroid
    boost::random::mt19937 gen(std::time(0));
    boost::random::uniform_int_distribution<> angle(10,20);
    float angx,angy,angz;
    if (angle(gen)%2)
      angx = -D2R*angle(gen);
    else
      angx = D2R*angle(gen);
    Eigen::AngleAxisf rotx (angx, Eigen::Vector3f::UnitX());
    T_rotx << rotx.matrix()(0,0), rotx.matrix()(0,1), rotx.matrix()(0,2), -0.05,
           rotx.matrix()(1,0), rotx.matrix()(1,1), rotx.matrix()(1,2), 0,
           rotx.matrix()(2,0), rotx.matrix()(2,1), rotx.matrix()(2,2), 0.05,
           0,                0,                  0,                 1;
    if (angle(gen)%2)
      angz = -D2R*angle(gen);
    else
      angz = D2R*angle(gen);
    Eigen::AngleAxisf rotz (angz, Eigen::Vector3f::UnitZ());
    T_rotz << rotz.matrix()(0,0), rotz.matrix()(0,1), rotz.matrix()(0,2), 0,
           rotz.matrix()(1,0), rotz.matrix()(1,1), rotz.matrix()(1,2), 0,
           rotz.matrix()(2,0), rotz.matrix()(2,1), rotz.matrix()(2,2), 0,
           0,                0,                  0,                 1;
    if (angle(gen)%2)
      angy = -D2R*angle(gen);
    else
      angy = D2R*angle(gen);
    Eigen::AngleAxisf roty (angy, Eigen::Vector3f::UnitY());
    T_roty << roty.matrix()(0,0), roty.matrix()(0,1), roty.matrix()(0,2), 0,
           roty.matrix()(1,0), roty.matrix()(1,1), roty.matrix()(1,2), 0,
           roty.matrix()(2,0), roty.matrix()(2,1), roty.matrix()(2,2), 0,
           0,                0,                  0,                 1;

    ROS_WARN("DISTURBANCE !! %g %g %g", angx/D2R, angy/D2R, angz/D2R);
    Eigen::Matrix4f disturbed;
    disturbed = (T_roty*T_rotz*T_rotx*inv_trans).inverse();
    icp.align(*tmp, disturbed);
    */
    disturbance_counter = 0;
  }
  else
    icp.align(*tmp, transform);
  float fitness = icp.getFitnessScore();
  ROS_INFO("corr: %d, filt: %d, fitness: %g", (int)corr.size(), (int)filt.size(), fitness);
  this->transform = icp.getFinalTransformation();
  //adjust distance and factor according to fitness
  if (fitness > 0.0008 ) //something is probably wrong
  {
    rej_distance +=0.001;
    factor += 0.05;
    ++disturbance_counter;
    return;
  }
  else if (fitness < 0.0005)
  {
    rej_distance -=0.005;
    if(rej_distance < 0.025)
      rej_distance = 0.025; //we dont want to go lower than this
    factor -=0.05;
    if(factor < 1.5)
      factor = 1.5;
  }
  error_count = 0;
  disturbance_counter = 0;
}

void Tracker::find_object_in_scene()
{
//TODO
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
  if (boost::filesystem::exists(model_path) && boost::filesystem::is_regular_file(model_path))
  {
    if (pcl::io::loadPCDFile(model_path.c_str(), *orig_model))
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
  old_leaf = leaf;
  vg.setInputCloud (orig_model);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter (*model);
  pcl::io::savePCDFile("/home/tabjones/Desktop/model.pcd", *model); //TODO tmp
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
  icp.addCorrespondenceRejector(crd);
  icp.addCorrespondenceRejector(cro2o);
  //icp.addCorrespondenceRejector(crsc);
  icp.setInputSource(model);
  //do one step of icp
  icp.setTransformationEstimation(teDQ);
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

