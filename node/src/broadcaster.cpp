#include "pacman_vision/broadcaster.h"

/////////////////
// Broadcaster //
/////////////////
Broadcaster::Broadcaster(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor)
{
  this->nh = ros::NodeHandle (n, "broadcaster");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->storage = stor;
  obj_tf = rviz_markers = true;
  rviz_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("broadcasted_markers", 1);
}
Broadcaster::~Broadcaster()
{
  this->nh.shutdown();
}

void Broadcaster::elaborate_estimated_objects()
{
  if (!this->storage->read_obj_transforms(estimated) || !this->storage->read_obj_names(names) )
  {
    ROS_WARN("[Broadcaster][%s] Can not read estimated objects to broadcast. Did you run an estimation with Estimator module ?", __func__);
    return;
  }
  int size = estimated->size();
  transforms.resize(size);
  //read tracked object index if Tracker is tracking
  int index;
  this->storage->read_tracked_index(index);
  for (int i=0; i<size; ++i) //if size is zero dont do anything
  {
    geometry_msgs::Pose pose;
    fromEigen(estimated->at(i), pose, transforms[i]);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/kinect2_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns=names->at(i).second.c_str();
    if (names->at(i).second.compare(names->at(i).first) != 0)
    {
      std::vector<std::string> vst;
      boost::split(vst, names->at(i).first, boost::is_any_of("_"), boost::token_compress_on);
      int id = std::stoi(vst.at(vst.size()-1));
      marker.id = id;
    }
    else
      marker.id = 1;
    marker.scale.x=1;
    marker.scale.y=1;
    marker.scale.z=1;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    std::string mesh_path ("package://asus_scanner_models/" + names->at(i).second + "/" + names->at(i).second + ".stl");
    marker.mesh_resource = mesh_path.c_str();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.lifetime = ros::Duration(1);
    if (index >=0 && index == i)
    {
      //We are Tracking the i-th object, color it red
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.3f;
      marker.color.a = 1.0f;
      //publish also bounding box of tracked object
      //We need to create it first
      visualization_msgs::Marker box_marker;
      boost::shared_ptr<Box> bb;
      this->storage->read_tracked_box(bb);
      this->create_box_marker(box_marker, bb);
      box_marker.ns = "Tracked Object Bounding Box";
      box_marker.id = 0;
      tf::Transform t;
      Eigen::Matrix4f inv = estimated->at(i).inverse();
      //overwrite object pose, it was already saved into marker
      fromEigen(inv, pose, t);
      box_marker.pose=pose;
      this->markers.markers.push_back(box_marker);
    }
    else
    {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.3f;
      marker.color.a = 1.0f;
    }
    markers.markers.push_back(marker);
  }
}

bool Broadcaster::create_box_marker(visualization_msgs::Marker &box, boost::shared_ptr<Box> &limits)
{
  //Does not set time header, pose, namespace and id of marker
  if(!limits)
  {
    ROS_ERROR("[Broadcaster][%s] Cannot create a box marker with empty passed limits, aborting...", __func__);
    return false;
  }
  box.type = visualization_msgs::Marker::LINE_LIST;
  box.header.frame_id = "/kinect2_rgb_optical_frame";
  //adjust these two values later if needed
  box.ns = "box";
  box.id = 0;
  ////////////////////////////////////////
  box.scale.x = 0.002;
  box.action = visualization_msgs::Marker::ADD;
  //blue color default
  box.color.r = 0.0f;
  box.color.g = 0.0f;
  box.color.b = 1.0f;
  box.color.a = 1.0f;
  box.lifetime = ros::Duration(1);
  geometry_msgs::Point p, pf;
  //0-1
  p.x = limits->x1;
  p.y = limits->y1;
  p.z = limits->z1;
  pf.x = limits->x2;
  pf.y = limits->y1;
  pf.z = limits->z1;
  box.points.push_back(p);
  box.points.push_back(pf);
  //2-3
  pf.x = limits->x1;
  pf.y = limits->y2;
  box.points.push_back(p);
  box.points.push_back(pf);
  //4-5
  pf.x = limits->x1;
  pf.y = limits->y1;
  pf.z = limits->z2;
  box.points.push_back(p);
  box.points.push_back(pf);
  //6-7
  p.x = limits->x2;
  p.y = limits->y2;
  p.z = limits->z2;
  pf.x = limits->x2;
  pf.y = limits->y2;
  pf.z = limits->z1;
  box.points.push_back(p);
  box.points.push_back(pf);
  //8-9
  pf.x = limits->x1;
  pf.y = limits->y2;
  pf.z = limits->z2;
  box.points.push_back(p);
  box.points.push_back(pf);
  //10-11
  pf.x = limits->x2;
  pf.y = limits->y1;
  pf.z = limits->z2;
  box.points.push_back(p);
  box.points.push_back(pf);
  //12-13
  p.x = limits->x1;
  p.y = limits->y1;
  p.z = limits->z2;
  box.points.push_back(p);
  box.points.push_back(pf);
  //14-15
  pf.x = limits->x1;
  pf.y = limits->y2;
  pf.z = limits->z2;
  box.points.push_back(p);
  box.points.push_back(pf);
  //16-17
  p.x = limits->x2;
  p.y = limits->y2;
  p.z = limits->z1;
  pf.x = limits->x2;
  pf.y = limits->y1;
  pf.z = limits->z1;
  box.points.push_back(p);
  box.points.push_back(pf);
  //18-19
  pf.x = limits->x1;
  pf.y = limits->y2;
  pf.z = limits->z1;
  box.points.push_back(p);
  box.points.push_back(pf);
  //20-21
  p.x = limits->x1;
  p.y = limits->y2;
  p.z = limits->z2;
  box.points.push_back(p);
  box.points.push_back(pf);
  //22-23
  p.x = limits->x2;
  p.y = limits->y1;
  p.z = limits->z2;
  pf.x = limits->x2;
  pf.y = limits->y1;
  pf.z = limits->z1;
  box.points.push_back(p);
  box.points.push_back(pf);
  return true;
}

void Broadcaster::broadcast_once()
{
  if (obj_tf)
  {
    for (int i = 0; i < transforms.size(); ++i)
      tf_broadcaster.sendTransform(tf::StampedTransform(transforms[i], ros::Time::now(), "/kinect2_rgb_optical_frame", names->at(i).first.c_str()));
  }
  if (rviz_markers)
  {
    for (int i = 0; i< markers.markers.size(); ++i)
      markers.markers[i].header.stamp = ros::Time();
    rviz_markers_pub.publish(markers);
  }
}

void Broadcaster::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}
