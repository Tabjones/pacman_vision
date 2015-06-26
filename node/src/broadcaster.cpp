#include "pacman_vision/broadcaster.h"
#include "pacman_vision/utility.h"

/////////////////
// Broadcaster //
/////////////////
Broadcaster::Broadcaster(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor)
{
  this->nh = ros::NodeHandle (n, "broadcaster");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->storage = stor;
  tf = rviz_markers = true;
  rviz_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("objects", 1);
}
Broadcaster::~Broadcaster()
{
  this->nh.shutdown();
}

void Broadcaster::compute_transforms()
{
  if (!this->storage->read_obj_transforms(estimated) || !this->storage->read_obj_names(names) )
  {
    ROS_WARN("[Broadcaster][%s] Can not read estimated objects to broadcast. Did you run an estimation with Estimator module ?", __func__);
    return;
  }
  int size = estimated->size();
  transforms.resize(size);
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
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.3f;
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration(1);
    markers.markers.push_back(marker);
  }
}

void Broadcaster::create_box_marker(visualization_msgs::Marker &box)
{
  //bounding box visualization TODO
  box.type = visualization_msgs::Marker::LINE_STRIP;
  box.header.frame_id = "/kinect2_rgb_optical_frame";
  box.header.stamp = ros::Time();
  box.ns = "bounding_box";
  box.id = 0;
  box.scale.x = 0.002;
  box.pose = pose;
  box.action = visualization_msgs::Marker::ADD;
  box.color.r = 0.0f;
  box.color.g = 0.0f;
  box.color.b = 1.0f;
  box.color.a = 1.0f;
  box.lifetime = ros::Duration(1);
  geometry_msgs::Point p;
  p.x = factor*x1;
  p.y = factor*y1;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y1;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y2;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x1;
  p.y = factor*y2;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x1;
  p.y = factor*y1;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x1;
  p.y = factor*y1;
  p.z = factor*z2;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y1;
  p.z = factor*z2;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y1;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y1;
  p.z = factor*z2;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y2;
  p.z = factor*z2;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y2;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x2;
  p.y = factor*y2;
  p.z = factor*z2;
  box.points.push_back(p);
  p.x = factor*x1;
  p.y = factor*y2;
  p.z = factor*z2;
  box.points.push_back(p);
  p.x = factor*x1;
  p.y = factor*y2;
  p.z = -factor*z1;
  box.points.push_back(p);
  p.x = factor*x1;
  p.y = factor*y2;
  p.z = factor*z2;
  box.points.push_back(p);
  p.x = factor*x1;
  p.y = factor*y1;
  p.z = factor*z2;
  box.points.push_back(p);
  rviz_marker_pub.publish(box);
}

void Broadcaster::broadcast_once()
{
  if (tf)
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
