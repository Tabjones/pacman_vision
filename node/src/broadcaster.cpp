#include "pacman_vision/broadcaster.h"
#include "pacman_vision/utility.h"

/////////////////
// Broadcaster //
/////////////////
Broadcaster::Broadcaster(ros::NodeHandle &n)
{
  this->nh = ros::NodeHandle (n, "broadcaster");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  tf = rviz_markers = true;
  rviz_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("objects", 1);
}
Broadcaster::~Broadcaster()
{
  this->nh.shutdown();
}

void Broadcaster::compute_transforms()
{
  int size = estimated.size();
  transforms.clear(); 
  markers.markers.clear();
  for (int i=0; i<size; ++i) //if size is zero dont do anything
  {
    geometry_msgs::Pose pose;
    tf::Transform trans;
    fromEigen(estimated[i], pose, trans);
    transforms.push_back(trans);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns=ids[i].c_str();
    if (ids[i].compare(names[i]) != 0)
    {
      std::vector<std::string> vst;
      boost::split(vst, names[i], boost::is_any_of("_"), boost::token_compress_on);
      int id = std::stoi(vst.at(vst.size()-1));
      marker.id = id;
    }
    else
      marker.id = 1;
    marker.scale.x=1;
    marker.scale.y=1;
    marker.scale.z=1;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    std::string mesh_path ("package://asus_scanner_models/" + ids[i] + "/" + ids[i] + ".stl");
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

void Broadcaster::broadcast_once()
{
  if (tf)
  {
    for (int i = 0; i < transforms.size(); ++i)
      tf_broadcaster.sendTransform(tf::StampedTransform(transforms[i], ros::Time::now(), "/camera_rgb_optical_frame", names[i].c_str()));
  }
  if (rviz_markers)
    rviz_markers_pub.publish(markers);
}

void Broadcaster::spin_once()
{
  this->queue_ptr->callAvailable(ros::WallDuration(0, 10000));
}
