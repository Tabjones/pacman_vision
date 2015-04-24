// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

// PCL headers
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

//PEL
#include <pel.h>

// ROS generated headers
#include "scene_filter_node/acquire_scene.h"
#include "dual_manipulation_shared/estimate.h"
#include "dual_manipulation_shared/pe.h"
#include "dual_manipulation_shared/peArray.h"

//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

#define D2R 0.017453293 //degrees to radians conversion

class PoseEstimationOnline
{
  public:
    PoseEstimationOnline();
    ros::NodeHandle nh;
    PoseEstimation estimator;
    //method to acquire scene from openni2
    bool acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired);
    //method to convert from eigen to geometry_msgs and tf
    bool convert (Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);
    //method to perform estimation of queries
    void pose_estimate();
    //method to segment table and extract clusters
    int extract_queries();
    bool online_mode, object_calibration, broadcast_disabled;
    int k_neigh, iterations;
    double cluster_tol;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene;
    void stop_broadcast_thread();
    void start_broadcast_thread();
    bool thread_started;

  private:
    boost::thread broadcast_thread_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Publisher rviz_markers_pub_;
    std::vector<Eigen::Matrix4f> estimations_;
    ros::ServiceServer srv_estimate_;
    std::vector< pcl::PointCloud<pcl::PointXYZRGBA> > queries_;
    std::vector<std::string> names_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_;
    boost::filesystem::path db_path_;
    bool visualize_;
    bool stop_thread_;

    bool estimate_scene(dual_manipulation_shared::estimate::Request& req, dual_manipulation_shared::estimate::Response& res);
    void broadcast_thread_body();

};

//Class Constructor
PoseEstimationOnline::PoseEstimationOnline()
{
  pcl::PointCloud<pcl::PointXYZRGBA> a,b;
  scene = a.makeShared();
  table_ = b.makeShared();
  stop_thread_ = false;
  nh = ros::NodeHandle("pose_estimation_online");
  db_path_ = ( ros::package::getPath("pose_estimation_online") + "/database" );
  srv_estimate_ = nh.advertiseService("estimate", &PoseEstimationOnline::estimate_scene, this);
  nh.param("/pose_estimation_online/online_mode", online_mode, false);
  nh.param("/pose_estimation_online/object_calibration", object_calibration, false);
  nh.param("/pose_estimation_online/disable_tf_broadcast", broadcast_disabled, false);
  nh.param("/pose_estimation_online/iterations_in_prog_bisection", iterations, 10);
  nh.param("/pose_estimation_online/neighbors", k_neigh, 10);
  nh.param("/pose_estimation_online/clustering_tolerance", cluster_tol, 0.05);
  estimator.setParam("verbosity",2);
  estimator.setParam("progItera",iterations);
  estimator.setParam("icpReciprocal",1);
  estimator.setParam("kNeighbors",k_neigh);
  estimator.setParam("downsampling",0); //downsampling is performed as soon a scene is acquired
  thread_started = false;
  visualize_=false;
  rviz_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("estimated_objects", 1);
}

bool PoseEstimationOnline::convert(Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest)
{
  try
  {
    Eigen::Matrix3f rot;
    rot << source(0,0), source(0,1), source(0,2),
           source(1,0), source(1,1), source(1,2),
           source(2,0), source(2,1), source(2,2);
    Eigen::Quaternionf quad(rot);
    quad.normalize();
    tf::Quaternion q(quad.x(), quad.y(), quad.z(), quad.w());
    tf_dest.setOrigin(tf::Vector3(source(0,3), source(1,3), source(2,3)));
    tf_dest.setRotation(q);
    dest.orientation.x = quad.x();
    dest.orientation.y = quad.y();
    dest.orientation.z = quad.z();
    dest.orientation.w = quad.w();
    dest.position.x = source(0,3);
    dest.position.y = source(1,3);
    dest.position.z = source(2,3);
  }
  catch (...)
  {
    return false;
  }
  return true;
}

void PoseEstimationOnline::broadcast_thread_body()
{
  geometry_msgs::Pose pose;
  tf::Transform trans;
  boost::mutex mutex;
  while(nh.ok() && !stop_thread_)
  {
    visualization_msgs::MarkerArray markers;
    mutex.lock();
    int size = estimations_.size();
    float step = 1.0f / (size-1);
    for (int i=0; i<size; ++i)
    {
      convert (estimations_[i], pose, trans);
      tf_broadcaster_.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "/camera_rgb_optical_frame", names_[i].c_str()));
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/camera_rgb_optical_frame";
      marker.header.stamp = ros::Time();
      std::vector<std::string> vst;
      boost::split(vst, names_[i], boost::is_any_of("_"), boost::token_compress_on);
      marker.ns=vst.at(0).c_str();
      if (vst.size() >= 2)
        marker.id=std::stoi(vst.at(1));
      else
        marker.id=0;
      marker.scale.x=1;
      marker.scale.y=1;
      marker.scale.z=1;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      std::string mesh_path ("package://asus_scanner_models/" + vst.at(0) + "/" + vst.at(0) + ".stl");
      marker.mesh_resource = mesh_path.c_str();
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = pose;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f - i*step;
      marker.color.b = i*step;
      marker.color.a = 1.0f;
      marker.lifetime = ros::Duration(1);
      markers.markers.push_back(marker);
    }
    mutex.unlock();
    rviz_markers_pub_.publish(markers);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
}

void PoseEstimationOnline::start_broadcast_thread()
{
  if (!thread_started )
  {
    stop_thread_ = false;
    broadcast_thread_ = boost::thread(&PoseEstimationOnline::broadcast_thread_body, this);
    thread_started = true;
  }
  else
    ROS_WARN("Broadcaster thread is already started, won't start it again."); 
}
void PoseEstimationOnline::stop_broadcast_thread()
{
  if (thread_started)
  {
    stop_thread_=true;
    broadcast_thread_.join();
    stop_thread_=false;
    thread_started=false;
  }
  else
    ROS_WARN("Broadcast thread is not running, cannot stop it.");
}

//wrapper function to grab a cloud
bool PoseEstimationOnline::acquire_scene (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr acquired)
{ 
  std::string acquire_scene_srv_name = nh.resolveName("/scene_filter_node/acquire_scene");
  scene_filter_node::acquire_scene acquire_srv;
  acquire_srv.request.save = "false";
  boost::this_thread::sleep (boost::posix_time::microseconds (300000));
  if ( !ros::service::call<scene_filter_node::acquire_scene>(acquire_scene_srv_name, acquire_srv))
  {
    ROS_ERROR("Acquire scene service failed!");
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromROSMsg (acquire_srv.response.cloud, *tmp);
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud (tmp);
  vg.setLeafSize(0.005f, 0.005f, 0.005f);
  vg.filter (*acquired);
  return true;
}

int PoseEstimationOnline::extract_queries()
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
  extract.setNegative(false);
  extract.setIndices(inliers);
  extract.filter(*table_);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_top (new pcl::PointCloud<pcl::PointXYZRGBA>);
  extract.setNegative(true);
  extract.filter(*table_top);
  //cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  nh.getParam("/pose_estimation_online/clustering_tolerance", cluster_tol);
  ec.setClusterTolerance(cluster_tol);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(table_top->points.size());
  ec.setInputCloud(table_top);
  ec.extract(cluster_indices);
  queries_.resize(cluster_indices.size());
  names_.resize(cluster_indices.size());
  int j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGBA> object;
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      object.points.push_back(table_top->points[*pit]);
    object.width = object.points.size();
    object.height = 1;
    object.is_dense = true;
    pcl::copyPointCloud(object,queries_[j]);
    //ROS_WARN("object %d -> points %d", j+1, (int)queries_[j].points.size()); //TODO
    ++j;
  }
  return queries_.size();
}

void PoseEstimationOnline::pose_estimate()
{
  if (thread_started)
    stop_broadcast_thread();
  estimator.setDatabase(db_path_);
  estimations_.resize(queries_.size());
  nh.getParam("/pose_estimation_online/object_calibration", object_calibration);
  nh.getParam("/pose_estimation_online/iterations_in_prog_bisection", iterations);
  nh.getParam("/pose_estimation_online/neighbors", k_neigh);
  estimator.setParam("progItera",iterations);
  estimator.setParam("kNeighbors",k_neigh);
  //ROS_WARN("query size %d",(int)estimations_.size()); //TODO 
  for (int i=0; i<queries_.size(); ++i)
  {
    //ROS_WARN("query %d ->points %d", i+1, (int)queries_[i].points.size());//TODO
    estimator.setQuery("object", queries_[i]);
    estimator.generateLists();
    estimator.refineCandidates();
    boost::shared_ptr<Candidate> est (new Candidate);
    estimator.getEstimation(est);
    std::string name;
    est->getName(name);
    std::vector<std::string> vst;
    boost::split (vst, name, boost::is_any_of("_"), boost::token_compress_on);
    names_[i] = vst.at(0);
    if (object_calibration)
      names_[i] = "object";
/*
    Eigen::Matrix4d T_kl0, T_icp, T_rot;
    estimator.getTableTransformation(T_kl0,lat);
    if (i>0)
    {
      //check if we have already estimated the same name
      int copy(1);
      std::string original_name = name;
      for(int j=0; j<i; ++j)
      {
        if (name.compare(names_[j]) == 0)
        {//they compare egual
          name = original_name + "_" + std::to_string(++copy);
        }
      }
    }

    //ROS_WARN("angle %d",angle);//TODO
    Eigen::AngleAxisd rot (lon, Eigen::Vector3d::UnitZ());
    T_rot <<   rot.matrix()(0,0), rot.matrix()(0,1), rot.matrix()(0,2), 0,
               rot.matrix()(1,0), rot.matrix()(1,1), rot.matrix()(1,2), 0,
               rot.matrix()(2,0), rot.matrix()(2,1), rot.matrix()(2,2), 0,
               0,                 0,                 0,                 1;
    */
    Eigen::Matrix4f T_icp;
    estimator.getEstimationTransformation(T_icp);
    estimations_[i] = T_icp;
      
    if (visualize_)
      estimator.viewEstimation();
  }
  //first check if we have more copy of the same name in names_
  for (int i=0; i<names_.size(); ++i)
  {
    int count(1);
    string name_original = names_[i];
    if (i>0)
    {
      for (int j=0; j<i; ++j)
      {
        if (names_[i].compare(names_[j]) == 0)
        { //i-th name is equal to j-th name
         names_[i] = name_original + "_" + std::to_string(++count);
        }
      }
    }
  }
  if (!thread_started && !broadcast_disabled)
    start_broadcast_thread();
}

bool PoseEstimationOnline::estimate_scene(dual_manipulation_shared::estimate::Request& req, dual_manipulation_shared::estimate::Response& res)
{
  if (acquire_scene(scene))
    if (extract_queries() >= 1)
    {
      if (req.visualize == true)
        visualize_=true;
      else
        visualize_=false;
      pose_estimate();
      if (!broadcast_disabled && !thread_started)
      {
        start_broadcast_thread();
      }
      for (int i=0; i<estimations_.size(); ++i)
      {
        geometry_msgs::Pose pose;
        tf::Transform trans;
        convert (estimations_[i], pose, trans);
        dual_manipulation_shared::pe poseEst;
        poseEst.pose = pose;
        poseEst.name = names_[i];
        poseEst.parent_frame = "/camera_rgb_optical_frame" ;
        res.estimated_poses.poses.push_back(poseEst);
      }
      return true;
    }
  return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation_online");
    PoseEstimationOnline node;
    boost::this_thread::sleep (boost::posix_time::seconds (3));
    ros::Rate rate(5); //go at 5 hz
    while (node.nh.ok())
    {
      node.nh.getParam("/pose_estimation_online/online_mode",node.online_mode);
      node.nh.getParam("/pose_estimation_online/disable_tf_broadcast",node.broadcast_disabled);
      if (node.broadcast_disabled && node.thread_started)
      {
        node.stop_broadcast_thread();
      }
      else if (!node.broadcast_disabled && !node.thread_started)
      {
        node.start_broadcast_thread();
      }
      if (node.online_mode)
      {//online, try to perform an estimation online (stil W.i.p. leave it disabled) TODO
        if (node.acquire_scene(node.scene) )
        {
          if (node.extract_queries() >= 1)
          {
            node.pose_estimate();
          }
        }
        ros::spinOnce();
      }
      else
      {//offline, let user call service when he wants an estimation
        ros::spinOnce();
        rate.sleep();
      }
    }
    return 0;
}
