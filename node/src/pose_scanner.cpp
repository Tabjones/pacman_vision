#include <pacman_vision/pose_scanner.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>

PoseScanner::PoseScanner(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor) : has_transform(false)
{
  this->scene.reset(new PC);
  this->nh = ros::NodeHandle(n, "pose_scanner");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->storage = stor;
  this->srv_acquire = nh.advertiseService("acquire", &PoseScanner::cb_acquire, this);
  this->srv_reload = nh.advertiseService("reload_transform", &PoseScanner::cb_reload, this);
  this->poses.reset(new std::vector<PC>);
  nh.param<int>("/pacman_vision/table_pass", table_pass, 10);
  nh.param<bool>("/pacman_vision/ignore_clicked_point",ignore_clicked_point , false);
  std::string work_dir_s;
  nh.param<std::string>("/pacman_vision/work_dir", work_dir_s, "PoseScanner");
  work_dir = work_dir_s;
  timestamp = boost::posix_time::second_clock::local_time();
  session_dir = (work_dir.string() + "/Session_" + to_simple_string(timestamp) + "/");
  sub_clicked = nh.subscribe(nh.resolveName("/clicked_point"), 1, &PoseScanner::cb_clicked, this);
  pub_poses = nh.advertise<PC> ("poses",1);
}

bool PoseScanner::computeTableTransform(PT pt, float nx, float ny, float nz)
{
  try
  {
    Eigen::Vector3f new_z(nx,ny,nz); //Z axis is the plane normal
    new_z.normalize();
    //Find a vector as close as possible to x-kinect but orthonormal to new_z
    Eigen::Vector3f kin_x(Eigen::Vector3f::UnitX());
    Eigen::Vector3f new_x;
    new_x = kin_x - (new_z*(new_z.dot(kin_x)));
    new_x.normalize();
    //New_y is new_z cross new_x;
    Eigen::Vector3f new_y;
    new_y = new_z.cross(new_x);
    new_y.normalize();

    //Rotation matrix of new table reference system with respect to kinect ref. frame
    //Translation that express table ref system centre with respect to kinect (the clicked point)
    //Compose the 4x4 rototraslation
    T_kt << new_x[0], new_y[0], new_z[0], pt.x,
      new_x[1], new_y[1], new_z[1], pt.y,
      new_x[2], new_y[2], new_z[2], pt.z,
      0,        0,        0,        1;
    T_tk = T_kt.inverse();
    has_transform = true;
  }
  catch (...)
  {
    return (false);
  }
  return (true);
}

bool PoseScanner::saveTableTransform()
{
  //update session dir if it was dynamic reconfigured
  session_dir = (work_dir.string() + "/Session_" + to_simple_string(timestamp) + "/");
  ROS_WARN("[session] %s",session_dir.c_str());
  try
  {
    if (!boost::filesystem::exists(session_dir) || !boost::filesystem::is_directory(session_dir))
      boost::filesystem::create_directory(session_dir);
    flann::Matrix<float> T_kt_flann (new float[4*4],4,4);
    for (size_t i=0; i<T_kt_flann.rows; ++i)
      for (size_t j=0; j<T_kt_flann.cols; ++j)
        T_kt_flann[i][j] = T_kt(i,j);
    flann::save_to_file(T_kt_flann, session_dir.string()+"T_kt.h5", "TurnTable with respect to Kinect");
  }
  catch (...)
  {
    return (false);
  }
  return (true);
}

bool PoseScanner::savePoses()
{
  //TODO
}


void PoseScanner::cb_clicked (const geometry_msgs::PointStamped::ConstPtr& msg)
{
  if(!ignore_clicked_point)
  {
    float nx,ny,nz;
    PT pt;
    //Get clicked point
    pt.x = msg->point.x;
    pt.y = msg->point.y;
    pt.z = msg->point.z;
    std::string sensor;
    this->storage->read_scene_processed(scene);
    this->storage->read_sensor_ref_frame(sensor);
    if (msg->header.frame_id.compare(sensor) != 0)
    {
      tf_listener.waitForTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(1.0));
      tf::StampedTransform t_msg;
      tf_listener.lookupTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), t_msg);
      Eigen::Matrix4f T;
      geometry_msgs::Pose pose;
      fromTF(t_msg, T, pose);
      PT pt_t;
      pt_t = pcl::transformPoint<PT>(pt, Eigen::Affine3f(T));
      pt.x = pt_t.x;
      pt.y = pt_t.y;
      pt.z = pt_t.z;
    }
    //compute a normal around its neighborhood (3cm)
    pcl::search::KdTree<PT> kdtree;
    std::vector<int> idx(scene->points.size());
    std::vector<float> dist(scene->points.size());
    kdtree.setInputCloud(scene);
    kdtree.radiusSearch(pt, 0.03, idx, dist);
    pcl::NormalEstimation<PT, pcl::Normal> ne;
    ne.setInputCloud(scene);
    ne.useSensorOriginAsViewPoint();
    float curv;
    ne.computePointNormal (*scene, idx, nx,ny,nz, curv);
    //Compute table transform
    if (!computeTableTransform(pt,nx,ny,nz))
    {
      ROS_WARN("[PoseScanner][%s]\tFailed to compute TableTransform, please click again!",__func__);
      return;
    }
    if (!saveTableTransform())
      ROS_WARN("[PoseScanner][%s]\tFailed to save TableTransform to disk",__func__);
  }
}

bool PoseScanner::cb_acquire(pacman_vision_comm::acquire::Request& req, pacman_vision_comm::acquire::Response& res)
{
  if (!has_transform)
  {
    ROS_ERROR("[PoseScanner][%s]\tNo table transform defined, please click and publish a point in rviz",__func__);
    return (false);
  }
  std::string name;
  if (req.objname.compare(0,1,"-") == 0) //check if we wanted flipped object (- in front)
  {
    name = req.objname.substr(1); //get all name except first character
    //TODO still need to implement this... if it is doable
  }
  else
    name = req.objname;
  PC::Ptr scan (new PC);
  pcl::PCDWriter writer;
  //Put table back at zero if it wasnt
  if(!move_turn_table_smoothly(0))
  {
    ROS_ERROR("[PoseScanner][%s]\tCannot move TurnTable correctly!",__func__);
    return false;
  }
  for (int angle=0; angle<360; angle+=table_pass)
  {
    if(!move_turn_table_smoothly(angle))
    {
      ROS_ERROR("[PoseScanner][%s]\tCannot move TurnTable correctly!",__func__);
      return false;
    }
  }
  return (true);
}
  /*
      //save scene on disk
      std::string scenename (scenepath.string() + "/" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (scenename.c_str(), *scene_);

      if (! acquire_scene (cloud_, false) )
      {
        ROS_ERROR("[posesScanner] Cannot acquire a scene!");
        return false;
      }
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGBA>);

      //Transforms used
      Eigen::Matrix4f T_l0k, T_kl0, T_l0li, T_lil0;

      if (lat==70)
      {
        T_kl0 = T_70;
        T_l0k = T_70.inverse();
      }
      if (lat==50)
      {
        T_kl0 = T_50;
        T_l0k = T_50.inverse();
      }
      if (lat==30)
      {
        T_kl0 = T_30;
        T_l0k = T_30.inverse();
      }
        //temporary transform to local frame (li) for easier cropping
      pcl::transformPointCloud(*cloud_, *temp, T_l0k);
      extract_object(lat, temp, object); //temp is in l0

      //save the cloud in local frame (li)
      Eigen::AngleAxisf rot (lon*D2R, Eigen::Vector3f::UnitZ() );
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_local (new pcl::PointCloud<pcl::PointXYZRGBA>);
      //init matrix as simple rotation around z
      T_lil0 << rot.matrix()(0,0), rot.matrix()(0,1), rot.matrix()(0,2), 0,
                rot.matrix()(1,0), rot.matrix()(1,1), rot.matrix()(1,2), 0,
                rot.matrix()(2,0), rot.matrix()(2,1), rot.matrix()(2,2), 0,
                0,                 0,                 0,                 1;
      T_l0li = T_lil0.inverse();
      pcl::transformPointCloud(*object, *cloud_local, T_lil0); //now cloud local is in li

      //also let user view the local pose
      _viewer_->updatePointCloud(cloud_local,"pose");
      _viewer_->spinOnce(300);

      //transform cloud back in sensor frame
      pcl::transformPointCloud(*cloud_local, *temp, T_l0li );
      pcl::transformPointCloud(*temp, *cloud_, T_kl0 );

      //get sensor information
      Eigen::Matrix4f T_sensor; //create one matrix for convinience
      T_sensor = T_kl0 * T_l0li;
      //extract quaternion of orientation from it
      Eigen::Matrix3f R_sensor;
      R_sensor = T_sensor.topLeftCorner(3,3);
      Eigen::Quaternionf Q_sensor (R_sensor); //init quaternion from rotation matrix
      Q_sensor.normalize();
      Eigen::Vector4f trasl_sensor (T_sensor(0,3), T_sensor(1,3), T_sensor(2,3), 1);
      //save sensor information in cloud local
      cloud_local->sensor_origin_ = trasl_sensor;
      cloud_local->sensor_orientation_ = Q_sensor;
      //now save local cloud
      std::string filename (localpath.string() + "/" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (filename.c_str(), *cloud_local);
      //publish local pose
      pub_poses_.publish(*cloud_local); //automatic conversion to rosmsg

      //save pose in sensor(kinect) on disk
      std::string kinectname (kinectpath.string() + "/" + name + "_" + std::to_string(lat) + "_" + std::to_string(lon) + ".pcd" );
      writer.writeBinaryCompressed (kinectname.c_str(), *cloud_);

      //move turntable
      for (int t=1; t<=lon_pass; ++t)
      {//for table steps: 1 degree
        if (lon+t >= 360)
          break; //skip last step
        if(!set_turnTable_pos(lon+t) )
        {
          ROS_ERROR("[posesScanner] turnTable communication failed!");
          return false;
        }
      }
    }
  }//end of acquisitions
  _viewer_->removePointCloud("pose");
  _viewer_->removeCoordinateSystem();
  _viewer_->addText("DO NOT CLOSE THE VIEWER!!\nNODE WILL NOT FUNCTION PROPERLY WITHOUT THE VIEWER", 200,200,18,250,150,150,"text");
  _viewer_->spinOnce(300);
  float cur_pos = get_turnTable_pos();
  if (cur_pos != 0)
  {
    float step = -cur_pos/360;
    for (int i=1; i<=360; i++)
    {
      if(!set_turnTable_pos(cur_pos + step*i) )
      {
        ROS_ERROR("[posesScanner] turnTable communication failed!");
        return false;
      }
    }
  }
  return true;
    //TODO
}
*/

bool PoseScanner::set_turn_table_pos(float pos)
{
  std::string srv_name = nh.resolveName("/turn_table_interface/set_pos");
  turn_table_interface::setPos srv;
  srv.request.position = pos;
  if (!ros::service::call<turn_table_interface::setPos>(srv_name, srv))
  {
    ROS_ERROR("[PoseScanner][%s]\tService for setting turn table position failed!",__func__);
    return (false);
  }
  return (true);
}

float PoseScanner::get_turn_table_pos()
{
  std::string srv_name = nh.resolveName("/turn_table_interface/get_pos");
  turn_table_interface::getPos srv;
  if (!ros::service::call<turn_table_interface::getPos>(srv_name, srv))
  {
    ROS_ERROR("[PoseScanner][%s]\tService for getting turn table position failed!",__func__);
    return (-1);
  }
  return (srv.response.current_pos);
}

bool PoseScanner::move_turn_table_smoothly(float pos)
{
  float current_pos = get_turn_table_pos();
  int count (0);
  while (current_pos > pos +1 || current_pos < pos -1)
  {
    int steps = (int)(pos - current_pos);
    if (steps >= 0)
    {
      //move forward
      for (int s=0; s < steps; ++s)
      {
        if (!set_turn_table_pos(current_pos + s + 1))
          return (false);
        boost::this_thread::sleep (boost::posix_time::milliseconds(20));
      }
    }
    else if (steps < 0)
    {
      //move backwards
      for (int s=0; s > steps; --s)
      {
        if (!set_turn_table_pos(current_pos + s - 1))
          return (false);
        boost::this_thread::sleep (boost::posix_time::milliseconds(20));
      }
    }
    //update position
    boost::this_thread::sleep (boost::posix_time::milliseconds(100));
    current_pos = get_turn_table_pos();
    if (++count > 5) //5 tries are enough
      return (false);
  }
  return (true);
}

bool PoseScanner::cb_reload(pacman_vision_comm::reload_transform::Request& req, pacman_vision_comm::reload_transform::Response& res)
{
  boost::filesystem::path file (req.path);
  if (boost::filesystem::exists(file) && boost::filesystem::is_regular_file(file))
  {
    try
    {
      flann::Matrix<float> trans;
      flann::load_from_file(trans, file.string(), "TurnTable with respect to Kinect");
      for (size_t i=0; i<trans.rows; ++i)
        for (size_t j=0; j<trans.cols; ++j)
          T_kt(i,j) = trans[i][j];
      T_tk = T_kt.inverse();
      has_transform = true;
    }
    catch (...)
    {
      ROS_ERROR("[PoseScanner][%s]\tError loading transform!",__func__);
      return (false);
    }
  }
  else
  {
    ROS_ERROR("[PoseScanner][%s]\tSpecified path does not name a file or does not exists!",__func__);
    return (false);
  }
  return (true);
}

void PoseScanner::spin_once()
{
  //update session dir if it was dynamic reconfigured
  session_dir = (work_dir.string() + "/Session_" + to_simple_string(timestamp) + "/");
  if (has_transform)
  {
    tf::Transform t_kt;
    geometry_msgs::Pose pose;
    fromEigen(T_kt, pose, t_kt);
    std::string sensor_ref_frame;
    this->storage->read_sensor_ref_frame(sensor_ref_frame);
    tf_table_trans.sendTransform(tf::StampedTransform(t_kt, ros::Time::now(), sensor_ref_frame.c_str(), "turn_table"));
  }
  //process this module callbacks
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}
