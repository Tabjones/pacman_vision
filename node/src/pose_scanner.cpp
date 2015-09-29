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
  nh.param<std::string>("/pacman_vision/work_dir", work_dir_s, "~/PoseScanner");
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
    std::cout<<T_kt;
  }
  catch (...)
  {
    return (false);
  }
  return (true);
}

bool PoseScanner::saveTableTransform()
{
  try
  {
    if (!boost::filesystem::exists(session_dir) || !boost::filesystem::is_directory(session_dir))
      boost::filesystem::create_directory(session_dir);
    flann::Matrix<float> T_kt_flann (new float[4*4],4,4);
    for (size_t i=0; i<T_kt_flann.rows; ++i)
      for (size_t j=0; j<T_kt_flann.cols; ++j)
        T_kt_flann[i][j] = T_kt(i,j);
    flann::save_to_file(T_kt_flann, session_dir.string()+"/T_kt.h5", "TurnTable with respect to Kinect");
  }
  catch (...)
  {
    return (false);
  }
  return (true);
}

bool PoseScanner::savePoses()
{
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
    if (msg->header.frame_id.compare(sensor) |= 0)
    {
      //TODO transform point back into sensor frame
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
  //TODO
}

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
