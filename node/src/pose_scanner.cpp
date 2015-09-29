#include <pacman_vision/pose_scanner.h>

PoseScanner::PoseScanner(ros::NodeHandle &n, boost::shared_ptr<Storage> &stor)
{
  this->scene.reset(new PC);
  this->nh = ros::NodeHandle(n, "pose_scanner");
  this->queue_ptr.reset(new ros::CallbackQueue);
  this->nh.setCallbackQueue(&(*this->queue_ptr));
  this->storage = stor;
  this->srv_acquire = nh.advertiseService("acquire", &PoseScanner::cb_acquire, this);
  nh.param<int>("/pacman_vision/table_pass", table_pass, 10);
  std::string work_dir_s;
  nh.param<std::string>("/pacman_vision/work_dir", work_dir_s, "~/PoseScanner");
  work_dir = work_dir_s;
  timestamp = boost::posix_time::second_clock::local_time();
  session_dir = (work_dir.string() + "/Session_" + to_simple_string(timestamp) + "/");
  sub_clicked = nh.subscribe(nh.resolveName("/clicked_point"), 1, &PoseScanner::cb_clicked, this);
}

void PoseScanner::cb_clicked (const geometry_msgs::PointStamped::ConstPtr& msg)
{
  float x,y,z;
  x = msgs->point.x;
  y = msgs->point.y;
  z = msgs->point.z;
  //TODO add normal estimation
}
bool PoseScanner::cb_acquire(pacman_vision_comm::acquire::Request& req, pacman_vision_comm::acquire::Response& res)
{
}

bool PoseScanner::set_turn_table_pos(float pos)
{
}

float PoseScanner::get_turn_table_pos()
{
}

void PoseScanner::spin_once()
{
  //process this module callbacks
  this->queue_ptr->callAvailable(ros::WallDuration(0));
}
