#include "pacman_vision/vision_node.h"

VisionNode::VisionNode()
{
  this->nh = ros::NodeHandle("pacman_vision");
  this->dyn_srv.setCallback(boost::bind(&VisionNode::cb_reconfigure, this, _1, _2));
  en_processor = false;
  en_estimator = false;
  en_tracker = false;
}
void VisionNode::check_modules()
{
  //check if we want processor module and it is not started
  if (this->en_processor && !this->processor_module)
  {
    std::cout<<"created processor"<<std::endl;
    Processor p (this->nh);
    this->processor_module = boost::make_shared<Processor>(p);
  }
  else if (!this->en_processor && this->processor_module)
  {
    std::cout<<"killed processor"<<std::endl;
    this->processor_module.reset();
  }
}

void VisionNode::cb_reconfigure(pacman_vision::pacman_visionConfig &config, uint32_t level)
{
  ROS_WARN("Reconfigure Callback!");
  en_processor = config.enable_processor;
  en_estimator = config.enable_estimator;
  en_tracker = config.enable_tracker;
}

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "pacman_vision");
  VisionNode node;
  ros::Rate rate(50); //try to go at 50hz
  while (node.nh.ok())
  {
    std::cout<<"main"<<std::endl;
    node.check_modules();
    ros::spinOnce(); 
    rate.sleep();
  }
  return 0;
}
