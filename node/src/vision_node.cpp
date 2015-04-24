#include "pacman_vision/modules.h"
#include "pacman_vision/vision_node.h"

VisionNode::VisionNode(ros::NodeHandle &nh)
{
  this->nh = nh;
  nh_ptr = boost::make_shared<ros::NodeHandle> ( nh );
  processor_module.reset();
}

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "pacman_vision_node");
  ros::NodeHandle nodehandle("pacman_vision_node");
  VisionNode node(nodehandle);
  ros::Rate rate(50); //try to go at 50hz
  while (nodehandle.ok())
  {
    ros::spinOnce(); 
    rate.sleep();
  }
  return 0;
}
