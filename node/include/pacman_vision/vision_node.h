#ifndef _INCL_NODE

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

//general utilities
#include <string>
#include <stdlib.h>
#include <boost/smart_ptr.hpp>

#include "pacman_vision/processor.h"
#include "pacman_vision/estimator.h"


class VisionNode
{
  public:
    VisionNode(ros::NodeHandle &nh);
  private:
    //node handle
    ros::NodeHandle nh;
    //shard_ptr to node handle
    NHPtr nh_ptr;
    
    //Scoped pointers of modules
    boost::scoped_ptr<Processor> processor_module; 
    boost::scoped_ptr<Estimator> estimator_module; 
};

#define _INCL_NODE
#endif
