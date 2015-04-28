#ifndef _INCL_NODE

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include "pacman_vision/pacman_visionConfig.h"

//general utilities
#include <string>
#include <stdlib.h>
#include <boost/smart_ptr.hpp>

#include "pacman_vision/processor.h"
#include "pacman_vision/estimator.h"


class VisionNode
{
  public:
    VisionNode();
    void check_modules();
    //node handle
    ros::NodeHandle nh;
  private:
    //bools to control modules
    bool en_processor, en_estimator, en_tracker;

    //Scoped pointers of modules
    boost::shared_ptr<Processor> processor_module; 
    boost::shared_ptr<Estimator> estimator_module; 

    //Dynamic Reconfigure//
    //Server
    dynamic_reconfigure::Server<pacman_vision::pacman_visionConfig> dyn_srv;
    //Callback
    void cb_reconfigure(pacman_vision::pacman_visionConfig &config, uint32_t level);
};

#define _INCL_NODE
#endif
