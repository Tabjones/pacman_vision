#ifndef _MODULE_CONFIG_H_
#define _MODULE_CONFIG_H_

#include <string>
#include <pacman_vision/box.h>

struct BasicNodeConfig
{
    //filter parameters
    bool cropping, downsampling, keep_organized, segmenting;
    Box limits; //cropbox limits
    //publish filter limits and or plane model
    bool publish_limits; //, publish_plane;
    double downsampling_leaf_size, plane_tolerance;
    typedef std::shared_ptr<BasicNodeConfig> Ptr;
};

struct SensorProcessorConfig
{
    //Use the internal kinect2 processor, or a subscriber
    bool internal;
    //on which topic to listen if !internal
    std::string topic;
    //name of the internal processor
    std::string name;
    typedef std::shared_ptr<SensorProcessorConfig> Ptr;
};

struct EstimatorConfig
{
    bool spawned;
    //TODO
    typedef std::shared_ptr<EstimatorConfig> Ptr;
};

#endif

