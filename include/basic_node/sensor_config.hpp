#ifndef _SENSOR_CONFIG_HPP_
#define _SENSOR_CONFIG_HPP_

#include <common/modules_config.hpp>
#include <array>

namespace pacv
{
class SensorConfig: public Config<SensorConfig>
{
    public:
    typedef std::shared_ptr<SensorConfig> Ptr;
    friend class Config<SensorConfig>;
    const std::array<std::string,3> valid_keys;
    SensorConfig(): valid_keys {{"internal", "topic", "name"}}
    {
        //create maps
        map_bool["internal"] = false;
        map_string["topic"] = "/camera/depth_registered/points";
        map_string["name"] = "kinect2_optical_frame";
    }
};
}
#endif

