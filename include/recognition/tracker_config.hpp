#ifndef _TRACKER_CONFIG_HPP_
#define _TRACKER_CONFIG_HPP_

#include <common/modules_config.hpp>
#include <array>

namespace pacv
{
class TrackerConfig: public Config<TrackerConfig>
{
    public:
    typedef std::shared_ptr<TrackerConfig> Ptr;
    friend class Config<TrackerConfig>;
    const std::array<std::string,3> valid_keys;
    TrackerConfig():
        //empty for now
    valid_keys {{"publish_bounding_box", "publish_markers", "broadcast_tf" }}
    {
        //create maps
        map_bool["publish_bounding_box"]= true;
        map_bool["publish_markers"]= true;
        map_bool["broadcast_tf"]= true;
    }
};
}
#endif

