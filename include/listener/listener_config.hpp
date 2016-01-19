#ifndef _LISTENER_CONFIG_HPP_
#define _LISTENER_CONFIG_HPP_

#include <common/modules_config.hpp>
#include <array>

namespace pacv
{
class ListenerConfig: public Config<ListenerConfig>
{
    public:
    typedef std::shared_ptr<ListenerConfig> Ptr;
    friend class Config<ListenerConfig>;
    const std::array<std::string,10> valid_keys;
    ListenerConfig():
    valid_keys {{"listen_right_arm", "listen_left_arm", "listen_right_hand",
                 "listen_left_hand", "remove_right_arm", "remove_left_arm",
                 "remove_left_hand", "remove_right_hand", "publish_markers",
                 "geometry_scale"}}
    {
        //create maps
        map_bool["listen_right_arm"] = true;
        map_bool["listen_left_arm"] = true;
        map_bool["listen_right_hand"] = true;
        map_bool["listen_left_hand"] = true;
        map_bool["remove_right_arm"] = true;
        map_bool["remove_left_arm"] = true;
        map_bool["remove_right_hand"] = true;
        map_bool["remove_left_hand"] = true;
        map_bool["publish_markers"] = true;
        map_double["geometry_scale"] = 1.0;
    }
};
}
#endif

