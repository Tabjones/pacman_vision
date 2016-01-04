#ifndef _BASIC_NODE_CONFIG_HPP_
#define _BASIC_NODE_CONFIG_HPP_

#include <common/modules_config.hpp>
#include <array>

namespace pacv
{
class BasicConfig: public Config<BasicConfig>
{
    public:
    typedef std::shared_ptr<BasicConfig> Ptr;
    friend class Config<BasicConfig>;
    Box filter_limits;
    const std::array<std::string,8> valid_keys;
    BasicConfig(): filter_limits(-0.5, -0.5, 0.3, 0.5, 0.5, 2),
    valid_keys {{"cropping", "downsampling", "segmenting",
                 "publish_limits", "keep_organized", "downsampling_leaf_size",
                 "plane_tolerance", "filter_limits"}}
    {
        //create maps
        map_bool["cropping"] = false;
        map_bool["downsampling"] = false;
        map_bool["segmenting"] = false;
        map_bool["publish_limits"] = false;
        map_bool["keep_organized"] = false;
        map_double["downsampling_leaf_size"] = 0.01;
        map_double["plane_tolerance"] = 0.01;
        map_box["filter_limits"] = filter_limits;
    }
};
}
#endif

