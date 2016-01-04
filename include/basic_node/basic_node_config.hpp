#ifndef _BASIC_NODE_CONFIG_HPP_
#define _BASIC_NODE_CONFIG_HPP_

#include <common/modules_config.hpp>
#include <array>
#include <common/box.h>

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
    bool set(std::string key, Box value)
    {
        LOCK lock(mtx_config);
        size_t size =map_box.size();
        map_box[key] = value;
        if (size !=map_box.size()){
            //key did not exist
            map_box.erase(key);
            return false;
        }
        return true;
    }
    bool get(std::string key, Box& value)
    {
        LOCK lock(mtx_config);
        try
        {
            value = map_box.at(key);
        }
        catch (std::out_of_range)
        {
            return false;
        }
        return true;
    }
    protected:
    //parameters map of boxes
    std::unordered_map<std::string, Box> map_box;
};
}
#endif

