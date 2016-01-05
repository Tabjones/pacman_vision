#ifndef _ESTIMATOR_CONFIG_HPP_
#define _ESTIMATOR_CONFIG_HPP_

#include <common/modules_config.hpp>
#include <array>

namespace pacv
{
class EstimatorConfig: public Config<EstimatorConfig>
{
    public:
    typedef std::shared_ptr<EstimatorConfig> Ptr;
    friend class Config<EstimatorConfig>;
    const std::array<std::string,4> valid_keys;
    EstimatorConfig():
    valid_keys {{"cluster_tol", "iterations", "neighbors",
                 "object_calibration"}}
    {
        //create maps
        map_bool["object_calibration"] = false;
        map_double["cluster_tol"] = 0.05;
        map_int["iterations"] = 5;
        map_int["neighbors"] = 20;
    }
};
}
#endif

