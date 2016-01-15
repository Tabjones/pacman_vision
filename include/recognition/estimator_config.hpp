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
    const std::array<std::string,8> valid_keys;
    EstimatorConfig():
    valid_keys {{"cluster_tol", "iterations", "neighbors",
                 "object_calibration", "always_success", "rmse_thresh",
                 "broadcast_object_tfs", "publish_object_markers"}}
    {
        //create maps
        map_bool["object_calibration"] = false;
        map_bool["always_success"] = true;
        map_bool["broadcast_object_tfs"] = true;
        map_bool["publish_object_markers"] = true;
        map_double["cluster_tol"] = 0.05;
        map_double["rmse_thresh"] = 0.005;
        map_int["iterations"] = 5;
        map_int["neighbors"] = 20;
    }
};
}
#endif

