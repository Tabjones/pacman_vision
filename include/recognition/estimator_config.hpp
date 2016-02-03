// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
    const std::array<std::string,9> valid_keys;
    EstimatorConfig():
    valid_keys {{"running", "cluster_tol", "iterations", "neighbors",
                 "object_calibration", "always_success", "rmse_thresh",
                 "broadcast_tf", "publish_markers"}}
    {
        //create maps
        map_bool["running"] = false;
        map_bool["object_calibration"] = false;
        map_bool["always_success"] = true;
        map_bool["broadcast_tf"] = true;
        map_bool["publish_markers"] = true;
        map_double["cluster_tol"] = 0.05;
        map_double["rmse_thresh"] = 0.005;
        map_int["iterations"] = 5;
        map_int["neighbors"] = 20;
    }
};
}
#endif

