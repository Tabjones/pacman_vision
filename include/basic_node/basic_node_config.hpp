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
    const std::array<std::string,13> valid_keys;
    BasicConfig(): filter_limits(-0.5, -0.5, 0.3, 0.5, 0.5, 2),
    valid_keys {{"cropping", "downsampling", "segmenting", "outliers_filter", "color_filter",
                 "publish_limits", "keep_organized", "downsampling_leaf_size",
                 "plane_tolerance", "filter_limits", "outliers_mean_k", "outliers_std_mul",
                 "color_dist_thresh"}}
    {
        //create maps
        map_bool["cropping"] = false;
        map_bool["downsampling"] = false;
        map_bool["segmenting"] = false;
        map_bool["outliers_filter"] = false;
        map_bool["color_filter"] = false;
        map_bool["publish_limits"] = false;
        map_bool["keep_organized"] = false;
        map_double["downsampling_leaf_size"] = 0.01;
        map_int["outliers_mean_k"] = 30;
        map_double["plane_tolerance"] = 0.01;
        map_double["outliers_std_mul"] = 2.0;
        map_double["color_dist_thresh"] = 10.0;
        map_box["filter_limits"] = filter_limits;
    }
};
}
#endif

