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
#ifndef _COMMON_PCL_H_
#define _COMMON_PCL_H_
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>

#include <common/box.h>
/*! \file common_pcl.h
    \brief Bunch of global functions and typedefs from pcl library, used within Pacman Vision
*/

namespace pacv
{
///Convenient PointCloud Typedefs
typedef pcl::PointXYZRGB PT; ///< Default point type.
typedef pcl::PointCloud<pcl::PointXYZRGB> PTC; ///< Default point cloud with default point type.

typedef pcl::PointXYZ PX; ///< Point type without color
typedef pcl::PointCloud<pcl::PointXYZ> PXC; ///< Point cloud with PX type

typedef pcl::PointNormal PN; ///< Point normal type
typedef pcl::PointCloud<pcl::PointNormal> PNC; ///< Point normal cloud

typedef pcl::Normal NT; ///< Point type with only normals
typedef pcl::PointCloud<pcl::Normal> NTC; ///< Normal cloud


/*! \brief Crop a source point cloud into dest,  where cropbox is defined by the Box lim.
 *
 * \param[in] source Source Point Cloud.
 * \param[out] dest Destination Point Cloud, can be empty pointer.
 * \param[in] lim Box object defining the cropbox limits.
 * \param[in] remove_inside Optionally  remove whats  inside  the  box, rather  than  keeping it.
 * \param[in] trans Optionally transform the box with this transformation before applying the crop.
 * \param[in] keep_organized Optionally keep the point cloud structure organized.
 * \note dest pointer gets modified or initialized if empty.
 */
void crop_a_box(const PTC::ConstPtr source, PTC::Ptr &dest, const Box lim,
        const bool remove_inside=false,
        const Eigen::Matrix4f& trans=Eigen::Matrix4f::Identity(),
        const bool keep_organized=false);

void crop_a_box(const PXC::ConstPtr source, PXC::Ptr &dest, const Box lim,
        const bool remove_inside=false,
        const Eigen::Matrix4f& trans=Eigen::Matrix4f::Identity(),
        const bool keep_organized=false);

void convertUint8ToDouble(uint8_t b, double &d);
void castColorToDouble(const PT &pt, double &r, double &g, double &b);
}
#endif

