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

///color cast to double
void convertUint8ToDouble(uint8_t b, double &d);
///color cast to double
void castColorToDouble(const PT &pt, double &r, double &g, double &b);
///Convert from RGB to XYZ colorspace
void Rgb2Xyz(double *x, double *y, double *z, double r, double g, double b)
{
	r = INVGAMMACORRECTION(r);
	g = INVGAMMACORRECTION(g);
	b = INVGAMMACORRECTION(b);
	*x = (double)(0.4123955889674142161*r + 0.3575834307637148171*g + 0.1804926473817015735*b);
	*y = (double)(0.2125862307855955516*r + 0.7151703037034108499*g + 0.07220049864333622685*b);
	*z = (double)(0.01929721549174694484*r + 0.1191838645808485318*g + 0.9504971251315797660*b);
}
void Xyz2Lab(num *L, num *a, num *b, num X, num Y, num Z)
{
	X /= WHITEPOINT_X;
	Y /= WHITEPOINT_Y;
	Z /= WHITEPOINT_Z;
	X = LABF(X);
	Y = LABF(Y);
	Z = LABF(Z);
	*L = 116*Y - 16;
	*a = 500*(X - Y);
	*b = 200*(Y - Z);
}
#define INVGAMMACORRECTION(t)	\
	(((t) <= 0.0404482362771076) ? \
	((t)/12.92) : pow(((t) + 0.055)/1.055, 2.4))
}
/** @brief XYZ color of the D65 white point */
#define WHITEPOINT_X	0.950456
#define WHITEPOINT_Y	1.0
#define WHITEPOINT_Z	1.088754
/**
 * @brief CIE L*a*b* f function (used to convert XYZ to L*a*b*)
 * http://en.wikipedia.org/wiki/Lab_color_space
 */
#define LABF(t)	\
	((t >= 8.85645167903563082e-3) ? \
	pow(t,0.333333333333333) : (841.0/108.0)*(t) + (4.0/29.0))
#endif

