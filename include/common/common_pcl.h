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

typedef pcl::PointXYZRGBNormal PNT;
typedef pcl::PointCloud<PNT> PNTC;

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
void castUint8ToDouble(uint8_t b, double &d);
///color cast to double
void castPCLColorToDouble(const PT &pt, double &r, double &g, double &b);

/**
 * \brief Convert from RGB to XYZ colorspace
 *      https://en.wikipedia.org/wiki/List_of_color_spaces_and_their_uses
 * \param[in] r input Red channel in 0-1 range
 * \param[in] g input Green channel in 0-1 range
 * \param[in] b input Blue channel in 0-1 range
 * \param[out] x output X channel of XYZ colorspace
 * \param[out] y output Y channel of XYZ colorspace
 * \param[out] z output Z channel of XYZ colorspace
 */
void rgb2Xyz(double r, double g, double b, double &x, double &y, double &z);

/**
 * \brief Convert from XYZ colorspace to (CIE)L*a*b* colorspace
 *      https://en.wikipedia.org/wiki/List_of_color_spaces_and_their_uses
 * \param[in] x input X channel.
 * \param[in] y input Y channel.
 * \param[in] z input Z channel.
 * \param[out] L output L(uminance) channel of CIEL*a*b* colorspace
 * \param[out] a output a channel of CIEL*a*b* colorspace
 * \param[out] b output b channel of CIEL*a*b* colorspace
 */
void xyz2Lab(double x, double y, double z, double &L, double &a,  double &b);

/**
 * @brief CIE L*a*b* f function (used to convert XYZ to L*a*b*)
 * http://en.wikipedia.org/wiki/Lab_color_space
 */
inline double labF(const double t);
/**
 * \brief Inverse gamma correction function for rgb channels
 */
inline double invGammaCorrection(const double t);

/** @brief XYZ color of the D65 white point */
#define WHITEPOINT_X	0.950456
#define WHITEPOINT_Y	1.0
#define WHITEPOINT_Z	1.088754

/**
 * \brief Convenient function to convert PCL(rgb) color to CIEL*a*b*
 * \param[in] pt Point whose color is to be converted
 * \param[out] L Corresponding L* channel of CIEL*a*b* space
 * \param[out] a Corresponding a* channel of CIEL*a*b* space
 * \param[out] b Corresponding b* channel of CIEL*a*b* space
 */
void convertPCLColorToCIELAB(const PT &pt, double &L, double &a, double &b);

/**
 * \brief Calculate the CIEDE2000 color difference from two CIEL*a*b* colors
 *  As in http://www.ece.rochester.edu/~gsharma/ciede2000/ciede2000noteCRNA.pdf
 *  First color is assumed to be the reference color, while the second is assumed
 *  to be the sample color to test.
 * \param[in] L1 input L channel of first color
 * \param[in] a1 input a channel of first color
 * \param[in] b1 input b channel of first color
 * \param[in] L2 input L channel of second color
 * \param[in] a2 input a channel of second color
 * \param[in] b2 input b channel of second color
 * \param[in] Kl optional luminance weight
 * \param[in] Kc optional color weight
 * \param[in] Kh optional hue weight
 * \param[in] verbose True to spam every possible value computed
 * \return The color difference between two colors
 */
double deltaE(const double L1, const double a1, const double b1,
              const double L2, const double a2, const double b2,
              const double Kl=1.0, const double Kc=1.0, const double Kh=1.0,
              const bool verbose=false);

/**
 * \brief Test delta E implementation with some pairs of input from Table one of
 * http://www.ece.rochester.edu/~gsharma/ciede2000/ciede2000noteCRNA.pdf
 */
void testDeltaE();

}//namespace
#endif
