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

}
#endif

