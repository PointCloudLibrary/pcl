#ifndef PCL_COMMON_NORMALS_H_
#define PCL_COMMON_NORMALS_H_

/**
  * \file pcl/common/normals.h
  * Define methods related to normals
  * \ingroup common
  */
#include <pcl/point_types.h>

namespace pcl
{
  /** \brief Align pn's normal with av so they point in the same direction */
  bool
  alignNormals (Eigen::Ref<Eigen::Vector3f> pn, const Eigen::Ref<const Eigen::Vector3f> &av);

  /** \brief Check if two normals are within a tolerance
    * \param[in] n1 First normal.
    * \param[in] n2 Second normal.
    * \param[in] angle_threshold The angle threshold in radians.
    */
  bool
  checkNormalsEqual (const Eigen::Vector3f &n1, const Eigen::Vector3f &n2, const double &angle_threshold);


}

#include <pcl/common/impl/normals.hpp>

#endif  // PCL_COMMON_NORMALS_H_
