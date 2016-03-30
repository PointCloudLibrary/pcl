/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2012, Willow Garage, Inc.
*  Copyright (c) 2012-, Open Perception, Inc.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder(s) nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*
*/

#ifndef PCL_LRF_UTILS_H_
#define PCL_LRF_UTILS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl
{

  /** \brief Project point on the plane defined by plane_point and plane_normal.
  * 
  * \param[in] point point to project
  * \param[in] indices of normals used to disambiguate 
  * \param[in] normal to disambiguate. normal is modified by the function.
  * \ingroup features
  */
  void
    projectPointOnPlane (
    Eigen::Vector3f const &point,
    Eigen::Vector3f const &plane_point,
    Eigen::Vector3f const &plane_normal,
    Eigen::Vector3f &projected_point);


  /** \brief Find the unit vector from axis_origin, directed toward point and orthogonal to axis.
  * 
  * \param[in] axis input axis
  * \param[in] axis_origin point belonging to axis 
  * \param[in] point point input point not belonging to axis
  * \param[out] directed_ortho_axis unit vector from axis_origin, directed toward point and orthogonal to axis.
  * \ingroup features
  */
  void
    directedOrthogonalAxis (
    Eigen::Vector3f const &axis,
    Eigen::Vector3f const &axis_origin,
    Eigen::Vector3f const &point,
    Eigen::Vector3f &directed_ortho_axis);


  /** \brief Find the angle between unit vectors v1 and v2.
  * 
  * \param[in] v1 first unit vector
  * \param[in] v1 second unit vector
  * \param[in] axis axis orthogonal to v1 and v2. Sign of axis defines the sign of returned angle
  * \return angle between unit vectors v1 and v2
  * \ingroup features
  */
  float
    getAngleBetweenUnitVectors (
    Eigen::Vector3f const &v1,
    Eigen::Vector3f const &v2,
    Eigen::Vector3f const &axis);

  /** \brief Define a random unit vector orthogonal to axis.
  * 
  * \param[in] axis input axis
  * \param[out] rand_ortho_axis random unit vector orthogonal to axis
  * \ingroup features
  */
  void
    randomOrthogonalAxis (
    Eigen::Vector3f const &axis,
    Eigen::Vector3f &rand_ortho_axis);

  /** \brief Find the plane that best fits points in the least-square sense. Use SVD decomposition.
  * 
  * \param[in] points set of points to fit
  * \param[out] centroid point belonging to the fitted plane and centroid of points
  * \param[out] plane_normal normal of the fitted plane
  * \ingroup features
  */
  void
    planeFitting (
    Eigen::Matrix<float, Eigen::Dynamic, 3> const &points,
    Eigen::Vector3f &centroid,
    Eigen::Vector3f &plane_normal);


  /** \brief Disambiguate normal sign as described in:
  * 
  * A. Petrelli, L. Di Stefano, "A repeatable and efficient canonical reference for surface matching", 3DimPVT, 2012
  * A. Petrelli, L. Di Stefano, "On the repeatability of the local reference frame for partial shape matching", 13th International Conference on Computer Vision (ICCV), 2011
  * \param[in] normal_cloud input cloud of normals
  * \param[in] normal_indices indices of normals used to disambiguate 
  * \param[in] normal normal to disambiguate. normal is modified by the function.
  * \return false if normal_indices does not contain any valid normal.
  * \ingroup features
  */
  template<typename PointNT> bool
    normalDisambiguation (
    pcl::PointCloud<PointNT> const &normal_cloud,
    std::vector<int> const &normal_indices,
    Eigen::Vector3f &normal);


}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/lrf_utils.hpp>
#endif

#endif  //#ifndef PCL_LRF_UTILS_H_
