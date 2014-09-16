/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_INTERSECTIONS_H_
#define PCL_INTERSECTIONS_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

/** 
  * \file pcl/common/intersections.h
  * Define line with line intersection functions
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  /** \brief Get the intersection of a two 3D lines in space as a 3D point
    * \param[in] line_a the coefficients of the first line (point, direction)
    * \param[in] line_b the coefficients of the second line (point, direction)
    * \param[out] point holder for the computed 3D point
    * \param[in] sqr_eps maximum allowable squared distance to the true solution
    * \ingroup common
    */
  PCL_EXPORTS inline bool
  lineWithLineIntersection (const Eigen::VectorXf &line_a, 
                            const Eigen::VectorXf &line_b, 
                            Eigen::Vector4f &point,
                            double sqr_eps = 1e-4);

  /** \brief Get the intersection of a two 3D lines in space as a 3D point
    * \param[in] line_a the coefficients of the first line (point, direction)
    * \param[in] line_b the coefficients of the second line (point, direction)
    * \param[out] point holder for the computed 3D point
    * \param[in] sqr_eps maximum allowable squared distance to the true solution
    * \ingroup common
    */

  PCL_EXPORTS inline bool
  lineWithLineIntersection (const pcl::ModelCoefficients &line_a, 
                            const pcl::ModelCoefficients &line_b, 
                            Eigen::Vector4f &point,
                            double sqr_eps = 1e-4);

  /** \brief Determine the line of intersection of two non-parallel planes using lagrange multipliers
    * \note Described in: "Intersection of Two Planes, John Krumm, Microsoft Research, Redmond, WA, USA"
    * \param[in] plane_a coefficients of plane A and plane B in the form ax + by + cz + d = 0
    * \param[in] plane_b coefficients of line where line.tail<3>() = direction vector and
    * line.head<3>() the point on the line clossest to (0, 0, 0)
    * \param[out] line the intersected line to be filled
    * \param[in] angular_tolerance tolerance in radians
    * \return true if succeeded/planes aren't parallel
    */
  PCL_EXPORTS template <typename Scalar> bool
  planeWithPlaneIntersection (const Eigen::Matrix<Scalar, 4, 1> &plane_a,
                              const Eigen::Matrix<Scalar, 4, 1> &plane_b,
                              Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &line,
                              double angular_tolerance = 0.1);

  PCL_EXPORTS inline bool
  planeWithPlaneIntersection (const Eigen::Vector4f &plane_a,
                              const Eigen::Vector4f &plane_b,
                              Eigen::VectorXf &line,
                              double angular_tolerance = 0.1)
  {
    return (planeWithPlaneIntersection<float> (plane_a, plane_b, line, angular_tolerance));
  }

  PCL_EXPORTS inline bool
  planeWithPlaneIntersection (const Eigen::Vector4d &plane_a,
                              const Eigen::Vector4d &plane_b,
                              Eigen::VectorXd &line,
                              double angular_tolerance = 0.1)
  {
    return (planeWithPlaneIntersection<double> (plane_a, plane_b, line, angular_tolerance));
  }

  /** \brief Determine the point of intersection of three non-parallel planes by solving the equations.
    * \note If using nearly parralel planes you can lower the determinant_tolerance value. This can
    * lead to inconsistent results.
    * If the three planes intersects in a line the point will be anywhere on the line.
    * \param[in] plane_a are the coefficients of the first plane in the form ax + by + cz + d = 0
    * \param[in] plane_b are the coefficients of the second plane
    * \param[in] plane_c are the coefficients of the third plane
    * \param[in] determinant_tolerance is a limit to determine whether planes are parallel or not
    * \param[out] intersection_point the three coordinates x, y, z of the intersection point
    * \return true if succeeded/planes aren't parallel
    */
  PCL_EXPORTS template <typename Scalar> bool
  threePlanesIntersection (const Eigen::Matrix<Scalar, 4, 1> &plane_a,
                           const Eigen::Matrix<Scalar, 4, 1> &plane_b,
                           const Eigen::Matrix<Scalar, 4, 1> &plane_c,
                           Eigen::Matrix<Scalar, 3, 1> &intersection_point,
                           double determinant_tolerance = 1e-6);


  PCL_EXPORTS inline bool
  threePlanesIntersection (const Eigen::Vector4f &plane_a,
                           const Eigen::Vector4f &plane_b,
                           const Eigen::Vector4f &plane_c,
                           Eigen::Vector3f &intersection_point,
                           double determinant_tolerance = 1e-6)
  {
    return (threePlanesIntersection<float> (plane_a, plane_b, plane_c,
                                            intersection_point, determinant_tolerance));
  }

  PCL_EXPORTS inline bool
  threePlanesIntersection (const Eigen::Vector4d &plane_a,
                           const Eigen::Vector4d &plane_b,
                           const Eigen::Vector4d &plane_c,
                           Eigen::Vector3d &intersection_point,
                           double determinant_tolerance = 1e-6)
  {
    return (threePlanesIntersection<double> (plane_a, plane_b, plane_c,
                                            intersection_point, determinant_tolerance));
  }

}
/*@}*/

#include <pcl/common/impl/intersections.hpp>

#endif  //#ifndef PCL_INTERSECTIONS_H_
