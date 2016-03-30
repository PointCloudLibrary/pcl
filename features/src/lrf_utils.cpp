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
*/
#include <pcl/features/impl/lrf_utils.hpp>



void
  pcl::projectPointOnPlane (
  Eigen::Vector3f const &point,
  Eigen::Vector3f const &plane_point,
  Eigen::Vector3f const &plane_normal,
  Eigen::Vector3f &projected_point)
{
  float t;
  Eigen::Vector3f xo;

  xo = point - plane_point;
  t = plane_normal.dot (xo);

  projected_point = point - (t * plane_normal);
}


void
  pcl::directedOrthogonalAxis (
  Eigen::Vector3f const &axis,
  Eigen::Vector3f const &axis_origin,
  Eigen::Vector3f const &point,
  Eigen::Vector3f &directed_ortho_axis)
{
  Eigen::Vector3f projection;
  projectPointOnPlane (point, axis_origin, axis, projection);
  directed_ortho_axis = projection - axis_origin;

  directed_ortho_axis.normalize ();
}


float
  pcl::getAngleBetweenUnitVectors (
  Eigen::Vector3f const &v1,
  Eigen::Vector3f const &v2,
  Eigen::Vector3f const &axis)
{
  Eigen::Vector3f angle_orientation;
  angle_orientation = v1.cross (v2);
  float angle_radians = acosf (std::max (-1.0f, std::min (1.0f, v1.dot (v2))));

  angle_radians = angle_orientation.dot (axis) < 0.f ? (2 * static_cast<float> (M_PI) - angle_radians) : angle_radians;

  return (angle_radians);
}


void
  pcl::randomOrthogonalAxis (
  Eigen::Vector3f const &axis,
  Eigen::Vector3f &rand_ortho_axis)
{
  if (std::abs (axis.z ()) > 1E-8f)
  {
    rand_ortho_axis.x () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.y () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.z () = -(axis.x () * rand_ortho_axis.x () + axis.y () * rand_ortho_axis.y ()) / axis.z ();
  }
  else if (std::abs (axis.y ()) > 1E-8f)
  {
    rand_ortho_axis.x () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.z () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.y () = -(axis.x () * rand_ortho_axis.x () + axis.z () * rand_ortho_axis.z ()) / axis.y ();
  }
  else if (std::abs (axis.x ()) > 1E-8f)
  {
    rand_ortho_axis.y () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.z () = (static_cast<float> (rand ()) / static_cast<float> (RAND_MAX)) * 2.0f - 1.0f;
    rand_ortho_axis.x () = -(axis.y () * rand_ortho_axis.y () + axis.z () * rand_ortho_axis.z ()) / axis.x ();
  }

  rand_ortho_axis.normalize ();
}


void
  pcl::planeFitting (
  Eigen::Matrix<float, Eigen::Dynamic, 3> const &points,
  Eigen::Vector3f &centroid,
  Eigen::Vector3f &plane_normal)
{

  int n_points = static_cast<int> (points.rows ());
  if (n_points == 0)
  {
    return;
  }

  //find the center by averaging the points positions
  centroid.setZero ();

  for (int i = 0; i < n_points; ++i)
  {
    centroid += points.row (i);
  }

  centroid /= static_cast<float> (n_points);

  //copy points - average (center)
  Eigen::Matrix<float, Eigen::Dynamic, 3> A (n_points, 3);
  for (int i = 0; i < n_points; ++i)
  {
    A (i, 0) = points (i, 0) - centroid.x ();
    A (i, 1) = points (i, 1) - centroid.y ();
    A (i, 2) = points (i, 2) - centroid.z ();
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd (A, Eigen::ComputeFullV);
  plane_normal = svd.matrixV ().col (2);
}


#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
PCL_INSTANTIATE(normalDisambiguation, (pcl::Normal));
#else
PCL_INSTANTIATE(normalDisambiguation, PCL_NORMAL_POINT_TYPES);
#endif
#endif    // PCL_NO_PRECOMPILE

