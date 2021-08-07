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

#pragma once

#include <pcl/common/intersections.h>
#include <pcl/pcl_macros.h>
#include <pcl/console/print.h>


namespace pcl
{

bool
lineWithLineIntersection (const Eigen::VectorXf &line_a,
                          const Eigen::VectorXf &line_b,
                          Eigen::Vector4f &point, double sqr_eps)
{
  Eigen::Vector4f p1, p2;
  lineToLineSegment (line_a, line_b, p1, p2);

  // If the segment size is smaller than a pre-given epsilon...
  double sqr_dist = (p1 - p2).squaredNorm ();
  if (sqr_dist < sqr_eps)
  {
    point = p1;
    return (true);
  }
  point.setZero ();
  return (false);
}


bool
lineWithLineIntersection (const pcl::ModelCoefficients &line_a,
                          const pcl::ModelCoefficients &line_b,
                          Eigen::Vector4f &point, double sqr_eps)
{
  Eigen::VectorXf coeff1 = Eigen::VectorXf::Map (&line_a.values[0], line_a.values.size ());
  Eigen::VectorXf coeff2 = Eigen::VectorXf::Map (&line_b.values[0], line_b.values.size ());
  return (lineWithLineIntersection (coeff1, coeff2, point, sqr_eps));
}

template <typename Scalar> bool
planeWithPlaneIntersection (const Eigen::Matrix<Scalar, 4, 1> &plane_a,
                            const Eigen::Matrix<Scalar, 4, 1> &plane_b,
                            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &line,
                            double angular_tolerance)
{
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
  using Vector5 = Eigen::Matrix<Scalar, 5, 1>;
  using Matrix5 = Eigen::Matrix<Scalar, 5, 5>;

  // Normalize plane normals
  Vector3 plane_a_norm (plane_a.template head<3> ());
  Vector3 plane_b_norm (plane_b.template head<3> ());
  plane_a_norm.normalize ();
  plane_b_norm.normalize ();

  // Test if planes are parallel
  double test_cos = plane_a_norm.dot (plane_b_norm);
  double tolerance_cos = 1 - sin (std::abs (angular_tolerance));

  if (std::abs (test_cos) > tolerance_cos)
  {
      PCL_DEBUG ("Plane A and Plane B are parallel.\n");
      return (false);
  }

  Vector4 line_direction = plane_a.cross3 (plane_b);
  line_direction.normalized();

  // Construct system of equations using lagrange multipliers with one objective function and two constraints
  Matrix5 langrange_coefs;
  langrange_coefs << 2,0,0, plane_a[0], plane_b[0],
                     0,2,0, plane_a[1], plane_b[1],
                     0,0,2, plane_a[2], plane_b[2],
                     plane_a[0], plane_a[1], plane_a[2], 0, 0,
                     plane_b[0], plane_b[1], plane_b[2], 0, 0;

  Vector5 b;
  b << 0, 0, 0, -plane_a[3], -plane_b[3];

  line.resize(6);
  // Solve for the lagrange multipliers
  line.template head<3>() = langrange_coefs.colPivHouseholderQr().solve(b).template head<3> ();
  line.template tail<3>() = line_direction.template head<3>();
  return (true);
}

template <typename Scalar> bool
threePlanesIntersection (const Eigen::Matrix<Scalar, 4, 1> &plane_a,
                         const Eigen::Matrix<Scalar, 4, 1> &plane_b,
                         const Eigen::Matrix<Scalar, 4, 1> &plane_c,
                         Eigen::Matrix<Scalar, 3, 1> &intersection_point,
                         double determinant_tolerance)
{
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

  // TODO: Using Eigen::HyperPlanes is better to solve this problem
  // Check if some planes are parallel
  Matrix3 normals_in_lines;

  for (int i = 0; i < 3; i++)
  {
    normals_in_lines (i, 0) = plane_a[i];
    normals_in_lines (i, 1) = plane_b[i];
    normals_in_lines (i, 2) = plane_c[i];
  }

  Scalar determinant = normals_in_lines.determinant ();
  if (std::abs (determinant) < determinant_tolerance)
  {
    // det ~= 0
    PCL_DEBUG ("At least two planes are parallel.\n");
    return (false);
  }

  // Left part of the 3 equations
  Matrix3 left_member;

  for (int i = 0; i < 3; i++)
  {
    left_member (0, i) = plane_a[i];
    left_member (1, i) = plane_b[i];
    left_member (2, i) = plane_c[i];
  }

  // Right side of the 3 equations
  Vector3 right_member;
  right_member << -plane_a[3], -plane_b[3], -plane_c[3];

  // Solve the system
  intersection_point = left_member.fullPivLu ().solve (right_member);
  return (true);
}

} // namespace pcl

