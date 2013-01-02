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

#include <pcl/common/intersections.h>
#include <pcl/console/print.h>

bool
pcl::lineWithLineIntersection (const Eigen::VectorXf &line_a, 
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
pcl::lineWithLineIntersection (const pcl::ModelCoefficients &line_a, 
                               const pcl::ModelCoefficients &line_b, 
                               Eigen::Vector4f &point, double sqr_eps)
{
  Eigen::VectorXf coeff1 = Eigen::VectorXf::Map (&line_a.values[0], line_a.values.size ());
  Eigen::VectorXf coeff2 = Eigen::VectorXf::Map (&line_b.values[0], line_b.values.size ());
  return (lineWithLineIntersection (coeff1, coeff2, point, sqr_eps));
}

bool 
pcl::planeWithPlaneIntersection (const Eigen::Vector4f &plane_a, 
                                 const Eigen::Vector4f &plane_b,
                                 Eigen::VectorXf &line,
                                 double angular_tolerance)
{
  //planes shouldn't be parallel
  double test_cosine = plane_a.head<3>().dot(plane_b.head<3>());
  double upper_limit = 1 + angular_tolerance;
  double lower_limit = 1 - angular_tolerance;

  if ((test_cosine < upper_limit) && (test_cosine > lower_limit))
  {
      PCL_ERROR ("Plane A and Plane B are Parallel");
      return (false);
  }

  if ((test_cosine > -upper_limit) && (test_cosine < -lower_limit))
  {
      PCL_ERROR ("Plane A and Plane B are Parallel");
      return (false);
  }

  Eigen::Vector4f line_direction = plane_a.cross3(plane_b);
  line_direction.normalized();

  //construct system of equations using lagrange multipliers with one objective function and two constraints
  Eigen::MatrixXf langegrange_coefs(5,5);
  langegrange_coefs << 2,0,0,plane_a[0],plane_b[0],  0,2,0,plane_a[1],plane_b[1],  0,0,2, plane_a[2], plane_b[2], plane_a[0], plane_a[1] , plane_a[2], 0,0, plane_b[0], plane_b[1], plane_b[2], 0,0;

  Eigen::VectorXf b;
  b.resize(5);
  b << 0, 0, 0, -plane_a[3], -plane_b[3];

  //solve for the lagrange Multipliers
  Eigen::VectorXf x;
  x.resize(5);
  x = langegrange_coefs.colPivHouseholderQr().solve(b);

  line.resize(6);
  line.head<3>() = x.head<3>(); // the x[3] and x[4] are the values of the lagrange multipliers and are neglected
  line[3] = line_direction[0];
  line[4] = line_direction[1];
  line[5] = line_direction[2];
  return true;
} 
