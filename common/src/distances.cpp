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
#include <pcl/common/distances.h>

void
pcl::lineToLineSegment (const Eigen::VectorXf &line_a, const Eigen::VectorXf &line_b, 
                        Eigen::Vector4f &pt1_seg, Eigen::Vector4f &pt2_seg)
{
  // point + direction = 2nd point
  Eigen::Vector4f p1 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f p2 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f dir1 = Eigen::Vector4f::Zero ();
  p1.head<3> () = line_a.head<3> ();
  dir1.head<3> () = line_a.segment<3> (3);
  p2 = p1 + dir1;

  // point + direction = 2nd point
  Eigen::Vector4f q1 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f q2 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f dir2 = Eigen::Vector4f::Zero ();
  q1.head<3> () = line_b.head<3> ();
  dir2.head<3> () = line_b.segment<3> (3);
  q2 = q1 + dir2;

  // a = x2 - x1 = line_a[1] - line_a[0]
  Eigen::Vector4f u = dir1;
  // b = x4 - x3 = line_b[1] - line_b[0]
  Eigen::Vector4f v = dir2;
  // c = x2 - x3 = line_a[1] - line_b[0]
  Eigen::Vector4f w = p2 - q1;

  float a = u.dot (u);
  float b = u.dot (v);
  float c = v.dot (v);
  float d = u.dot (w);
  float e = v.dot (w);
  float denominator = a*c - b*b;
  float sc, tc;
  // Compute the line parameters of the two closest points
  if (denominator < 1e-5)          // The lines are almost parallel
  {
    sc = 0.0;
    tc = (b > c ? d / b : e / c);  // Use the largest denominator
  }
  else
  {
    sc = (b*e - c*d) / denominator;
    tc = (a*e - b*d) / denominator;
  }
  // Get the closest points
  pt1_seg = Eigen::Vector4f::Zero ();
  pt1_seg = p2 + sc * u;

  pt2_seg = Eigen::Vector4f::Zero ();
  pt2_seg = q1 + tc * v;
}

