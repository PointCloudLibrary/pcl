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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: distances.h 35984 2011-02-13 03:14:39Z rusu $
 *
 */
#ifndef PCL_DISTANCES_H_
#define PCL_DISTANCES_H_

#include <pcl/common/common.h>

namespace pcl
{
  /** \brief Get the shortest 3D segment between two 3D lines
    * \param line_a the coefficients of the first line (point, direction)
    * \param line_b the coefficients of the second line (point, direction)
    * \param pt1_seg the first point on the line segment
    * \param pt2_seg the second point on the line segment
    */
  void
  lineToLineSegment (const Eigen::VectorXf &line_a, const Eigen::VectorXf &line_b, 
                     Eigen::Vector4f &pt1_seg, Eigen::Vector4f &pt2_seg);

  /** \brief Get the square distance from a point to a line (represented by a point and a direction)
    * \param pt a point
    * \param line_pt a point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
    * \param line_dir the line direction
    */
  double inline
  sqrPointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
    return (line_dir.cross3 (line_pt - pt)).squaredNorm () / line_dir.squaredNorm ();
  }

  /** \brief Get the square distance from a point to a line (represented by a point and a direction)
    * \note This one is useful if one has to compute many distances to a fixed line, so the vector length can be pre-computed
    * \param pt a point
    * \param line_pt a point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
    * \param line_dir the line direction
    * \param sqr_length the squared norm of the line direction
    */
  double inline
  sqrPointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir, const double sqr_length)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
    return (line_dir.cross3 (line_pt - pt)).squaredNorm () / sqr_length;
  }
}

#endif  //#ifndef PCL_DISTANCES_H_

