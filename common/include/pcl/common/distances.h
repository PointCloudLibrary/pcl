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
#ifndef PCL_DISTANCES_H_
#define PCL_DISTANCES_H_

#include <pcl/common/common.h>

/**
  * \file pcl/common/distances.h
  * Define standard C methods to do distance calculations
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  /** \brief Get the shortest 3D segment between two 3D lines
    * \param line_a the coefficients of the first line (point, direction)
    * \param line_b the coefficients of the second line (point, direction)
    * \param pt1_seg the first point on the line segment
    * \param pt2_seg the second point on the line segment
    * \ingroup common
    */
  PCL_EXPORTS void
  lineToLineSegment (const Eigen::VectorXf &line_a, const Eigen::VectorXf &line_b, 
                     Eigen::Vector4f &pt1_seg, Eigen::Vector4f &pt2_seg);

  /** \brief Get the square distance from a point to a line (represented by a point and a direction)
    * \param pt a point
    * \param line_pt a point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
    * \param line_dir the line direction
    * \ingroup common
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
    * \ingroup common
    */
  double inline
  sqrPointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir, const double sqr_length)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
    return (line_dir.cross3 (line_pt - pt)).squaredNorm () / sqr_length;
  }

  /** \brief Obtain the maximum segment in a given set of points, and return the minimum and maximum points.
    * \param[in] cloud the point cloud dataset
    * \param[out] pmin the coordinates of the "minimum" point in \a cloud (one end of the segment)
    * \param[out] pmax the coordinates of the "maximum" point in \a cloud (the other end of the segment)
    * \return the length of segment length
    * \ingroup common
    */
  template <typename PointT> double inline
  getMaxSegment (const pcl::PointCloud<PointT> &cloud, 
                 PointT &pmin, PointT &pmax)
  {
    double max_dist = std::numeric_limits<double>::min ();
    int i_min = -1, i_max = -1;

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      for (size_t j = i; j < cloud.points.size (); ++j)
      {
        // Compute the distance 
        double dist = (cloud.points[i].getVector4fMap () - 
                       cloud.points[j].getVector4fMap ()).squaredNorm ();
        if (dist <= max_dist)
          continue;

        max_dist = dist;
        i_min = i;
        i_max = j;
      }
    }

    if (i_min == -1 || i_max == -1)
      return (max_dist = std::numeric_limits<double>::min ());

    pmin = cloud.points[i_min];
    pmax = cloud.points[i_max];
    return (std::sqrt (max_dist));
  }
 
  /** \brief Obtain the maximum segment in a given set of points, and return the minimum and maximum points.
    * \param[in] cloud the point cloud dataset
    * \param[in] indices a set of point indices to use from \a cloud
    * \param[out] pmin the coordinates of the "minimum" point in \a cloud (one end of the segment)
    * \param[out] pmax the coordinates of the "maximum" point in \a cloud (the other end of the segment)
    * \return the length of segment length
    * \ingroup common
    */
  template <typename PointT> double inline
  getMaxSegment (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                 PointT &pmin, PointT &pmax)
  {
    double max_dist = std::numeric_limits<double>::min ();
    int i_min = -1, i_max = -1;

    for (size_t i = 0; i < indices.size (); ++i)
    {
      for (size_t j = i; j < indices.size (); ++j)
      {
        // Compute the distance 
        double dist = (cloud.points[indices[i]].getVector4fMap () - 
                       cloud.points[indices[j]].getVector4fMap ()).squaredNorm ();
        if (dist <= max_dist)
          continue;

        max_dist = dist;
        i_min = i;
        i_max = j;
      }
    }

    if (i_min == -1 || i_max == -1)
      return (max_dist = std::numeric_limits<double>::min ());

    pmin = cloud.points[indices[i_min]];
    pmax = cloud.points[indices[i_max]];
    return (std::sqrt (max_dist));
  }

  /** \brief Calculate the squared euclidean distance between the two given points.
    * \param[in] p1 the first point
    * \param[in] p2 the second point
    */
  template<typename PointType1, typename PointType2> inline float
  squaredEuclideanDistance (const PointType1& p1, const PointType2& p2)
  {
    float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
    return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
  }

  /** \brief Calculate the squared euclidean distance between the two given points.
    * \param[in] p1 the first point
    * \param[in] p2 the second point
    */
  template<> inline float
  squaredEuclideanDistance (const PointXY& p1, const PointXY& p2)
  {
    float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y;
    return (diff_x*diff_x + diff_y*diff_y);
  }

   /** \brief Calculate the euclidean distance between the two given points.
    * \param[in] p1 the first point
    * \param[in] p2 the second point
    */
  template<typename PointType1, typename PointType2> inline float
  euclideanDistance (const PointType1& p1, const PointType2& p2)
  {
    return (sqrtf (squaredEuclideanDistance (p1, p2)));
  }
}
/*@*/
#endif  //#ifndef PCL_DISTANCES_H_

