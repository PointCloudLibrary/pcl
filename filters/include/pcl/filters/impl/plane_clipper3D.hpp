/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 */

#ifndef PCL_FILTERS_IMPL_PLANE_CLIPPER3D_HPP
#define PCL_FILTERS_IMPL_PLANE_CLIPPER3D_HPP

#include <pcl/filters/plane_clipper3D.h>

template<typename PointT>
pcl::PlaneClipper3D<PointT>::PlaneClipper3D (const Eigen::Vector4f& plane_params)
: plane_params_ (plane_params)
{
}

template<typename PointT> void
pcl::PlaneClipper3D<PointT>::setPlaneParameters (const Eigen::Vector4f& plane_params)
{
  plane_params_ = plane_params;
}

template<typename PointT> const Eigen::Vector4f&
pcl::PlaneClipper3D<PointT>::getPlaneParameters () const
{
  return plane_params_;
}

template<typename PointT> pcl::Clipper3D<PointT>*
pcl::PlaneClipper3D<PointT>::clone () const
{
  return new PlaneClipper3D<PointT> (plane_params_);
}

template<typename PointT> float
pcl::PlaneClipper3D<PointT>::getDistance (const PointT& point) const
{
  return (plane_params_[0] * point.x + plane_params_[1] * point.y + plane_params_[2] * point.z + plane_params_[3]);
}

template<typename PointT> bool
pcl::PlaneClipper3D<PointT>::clipPoint3D (const PointT& point) const
{
  return ((plane_params_[0] * point.x + plane_params_[1] * point.y + plane_params_[2] * point.z ) >= -plane_params_[3]);
}

/**
 * @attention untested code
 */
template<typename PointT> bool
pcl::PlaneClipper3D<PointT>::clipLineSegment3D (PointT& point1, PointT& point2) const
{
  float dist1 = getDistance (point1);
  float dist2 = getDistance (point2);

  if (dist1 * dist2 > 0) // both on same side of the plane -> nothing to clip
    return (dist1 > 0); // true if both are on positive side, thus visible

  float lambda = dist2 / (dist2 - dist1);

  // get the plane intersecion
  PointT intersection;
  intersection.x = (point1.x - point2.x) * lambda + point2.x;
  intersection.y = (point1.y - point2.y) * lambda + point2.y;
  intersection.z = (point1.z - point2.z) * lambda + point2.z;

  // point1 is visible, point2 not => point2 needs to be replaced by intersection
  if (dist1 >= 0)
    point2 = intersection;
  else
    point1 = intersection;

  return false;
}

/**
 * @attention untested code
 */
template<typename PointT> void
pcl::PlaneClipper3D<PointT>::clipPlanarPolygon3D (const std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon, std::vector<PointT, Eigen::aligned_allocator<PointT> >& clipped_polygon) const
{
  clipped_polygon.clear ();
  clipped_polygon.reserve (polygon.size ());

  // test for degenerated polygons
  if (polygon.size () < 3)
  {
    if (polygon.size () == 1)
    {
      // point outside clipping area ?
      if (clipPoint3D (polygon [0]))
        clipped_polygon.push_back (polygon [0]);
    }
    else if (polygon.size () == 2)
    {
      clipped_polygon.push_back (polygon [0]);
      clipped_polygon.push_back (polygon [1]);
      if (!clipLineSegment3D (clipped_polygon [0], clipped_polygon [1]))
        clipped_polygon.clear ();
    }
    return;
  }

  float previous_distance = getDistance (polygon [0]);

  if (previous_distance > 0)
    clipped_polygon.push_back (polygon [0]);

  typename std::vector<PointT, Eigen::aligned_allocator<PointT> >::const_iterator prev_it = polygon.begin ();

  for (typename std::vector<PointT, Eigen::aligned_allocator<PointT> >::const_iterator pIt = prev_it + 1; pIt != polygon.end (); prev_it = pIt++)
  {
    // if we intersect plane
    float distance = getDistance (*pIt);
    if (distance * previous_distance < 0)
    {
      float lambda = distance / (distance - previous_distance);

      PointT intersection;
      intersection.x = (prev_it->x - pIt->x) * lambda + pIt->x;
      intersection.y = (prev_it->y - pIt->y) * lambda + pIt->y;
      intersection.z = (prev_it->z - pIt->z) * lambda + pIt->z;

      clipped_polygon.push_back (intersection);
    }
    if (distance > 0)
      clipped_polygon.push_back (*pIt);

    previous_distance = distance;
  }
}

/**
 * @attention untested code
 */
template<typename PointT> void
pcl::PlaneClipper3D<PointT>::clipPlanarPolygon3D (std::vector<PointT, Eigen::aligned_allocator<PointT> > &polygon) const
{
  std::vector<PointT, Eigen::aligned_allocator<PointT> > clipped;
  clipPlanarPolygon3D (polygon, clipped);
  polygon = clipped;
}

// /ToDo: write fast version using eigen map and single matrix vector multiplication, that uses advantages of eigens SSE operations.
template<typename PointT> void
pcl::PlaneClipper3D<PointT>::clipPointCloud3D (const pcl::PointCloud<PointT>& cloud_in, Indices& clipped, const Indices& indices) const
{
  if (indices.empty ())
  {
    clipped.reserve (cloud_in.size ());

// #if 0
//     Eigen::MatrixXf points = cloud_in.getMatrixXfMap (4, sizeof (PointT) / sizeof (float), offsetof(PointT,x) / sizeof (float));
//     Eigen::VectorXf distances = plane_params_.transpose () * points;
//     for (unsigned rIdx = 0; rIdx < cloud_in.size (); ++ rIdx)
//     {
//       if (distances (rIdx, 0) >= -plane_params_[3])
//         clipped.push_back (rIdx);
//     }
// #else
//     Eigen::Matrix4Xf points (4, cloud_in.size ());
//     for (unsigned rIdx = 0; rIdx < cloud_in.size (); ++ rIdx)
//     {
//       points (0, rIdx) = cloud_in[rIdx].x;
//       points (1, rIdx) = cloud_in[rIdx].y;
//       points (2, rIdx) = cloud_in[rIdx].z;
//       points (3, rIdx) = 1;
//     }
//     Eigen::VectorXf distances = plane_params_.transpose () * points;
//     for (unsigned rIdx = 0; rIdx < cloud_in.size (); ++ rIdx)
//     {
//       if (distances (rIdx, 0) >= 0)
//         clipped.push_back (rIdx);
//     }
//
// #endif
//
//     //std::cout << "points   : " << points.rows () << " x " << points.cols () << " * " << plane_params_.transpose ().rows () << " x " << plane_params_.transpose ().cols () << std::endl;
//
//     //std::cout << "distances: " << distances.rows () << " x " << distances.cols () << std::endl;

    for (unsigned pIdx = 0; pIdx < cloud_in.size (); ++pIdx)
      if (clipPoint3D (cloud_in[pIdx]))
        clipped.push_back (pIdx);
  }
  else
  {
    for (const auto& index : indices)
      if (clipPoint3D (cloud_in[index]))
        clipped.push_back (index);
  }
}
#endif //PCL_FILTERS_IMPL_PLANE_CLIPPER3D_HPP
