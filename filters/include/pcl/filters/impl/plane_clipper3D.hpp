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
 */

#ifndef PCL_FILTERS_IMPL_PLANE_CLIPPER3D_HPP
#define PCL_FILTERS_IMPL_PLANE_CLIPPER3D_HPP

#include "pcl/filters/plane_clipper3D.h"

template<typename PointT>
pcl::PlaneClipper3D<PointT>::PlaneClipper3D (Eigen::Vector4f plane_params)
: plane_params_ (plane_params)
{
}

template<typename PointT>
pcl::PlaneClipper3D<PointT>::~PlaneClipper3D () throw ()
{
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
  float dist1 = (plane_params_[0] * point1.x + plane_params_[1] * point1.y + plane_params_[2] * point1.z + plane_params_[3]);
  float dist2 = (plane_params_[0] * point2.x + plane_params_[1] * point2.y + plane_params_[2] * point2.z + plane_params_[3]);

  if (dist1 * dist2 > 0) // both on same side of the plane -> nothing to clip
    return (dist1 > 0); // true if both are on positive side, thus visible

  float lambda = dist2 / (dist2 - dist1);
  float lambda_1 = 1.0 - lambda;

  // get the plane intersecion
  PointT intersection;
  intersection.x = point1.x * lambda + point2.x * lambda_1;
  intersection.y = point1.y * lambda + point2.y * lambda_1;
  intersection.z = point1.z * lambda + point2.z * lambda_1;

  // point1 is visible, point2 not => point2 needs to be replaced by intersection
  if (dist1 >= 0)
    point2 = intersection;
  else
    point1 = intersection;

  return false;
}

/**
 * @todo Implement me
 */
template<typename PointT> void
pcl::PlaneClipper3D<PointT>::clipPlanarPolygon3D (std::vector<PointT>& polygon) const
{
  polygon.clear ();
}

template<typename PointT> void
pcl::PlaneClipper3D<PointT>::clipPointCloud3D (const pcl::PointCloud<PointT>& cloud_in, std::vector<int>& clipped, const std::vector<int>& indices) const
{
  if (indices.empty ())
  {
    clipped.reserve (cloud_in.size ());
    /*
#if 0
    Eigen::MatrixXf points = cloud_in.getMatrixXfMap (4, sizeof (PointT) / sizeof (float), offsetof(PointT,x) / sizeof (float));
    Eigen::VectorXf distances = plane_params_.transpose () * points;
    for (register unsigned rIdx = 0; rIdx < cloud_in.size (); ++ rIdx)
    {
      if (distances (rIdx, 0) >= -plane_params_[3])
        clipped.push_back (rIdx);
    }
#else
    Eigen::Matrix4Xf points (4, cloud_in.size ());
    for (register unsigned rIdx = 0; rIdx < cloud_in.size (); ++ rIdx)
    {
      points (0, rIdx) = cloud_in[rIdx].x;
      points (1, rIdx) = cloud_in[rIdx].y;
      points (2, rIdx) = cloud_in[rIdx].z;
      points (3, rIdx) = 1;
    }
    Eigen::VectorXf distances = plane_params_.transpose () * points;
    for (register unsigned rIdx = 0; rIdx < cloud_in.size (); ++ rIdx)
    {
      if (distances (rIdx, 0) >= 0)
        clipped.push_back (rIdx);
    }

#endif

    //cout << "points   : " << points.rows () << " x " << points.cols () << " * " << plane_params_.transpose ().rows () << " x " << plane_params_.transpose ().cols () << endl;

    //cout << "distances: " << distances.rows () << " x " << distances.cols () << endl;
    /*/
    for (register unsigned pIdx = 0; pIdx < cloud_in.size (); ++pIdx)
    {
      if (clipPoint3D (cloud_in[pIdx]))
        clipped.push_back (pIdx);
    }
    //*/
  }
}
#endif //PCL_FILTERS_IMPL_PLANE_CLIPPER3D_HPP