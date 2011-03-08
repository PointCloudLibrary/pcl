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
 *
 * $Id: boundary.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_BOUNDARY_H_
#define PCL_FEATURES_IMPL_BOUNDARY_H_

#include "pcl/features/boundary.h"
#include <cfloat>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
  pcl::BoundaryEstimation<PointInT, PointNT, PointOutT>::isBoundaryPoint (
      const pcl::PointCloud<PointInT> &cloud, int q_idx, 
      const std::vector<int> &indices, 
      const Eigen::Vector3f &u, const Eigen::Vector3f &v, 
      float angle_threshold)
{
  return (isBoundaryPoint (cloud, cloud.points[q_idx], indices, u, v, angle_threshold));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
  pcl::BoundaryEstimation<PointInT, PointNT, PointOutT>::isBoundaryPoint (
      const pcl::PointCloud<PointInT> &cloud, const PointInT &q_point, 
      const std::vector<int> &indices, 
      const Eigen::Vector3f &u, const Eigen::Vector3f &v, 
      float angle_threshold)
{
  if (indices.size () < 3)
    return (false);
  float uvn_nn[2];
  Eigen::Vector3f delta;
  delta.setZero ();
  // Compute the angles between each neighboring point and the query point itself
  std::vector<float> angles;
  angles.reserve (indices.size ());
  for (size_t i = 0; i < indices.size (); ++i)
  {
    delta[0] = cloud.points[indices[i]].x - q_point.x;
    delta[1] = cloud.points[indices[i]].y - q_point.y;
    delta[2] = cloud.points[indices[i]].z - q_point.z;

    uvn_nn[0] = u.dot (delta);
    uvn_nn[1] = v.dot (delta);

    if (uvn_nn[0] == 0 && uvn_nn[1] == 0)
      continue;
    angles.push_back (atan2 (uvn_nn[1], uvn_nn[0])); // the angles are fine between -PI and PI too
  }
  std::sort (angles.begin (), angles.end ());

  // Compute the maximal angle difference between two consecutive angles
  float max_dif = FLT_MIN, dif;
  for (size_t i = 0; i < angles.size () - 1; ++i)
  {
    dif = angles[i + 1] - angles[i];
    if (max_dif < dif)
      max_dif = dif;
  }
  // Get the angle difference between the last and the first
  dif = 2 * M_PI - angles[angles.size () - 1] + angles[0];
  if (max_dif < dif)
    max_dif = dif;

  // Check results
  if (max_dif > angle_threshold)
    return (true);
  else
    return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
  pcl::BoundaryEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    ROS_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    ROS_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  Eigen::Vector3f u, v;

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Obtain a coordinate system on the least-squares plane
    getCoordinateSystemOnPlane (normals_->points[(*indices_)[idx]], u, v);

    // Estimate whether the point is lying on a boundary surface or not
    output.points[idx].boundary_point = isBoundaryPoint (*surface_, input_->points[(*indices_)[idx]], nn_indices, u, v, angle_threshold_);
  }
}

#define PCL_INSTANTIATE_BoundaryEstimation(PointInT,PointNT,PointOutT) template class pcl::BoundaryEstimation<PointInT, PointNT, PointOutT>;

#endif    // PCL_FEATURES_IMPL_BOUNDARY_H_ 

