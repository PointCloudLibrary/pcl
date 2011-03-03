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
 * $Id: intensity_gradient.hpp 35830 2011-02-08 06:18:23Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_INTENSITY_GRADIENT_H_
#define PCL_FEATURES_IMPL_INTENSITY_GRADIENT_H_

#include "pcl/features/intensity_gradient.h"

template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::IntensityGradientEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    PointOutT &p_out = output.points[idx];

    if (!searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists))
    {
      p_out.gradient[0] = p_out.gradient[1] = p_out.gradient[2] = std::numeric_limits<float>::quiet_NaN ();
      continue;
    }

    Eigen::Vector4f centroid;
    compute3DCentroid (*surface_, nn_indices, centroid);

    Eigen::Vector3f normal = Eigen::Vector3f::Map (normals_->points[idx].normal);
    Eigen::Vector3f gradient;
    computePointIntensityGradient (*surface_, nn_indices, centroid.head<3> (), normal, gradient);
   
    p_out.gradient[0] = gradient[0];
    p_out.gradient[1] = gradient[1];
    p_out.gradient[2] = gradient[2];
   
  }
}

template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::IntensityGradientEstimation <PointInT, PointNT, PointOutT>::computePointIntensityGradient (
  const pcl::PointCloud <PointInT> &cloud, const std::vector <int> &indices, 
  const Eigen::Vector3f &point, const Eigen::Vector3f &normal, Eigen::Vector3f &gradient)
{
  if (indices.size () < 3)
  { 
    gradient[0] = gradient[1] = gradient[2] = std::numeric_limits<float>::quiet_NaN ();
    return;
  }

  Eigen::Matrix3f A = Eigen::Matrix3f::Zero ();
  Eigen::Vector3f b = Eigen::Vector3f::Zero ();

  for (size_t i_point = 0; i_point < indices.size (); ++i_point)
  {
    PointInT p = cloud.points[indices[i_point]];
    if (!pcl_isfinite (p.x) || 
        !pcl_isfinite (p.y) || 
        !pcl_isfinite (p.z) || 
        !pcl_isfinite (p.intensity))
      continue;

    p.x -= point[0];
    p.y -= point[1];
    p.z -= point[2];

    A (0, 0) += p.x*p.x;
    A (0, 1) += p.x*p.y;
    A (0, 2) += p.x*p.z;

    A (1, 1) += p.y*p.y;
    A (1, 2) += p.y*p.z;

    A (2, 2) += p.z*p.z;
    
    b[0] += p.x * p.intensity;
    b[1] += p.y * p.intensity;
    b[2] += p.z * p.intensity;
  }
  // Fill in the lower triangle of A
  A (1, 0) = A (0, 1);
  A (2, 0) = A (0, 2);
  A (2, 1) = A (1, 2);

  // Fit a hyperplane to the data 
  Eigen::Vector3f x = A.colPivHouseholderQr ().solve (b);

  // Project the gradient vector, x, onto the tangent plane
  gradient = (Eigen::Matrix3f::Identity () - normal*normal.transpose ()) * x;
}


#define PCL_INSTANTIATE_IntensityGradientEstimation(InT,NT,OutT) template class pcl::IntensityGradientEstimation<InT,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_INTENSITY_GRADIENT_H_ 
