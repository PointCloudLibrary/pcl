/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_MOMENT_INVARIANTS_H_
#define PCL_FEATURES_IMPL_MOMENT_INVARIANTS_H_

#include <pcl/features/moment_invariants.h>
#include <pcl/common/centroid.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MomentInvariantsEstimation<PointInT, PointOutT>::computePointMomentInvariants (
      const pcl::PointCloud<PointInT> &cloud, const pcl::Indices &indices,
      float &j1, float &j2, float &j3)
{
  // Estimate the XYZ centroid
  compute3DCentroid (cloud, indices, xyz_centroid_);

  // Initialize the centralized moments
  float mu200 = 0, mu020 = 0, mu002 = 0, mu110 = 0, mu101 = 0, mu011  = 0;

  // Iterate over the nearest neighbors set
  for (const auto &index : indices)
  {
    // Demean the points
    temp_pt_[0] = cloud[index].x - xyz_centroid_[0];
    temp_pt_[1] = cloud[index].y - xyz_centroid_[1];
    temp_pt_[2] = cloud[index].z - xyz_centroid_[2];

    mu200 += temp_pt_[0] * temp_pt_[0];
    mu020 += temp_pt_[1] * temp_pt_[1];
    mu002 += temp_pt_[2] * temp_pt_[2];
    mu110 += temp_pt_[0] * temp_pt_[1];
    mu101 += temp_pt_[0] * temp_pt_[2];
    mu011 += temp_pt_[1] * temp_pt_[2];
  }

  // Save the moment invariants
  j1 = mu200             + mu020               + mu002;
  j2 = mu200*mu020       + mu200*mu002         + mu020*mu002       - mu110*mu110       - mu101*mu101       - mu011*mu011;
  j3 = mu200*mu020*mu002 + 2*mu110*mu101*mu011 - mu002*mu110*mu110 - mu020*mu101*mu101 - mu200*mu011*mu011;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MomentInvariantsEstimation<PointInT, PointOutT>::computePointMomentInvariants (
      const pcl::PointCloud<PointInT> &cloud, float &j1, float &j2, float &j3)
{
  // Estimate the XYZ centroid
  compute3DCentroid (cloud, xyz_centroid_);

  // Initialize the centralized moments
  float mu200 = 0, mu020 = 0, mu002 = 0, mu110 = 0, mu101 = 0, mu011  = 0;

  // Iterate over the nearest neighbors set
  for (const auto& point: cloud.points)
  {
    // Demean the points
    temp_pt_[0] = point.x - xyz_centroid_[0];
    temp_pt_[1] = point.y - xyz_centroid_[1];
    temp_pt_[2] = point.z - xyz_centroid_[2];

    mu200 += temp_pt_[0] * temp_pt_[0];
    mu020 += temp_pt_[1] * temp_pt_[1];
    mu002 += temp_pt_[2] * temp_pt_[2];
    mu110 += temp_pt_[0] * temp_pt_[1];
    mu101 += temp_pt_[0] * temp_pt_[2];
    mu011 += temp_pt_[1] * temp_pt_[2];
  }

  // Save the moment invariants
  j1 = mu200             + mu020               + mu002;
  j2 = mu200*mu020       + mu200*mu002         + mu020*mu002       - mu110*mu110       - mu101*mu101       - mu011*mu011;
  j3 = mu200*mu020*mu002 + 2*mu110*mu101*mu011 - mu002*mu110*mu110 - mu020*mu101*mu101 - mu200*mu011*mu011;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MomentInvariantsEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  pcl::Indices nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output[idx].j1 = output[idx].j2 = output[idx].j3 = std::numeric_limits<float>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }
     
      computePointMomentInvariants (*surface_, nn_indices,
                                    output[idx].j1, output[idx].j2, output[idx].j3);
    }
  }
  else
  {
    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (!isFinite ((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output[idx].j1 = output[idx].j2 = output[idx].j3 = std::numeric_limits<float>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      computePointMomentInvariants (*surface_, nn_indices,
                                    output[idx].j1, output[idx].j2, output[idx].j3);
    }
  }
}

#define PCL_INSTANTIATE_MomentInvariantsEstimation(T,NT) template class PCL_EXPORTS pcl::MomentInvariantsEstimation<T,NT>;

#endif    // PCL_FEATURES_IMPL_MOMENT_INVARIANTS_H_ 

