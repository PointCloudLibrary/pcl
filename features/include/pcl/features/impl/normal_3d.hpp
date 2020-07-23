/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 */

#pragma once

#include <pcl/features/normal_3d.h>

template <typename PointInT, typename PointOutT> void
pcl::NormalEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;

#pragma omp parallel for \
  default(none) \
  shared(output) \
  firstprivate(nn_indices, nn_dists) \
  num_threads(threads_) \
  if (threads_ != -1)
    // Iterating over the entire index vector
    for (std::ptrdiff_t idx = 0; idx < static_cast<std::ptrdiff_t> (indices_->size ()); ++idx)
    {
      Eigen::Vector4f n;

      if (input_->is_dense || isFinite ((*input_)[(*indices_)[idx]])) {
        if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) &&
            pcl::computePointNormal (*surface_, nn_indices, n, output[idx].curvature)) {
          output[idx].normal_x = n[0];
          output[idx].normal_y = n[1];
          output[idx].normal_z = n[2];

          flipNormalTowardsViewpoint ((*input_)[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                      output[idx].normal[0], output[idx].normal[1], output[idx].normal[2]);
          continue;
        }
      }

      output[idx].normal[0] = output[idx].normal[1] = output[idx].normal[2] = output[idx].curvature = output[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

      if (output.is_dense)
        output.is_dense = false;
    }
}

#define PCL_INSTANTIATE_NormalEstimation(T,NT) template class PCL_EXPORTS pcl::NormalEstimation<T,NT>;
