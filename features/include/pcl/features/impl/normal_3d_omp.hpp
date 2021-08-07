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

#ifndef PCL_FEATURES_IMPL_NORMAL_3D_OMP_H_
#define PCL_FEATURES_IMPL_NORMAL_3D_OMP_H_

#include <pcl/features/normal_3d_omp.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::NormalEstimationOMP<PointInT, PointOutT>::setNumberOfThreads (unsigned int nr_threads)
{
  if (nr_threads == 0)
#ifdef _OPENMP
    threads_ = omp_get_num_procs();
#else
    threads_ = 1;
#endif
  else
    threads_ = nr_threads;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::NormalEstimationOMP<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  pcl::Indices nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
#pragma omp parallel for \
  default(none) \
  shared(output) \
  firstprivate(nn_indices, nn_dists) \
  num_threads(threads_)
    // Iterating over the entire index vector
    for (std::ptrdiff_t idx = 0; idx < static_cast<std::ptrdiff_t> (indices_->size ()); ++idx)
    {
      Eigen::Vector4f n;
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
          !pcl::computePointNormal (*surface_, nn_indices, n, output[idx].curvature))
      {
        output[idx].normal[0] = output[idx].normal[1] = output[idx].normal[2] = output[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      output[idx].normal_x = n[0];
      output[idx].normal_y = n[1];
      output[idx].normal_z = n[2];

      flipNormalTowardsViewpoint ((*input_)[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output[idx].normal[0], output[idx].normal[1], output[idx].normal[2]);

    }
  }
  else
  {
#pragma omp parallel for \
  default(none) \
  shared(output) \
  firstprivate(nn_indices, nn_dists) \
  num_threads(threads_)
    // Iterating over the entire index vector
    for (std::ptrdiff_t idx = 0; idx < static_cast<std::ptrdiff_t> (indices_->size ()); ++idx)
    {
      Eigen::Vector4f n;
      if (!isFinite ((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
          !pcl::computePointNormal (*surface_, nn_indices, n, output[idx].curvature))
      {
        output[idx].normal[0] = output[idx].normal[1] = output[idx].normal[2] = output[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      output[idx].normal_x = n[0];
      output[idx].normal_y = n[1];
      output[idx].normal_z = n[2];

      flipNormalTowardsViewpoint ((*input_)[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output[idx].normal[0], output[idx].normal[1], output[idx].normal[2]);

    }
  }
}

#define PCL_INSTANTIATE_NormalEstimationOMP(T,NT) template class PCL_EXPORTS pcl::NormalEstimationOMP<T,NT>;

#endif    // PCL_FEATURES_IMPL_NORMAL_3D_OMP_H_

