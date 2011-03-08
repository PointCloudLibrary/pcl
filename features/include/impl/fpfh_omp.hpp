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
 * $Id: fpfh_omp.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_FPFH_OMP_H_
#define PCL_FEATURES_IMPL_FPFH_OMP_H_

#include "pcl/features/fpfh_omp.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::FPFHEstimationOMP<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
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

  int data_size = indices_->size ();
  // Reset the whole thing
  hist_f1_.setZero (data_size, nr_bins_f1_);
  hist_f2_.setZero (data_size, nr_bins_f2_);
  hist_f3_.setZero (data_size, nr_bins_f3_);

  int nr_bins = nr_bins_f1_ + nr_bins_f2_ + nr_bins_f3_;

  // Iterating over the entire index vector
#pragma omp parallel for schedule (dynamic, threads_)
  for (int idx = 0; idx < data_size; ++idx)
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the FPFH signature at each patch
    computePointSPFHSignature (*surface_, *normals_, (*indices_)[idx], nn_indices,
                               hist_f1_, hist_f2_, hist_f3_);
  }

  // Iterating over the entire index vector
#pragma omp parallel for schedule (dynamic, threads_)
  for (int idx = 0; idx < data_size; ++idx)
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    Eigen::VectorXf fpfh_histogram = Eigen::VectorXf::Zero (nr_bins);

    searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    weightPointSPFHSignature (hist_f1_, hist_f2_, hist_f3_, nn_indices, nn_dists, fpfh_histogram);

    // Copy into the resultant cloud
    for (int d = 0; d < fpfh_histogram.size (); ++d)
      output.points[idx].histogram[d] = fpfh_histogram[d];
  }
}

#define PCL_INSTANTIATE_FPFHEstimationOMP(T,NT,OutT) template class pcl::FPFHEstimationOMP<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_FPFH_OMP_H_ 

