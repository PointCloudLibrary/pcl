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
 *
 */

#ifndef PCL_FEATURES_IMPL_SHOT_OMP_H_
#define PCL_FEATURES_IMPL_SHOT_OMP_H_

#include "pcl/features/shot_omp.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::SHOTEstimationOMP<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if (threads_ < 0)
    threads_ = 1;

  descLength_ = nr_grid_sector_ * (nr_shape_bins_ + 1);

  sqradius_ = search_radius_ * search_radius_;
  radius3_4_ = (search_radius_ * 3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

	if (output.points[0].descriptor.size () != static_cast<size_t> (descLength_))
		for (size_t idx = 0; idx < indices_->size (); ++idx)
			output.points[idx].descriptor.resize (descLength_);

  int data_size = static_cast<int> (indices_->size ());
  Eigen::VectorXf *shot = new Eigen::VectorXf[threads_];
  std::vector<std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > > rfs (threads_);
  for (size_t i = 0; i < rfs.size (); ++i)
    rfs[i].resize (3);

  for (int i = 0; i < threads_; i++)
    shot[i].setZero (descLength_);

  // Iterating over the entire index vector
  #pragma omp parallel for num_threads(threads_)
  for (int idx = 0; idx < data_size; ++idx)
  {
	 // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

	// Estimate the SHOT at each patch
#ifdef _OPENMP
    int tid = omp_get_thread_num ();
#else
    int tid = 0;
#endif
	this->computePointSHOT ((*indices_)[idx], nn_indices, nn_dists, shot[tid], rfs[tid]);

	// Copy into the resultant cloud
    for (int d = 0; d < shot[tid].size (); ++d)
      output.points[idx].descriptor[d] = shot[tid][d];
    for (int d = 0; d < 9; ++d)
      output.points[idx].rf[d] = rfs[tid][d/3][d % 3];
  }
  delete[] shot;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointNT, typename PointOutT> void
pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if (threads_ < 0)
    threads_ = 1;

  descLength_ = (b_describe_shape_) ? nr_grid_sector_ * (nr_shape_bins_ + 1) : 0;
  descLength_ += (b_describe_color_) ? nr_grid_sector_ * (nr_color_bins_ + 1) : 0;

  sqradius_ = search_radius_ * search_radius_;
  radius3_4_ = (search_radius_ * 3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  if (output.points[0].descriptor.size () != static_cast<size_t> (descLength_))
    for (size_t idx = 0; idx < indices_->size (); ++idx)
      output.points[idx].descriptor.resize (descLength_);

  int data_size = static_cast<int> (indices_->size ());
  Eigen::VectorXf *shot = new Eigen::VectorXf[threads_];
  std::vector<std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > > rfs (threads_);
  for (size_t i = 0; i < rfs.size (); ++i)
    rfs[i].resize (3);

  for (int i = 0; i < threads_; i++)
    shot[i].setZero (descLength_);

  // Iterating over the entire index vector
#pragma omp parallel for num_threads(threads_)
  for (int idx = 0; idx < data_size; ++idx)
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the SHOT at each patch
#ifdef _OPENMP
    int tid = omp_get_thread_num ();
#else
    int tid = 0;
#endif
    this->computePointSHOT ((*indices_)[idx], nn_indices, nn_dists, shot[tid], rfs[tid]);

    // Copy into the resultant cloud
    for (int d = 0; d < shot[tid].size (); ++d)
      output.points[idx].descriptor[d] = shot[tid][d];
    for (int d = 0; d < 9; ++d)
      output.points[idx].rf[d] = rfs[tid][d / 3][d % 3];
  }

  delete[] shot;
}

#define PCL_INSTANTIATE_SHOTEstimationOMP(T,NT,OutT) template class PCL_EXPORTS pcl::SHOTEstimationOMP<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_SHOT_OMP_H_
