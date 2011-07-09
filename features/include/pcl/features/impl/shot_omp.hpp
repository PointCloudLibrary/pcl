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
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::SHOTEstimationOMP<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    PCL_ERROR (
      "[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n",
      getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  if (threads_ < 0)
    threads_ = omp_get_max_threads ();

  descLength_ = nr_grid_sector_*(nr_shape_bins_+1);

  sqradius_ = search_radius_*search_radius_;
  sqradius4_ = sqradius_ / 4;
  radius3_4_ = (search_radius_*3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  int data_size = indices_->size ();
  Eigen::VectorXf *shot = new Eigen::VectorXf[threads_];
  Eigen::Vector3f *rf = new Eigen::Vector3f[threads_*3];

  for (int i = 0; i < threads_; i++)
    shot[i].setZero (descLength_);

  int tid;

  // Iterating over the entire index vector
#pragma omp parallel for private(tid) num_threads(threads_)
  for (int idx = 0; idx < data_size; ++idx)
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the SHOT at each patch
    tid = omp_get_thread_num ();
    computePointSHOT (*surface_, *normals_, idx, nn_indices, nn_dists, shot[tid], &rf[tid*3]);

    // Copy into the resultant cloud
    for (int d = 0; d < shot[tid].size (); ++d)
      output.points[idx].descriptor[d] = shot[tid][d];
    for (int d = 0; d < 9; ++d)
      output.points[idx].rf[d] = rf_[tid*3 + d/3][d%3];
  }

  delete [] shot;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT, typename PointOutT> void
pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    PCL_ERROR (
      "[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n",
      getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  if (threads_ < 0)
    threads_ = omp_get_max_threads ();

  descLength_ = (b_describe_shape_) ? nr_grid_sector_*(nr_shape_bins_+1) : 0;
  descLength_ +=   (b_describe_color_) ? nr_grid_sector_*(nr_color_bins_+1) : 0;

  sqradius_ = search_radius_*search_radius_;
  sqradius4_ = sqradius_ / 4;
  radius3_4_ = (search_radius_*3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  int data_size = indices_->size ();
  Eigen::VectorXf *shot = new Eigen::VectorXf[threads_];
  Eigen::Vector3f *rf = new Eigen::Vector3f[threads_*3];

  for (int i = 0; i < threads_; i++)
    shot[i].setZero (descLength_);

  int tid;

  // Iterating over the entire index vector
#pragma omp parallel for private(tid) num_threads(threads_)
  for (int idx = 0; idx < data_size; ++idx)
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the SHOT at each patch
    tid = omp_get_thread_num ();
    computePointSHOT (*surface_, *normals_, idx, nn_indices, nn_dists, shot[tid], &rf[tid*3]);

    // Copy into the resultant cloud
    for (int d = 0; d < shot[tid].size (); ++d)
      output.points[idx].descriptor[d] = shot[tid][d];
    for (int d = 0; d < 9; ++d)
      output.points[idx].rf[d] = rf_[tid*3 + d/3][d%3];
  }

  delete [] shot;
}

#define PCL_INSTANTIATE_SHOTEstimationOMP(T,NT,OutT) template class PCL_EXPORTS pcl::SHOTEstimationOMP<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_SHOT_OMP_H_

