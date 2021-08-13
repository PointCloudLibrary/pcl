/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *
 */

#pragma once

#include <pcl/features/shot_omp.h>

#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/common/time.h>
#include <pcl/features/shot_lrf_omp.h>


template<typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> bool
pcl::SHOTEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>::initCompute ()
{
  if (!FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // SHOT cannot work with k-search
  if (this->getKSearch () != 0)
  {
    PCL_ERROR(
      "[pcl::%s::initCompute] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch( radius ) to use this class.\n",
      getClassName().c_str ());
    return (false);
  }

  // Default LRF estimation alg: SHOTLocalReferenceFrameEstimationOMP
  typename SHOTLocalReferenceFrameEstimationOMP<PointInT, PointRFT>::Ptr lrf_estimator(new SHOTLocalReferenceFrameEstimationOMP<PointInT, PointRFT>);
  lrf_estimator->setRadiusSearch ((lrf_radius_ > 0 ? lrf_radius_ : search_radius_));
  lrf_estimator->setInputCloud (input_);
  lrf_estimator->setIndices (indices_);
  lrf_estimator->setNumberOfThreads(threads_);

  if (!fake_surface_)
    lrf_estimator->setSearchSurface(surface_);

  if (!FeatureWithLocalReferenceFrames<PointInT, PointRFT>::initLocalReferenceFrames (indices_->size (), lrf_estimator))
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> bool
pcl::SHOTColorEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>::initCompute ()
{
  if (!FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // SHOT cannot work with k-search
  if (this->getKSearch () != 0)
  {
    PCL_ERROR(
      "[pcl::%s::initCompute] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch( radius ) to use this class.\n",
      getClassName().c_str ());
    return (false);
  }

  // Default LRF estimation alg: SHOTLocalReferenceFrameEstimationOMP
  typename SHOTLocalReferenceFrameEstimationOMP<PointInT, PointRFT>::Ptr lrf_estimator(new SHOTLocalReferenceFrameEstimationOMP<PointInT, PointRFT>);
  lrf_estimator->setRadiusSearch ((lrf_radius_ > 0 ? lrf_radius_ : search_radius_));
  lrf_estimator->setInputCloud (input_);
  lrf_estimator->setIndices (indices_);
  lrf_estimator->setNumberOfThreads(threads_);

  if (!fake_surface_)
    lrf_estimator->setSearchSurface(surface_);

  if (!FeatureWithLocalReferenceFrames<PointInT, PointRFT>::initLocalReferenceFrames (indices_->size (), lrf_estimator))
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::SHOTEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>::setNumberOfThreads (unsigned int nr_threads)
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

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::SHOTEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>::computeFeature (PointCloudOut &output)
{
  descLength_ = nr_grid_sector_ * (nr_shape_bins_ + 1);

  sqradius_ = search_radius_ * search_radius_;
  radius3_4_ = (search_radius_ * 3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  assert(descLength_ == 352);

  output.is_dense = true;
  // Iterating over the entire index vector
#pragma omp parallel for \
  default(none) \
  shared(output) \
  num_threads(threads_)
  for (std::ptrdiff_t idx = 0; idx < static_cast<std::ptrdiff_t> (indices_->size ()); ++idx)
  {

    Eigen::VectorXf shot;
    shot.setZero (descLength_);

    bool lrf_is_nan = false;
    const PointRFT& current_frame = (*frames_)[idx];
    if (!std::isfinite (current_frame.x_axis[0]) ||
        !std::isfinite (current_frame.y_axis[0]) ||
        !std::isfinite (current_frame.z_axis[0]))
    {
      PCL_WARN ("[pcl::%s::computeFeature] The local reference frame is not valid! Aborting description of point with index %d\n",
        getClassName ().c_str (), (*indices_)[idx]);
      lrf_is_nan = true;
    }

    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    pcl::Indices nn_indices (k_);
    std::vector<float> nn_dists (k_);

    if (!isFinite ((*input_)[(*indices_)[idx]]) || lrf_is_nan || this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices,
                                                                                           nn_dists) == 0)
    {
      // Copy into the resultant cloud
      for (Eigen::Index d = 0; d < shot.size (); ++d)
        output[idx].descriptor[d] = std::numeric_limits<float>::quiet_NaN ();
      for (int d = 0; d < 9; ++d)
        output[idx].rf[d] = std::numeric_limits<float>::quiet_NaN ();

      output.is_dense = false;
      continue;
    }

    // Estimate the SHOT at each patch
    this->computePointSHOT (idx, nn_indices, nn_dists, shot);

    // Copy into the resultant cloud
    for (Eigen::Index d = 0; d < shot.size (); ++d)
      output[idx].descriptor[d] = shot[d];
    for (int d = 0; d < 3; ++d)
    {
      output[idx].rf[d + 0] = (*frames_)[idx].x_axis[d];
      output[idx].rf[d + 3] = (*frames_)[idx].y_axis[d];
      output[idx].rf[d + 6] = (*frames_)[idx].z_axis[d];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::SHOTColorEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>::setNumberOfThreads (unsigned int nr_threads)
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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT> void
pcl::SHOTColorEstimationOMP<PointInT, PointNT, PointOutT, PointRFT>::computeFeature (PointCloudOut &output)
{
  descLength_ = (b_describe_shape_) ? nr_grid_sector_ * (nr_shape_bins_ + 1) : 0;
  descLength_ += (b_describe_color_) ? nr_grid_sector_ * (nr_color_bins_ + 1) : 0;

  assert( (!b_describe_color_ && b_describe_shape_ && descLength_ == 352) ||
          (b_describe_color_ && !b_describe_shape_ && descLength_ == 992) ||
          (b_describe_color_ && b_describe_shape_ && descLength_ == 1344)
        );

  sqradius_ = search_radius_ * search_radius_;
  radius3_4_ = (search_radius_ * 3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  output.is_dense = true;
  // Iterating over the entire index vector
#pragma omp parallel for \
  default(none) \
  shared(output) \
  num_threads(threads_)
  for (std::ptrdiff_t idx = 0; idx < static_cast<std::ptrdiff_t> (indices_->size ()); ++idx)
  {
    Eigen::VectorXf shot;
    shot.setZero (descLength_);

    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    pcl::Indices nn_indices (k_);
    std::vector<float> nn_dists (k_);

    bool lrf_is_nan = false;
    const PointRFT& current_frame = (*frames_)[idx];
    if (!std::isfinite (current_frame.x_axis[0]) ||
        !std::isfinite (current_frame.y_axis[0]) ||
        !std::isfinite (current_frame.z_axis[0]))
    {
      PCL_WARN ("[pcl::%s::computeFeature] The local reference frame is not valid! Aborting description of point with index %d\n",
        getClassName ().c_str (), (*indices_)[idx]);
      lrf_is_nan = true;
    }

    if (!isFinite ((*input_)[(*indices_)[idx]]) ||
        lrf_is_nan ||
        this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
    {
      // Copy into the resultant cloud
      for (Eigen::Index d = 0; d < shot.size (); ++d)
        output[idx].descriptor[d] = std::numeric_limits<float>::quiet_NaN ();
      for (int d = 0; d < 9; ++d)
        output[idx].rf[d] = std::numeric_limits<float>::quiet_NaN ();

      output.is_dense = false;
      continue;
    }

    // Estimate the SHOT at each patch
    this->computePointSHOT (idx, nn_indices, nn_dists, shot);

    // Copy into the resultant cloud
    for (Eigen::Index d = 0; d < shot.size (); ++d)
      output[idx].descriptor[d] = shot[d];
    for (int d = 0; d < 3; ++d)
    {
      output[idx].rf[d + 0] = (*frames_)[idx].x_axis[d];
      output[idx].rf[d + 3] = (*frames_)[idx].y_axis[d];
      output[idx].rf[d + 6] = (*frames_)[idx].z_axis[d];
    }
  }
}

#define PCL_INSTANTIATE_SHOTEstimationOMP(T,NT,OutT,RFT) template class PCL_EXPORTS pcl::SHOTEstimationOMP<T,NT,OutT,RFT>;
#define PCL_INSTANTIATE_SHOTColorEstimationOMP(T,NT,OutT,RFT) template class PCL_EXPORTS pcl::SHOTColorEstimationOMP<T,NT,OutT,RFT>;

