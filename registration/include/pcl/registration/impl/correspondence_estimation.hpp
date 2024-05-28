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

#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_

#include <pcl/common/copy_point.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h> // for isXYZFinite

namespace pcl {

namespace registration {

template <typename PointSource, typename PointTarget, typename Scalar>
void
CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::setInputTarget(
    const PointCloudTargetConstPtr& cloud)
{
  if (cloud->points.empty()) {
    PCL_ERROR("[pcl::registration::%s::setInputTarget] Invalid or empty point cloud "
              "dataset given!\n",
              getClassName().c_str());
    return;
  }
  target_ = cloud;

  // Set the internal point representation of choice
  if (point_representation_)
    tree_->setPointRepresentation(point_representation_);

  target_cloud_updated_ = true;
}

template <typename PointSource, typename PointTarget, typename Scalar>
bool
CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute()
{
  if (!target_) {
    PCL_ERROR("[pcl::registration::%s::compute] No input target dataset was given!\n",
              getClassName().c_str());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  if (target_cloud_updated_ && !force_no_recompute_) {
    // If the target indices have been given via setIndicesTarget
    if (target_indices_)
      tree_->setInputCloud(target_, target_indices_);
    else
      tree_->setInputCloud(target_);

    target_cloud_updated_ = false;
  }

  return (PCLBase<PointSource>::initCompute());
}

template <typename PointSource, typename PointTarget, typename Scalar>
bool
CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal()
{
  // Only update source kd-tree if a new target cloud was set
  if (source_cloud_updated_ && !force_no_recompute_reciprocal_) {
    if (point_representation_reciprocal_)
      tree_reciprocal_->setPointRepresentation(point_representation_reciprocal_);
    // If the target indices have been given via setIndicesTarget
    if (indices_)
      tree_reciprocal_->setInputCloud(getInputSource(), getIndicesSource());
    else
      tree_reciprocal_->setInputCloud(getInputSource());

    source_cloud_updated_ = false;
  }

  return (true);
}

namespace detail {

template <
    typename PointTarget,
    typename PointSource,
    typename Index,
    typename std::enable_if_t<isSamePointType<PointSource, PointTarget>()>* = nullptr>
const PointSource&
pointCopyOrRef(typename pcl::PointCloud<PointSource>::ConstPtr& input, const Index& idx)
{
  return (*input)[idx];
}

template <
    typename PointTarget,
    typename PointSource,
    typename Index,
    typename std::enable_if_t<!isSamePointType<PointSource, PointTarget>()>* = nullptr>
PointTarget
pointCopyOrRef(typename pcl::PointCloud<PointSource>::ConstPtr& input, const Index& idx)
{
  // Copy the source data to a target PointTarget format so we can search in the tree
  PointTarget pt;
  copyPoint((*input)[idx], pt);
  return pt;
}

} // namespace detail

template <typename PointSource, typename PointTarget, typename Scalar>
void
CorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineCorrespondences(
    pcl::Correspondences& correspondences, double max_distance)
{
  if (!initCompute())
    return;

  correspondences.resize(indices_->size());

  pcl::Indices index(1);
  std::vector<float> distance(1);
  std::vector<pcl::Correspondences> per_thread_correspondences(num_threads_);
  for (auto& corrs : per_thread_correspondences) {
    corrs.reserve(2 * indices_->size() / num_threads_);
  }
  double max_dist_sqr = max_distance * max_distance;

#pragma omp parallel for default(none)                                                 \
    shared(max_dist_sqr, per_thread_correspondences) firstprivate(index, distance)     \
        num_threads(num_threads_)
  // Iterate over the input set of source indices
  for (int i = 0; i < static_cast<int>(indices_->size()); i++) {
    const auto& idx = (*indices_)[i];
    // Check if the template types are the same. If true, avoid a copy.
    // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT
    // macro!
    const auto& pt = detail::pointCopyOrRef<PointTarget, PointSource>(input_, idx);
    if (!input_->is_dense && !pcl::isXYZFinite(pt))
      continue;
    tree_->nearestKSearch(pt, 1, index, distance);
    if (distance[0] > max_dist_sqr)
      continue;

    pcl::Correspondence corr;
    corr.index_query = idx;
    corr.index_match = index[0];
    corr.distance = distance[0];

#ifdef _OPENMP
    const int thread_num = omp_get_thread_num();
#else
    const int thread_num = 0;
#endif

    per_thread_correspondences[thread_num].emplace_back(corr);
  }

  if (num_threads_ == 1) {
    correspondences = std::move(per_thread_correspondences.front());
  }
  else {
    const unsigned int nr_correspondences = std::accumulate(
        per_thread_correspondences.begin(),
        per_thread_correspondences.end(),
        static_cast<unsigned int>(0),
        [](const auto sum, const auto& corr) { return sum + corr.size(); });
    correspondences.resize(nr_correspondences);

    // Merge per-thread correspondences while keeping them ordered
    auto insert_loc = correspondences.begin();
    for (const auto& corrs : per_thread_correspondences) {
      const auto new_insert_loc = std::move(corrs.begin(), corrs.end(), insert_loc);
      std::inplace_merge(correspondences.begin(),
                         insert_loc,
                         insert_loc + corrs.size(),
                         [](const auto& lhs, const auto& rhs) {
                           return lhs.index_query < rhs.index_query;
                         });
      insert_loc = new_insert_loc;
    }
  }
  deinitCompute();
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
CorrespondenceEstimation<PointSource, PointTarget, Scalar>::
    determineReciprocalCorrespondences(pcl::Correspondences& correspondences,
                                       double max_distance)
{
  if (!initCompute())
    return;

  // setup tree for reciprocal search
  // Set the internal point representation of choice
  if (!initComputeReciprocal())
    return;
  double max_dist_sqr = max_distance * max_distance;

  correspondences.resize(indices_->size());
  pcl::Indices index(1);
  std::vector<float> distance(1);
  pcl::Indices index_reciprocal(1);
  std::vector<float> distance_reciprocal(1);
  std::vector<pcl::Correspondences> per_thread_correspondences(num_threads_);
  for (auto& corrs : per_thread_correspondences) {
    corrs.reserve(2 * indices_->size() / num_threads_);
  }

#pragma omp parallel for default(none)                                                 \
    shared(max_dist_sqr, per_thread_correspondences)                                   \
        firstprivate(index, distance, index_reciprocal, distance_reciprocal)           \
            num_threads(num_threads_)
  // Iterate over the input set of source indices
  for (int i = 0; i < static_cast<int>(indices_->size()); i++) {
    const auto& idx = (*indices_)[i];
    // Check if the template types are the same. If true, avoid a copy.
    // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT
    // macro!

    const auto& pt_src = detail::pointCopyOrRef<PointTarget, PointSource>(input_, idx);
    if (!input_->is_dense && !pcl::isXYZFinite(pt_src))
      continue;
    tree_->nearestKSearch(pt_src, 1, index, distance);
    if (distance[0] > max_dist_sqr)
      continue;

    const auto target_idx = index[0];
    const auto& pt_tgt =
        detail::pointCopyOrRef<PointSource, PointTarget>(target_, target_idx);

    tree_reciprocal_->nearestKSearch(pt_tgt, 1, index_reciprocal, distance_reciprocal);
    if (distance_reciprocal[0] > max_dist_sqr || idx != index_reciprocal[0])
      continue;

    pcl::Correspondence corr;
    corr.index_query = idx;
    corr.index_match = index[0];
    corr.distance = distance[0];

#ifdef _OPENMP
    const int thread_num = omp_get_thread_num();
#else
    const int thread_num = 0;
#endif

    per_thread_correspondences[thread_num].emplace_back(corr);
  }

  if (num_threads_ == 1) {
    correspondences = std::move(per_thread_correspondences.front());
  }
  else {
    const unsigned int nr_correspondences = std::accumulate(
        per_thread_correspondences.begin(),
        per_thread_correspondences.end(),
        static_cast<unsigned int>(0),
        [](const auto sum, const auto& corr) { return sum + corr.size(); });
    correspondences.resize(nr_correspondences);

    // Merge per-thread correspondences while keeping them ordered
    auto insert_loc = correspondences.begin();
    for (const auto& corrs : per_thread_correspondences) {
      const auto new_insert_loc = std::move(corrs.begin(), corrs.end(), insert_loc);
      std::inplace_merge(correspondences.begin(),
                         insert_loc,
                         insert_loc + corrs.size(),
                         [](const auto& lhs, const auto& rhs) {
                           return lhs.index_query < rhs.index_query;
                         });
      insert_loc = new_insert_loc;
    }
  }

  deinitCompute();
}

} // namespace registration
} // namespace pcl

//#define PCL_INSTANTIATE_CorrespondenceEstimation(T,U) template class PCL_EXPORTS
// pcl::registration::CorrespondenceEstimation<T,U>;

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_ */
