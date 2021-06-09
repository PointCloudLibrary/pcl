/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef IA_RANSAC_HPP_
#define IA_RANSAC_HPP_

#include <pcl/common/distances.h>

namespace pcl {

template <typename PointSource, typename PointTarget, typename FeatureT>
void
SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::setSourceFeatures(
    const FeatureCloudConstPtr& features)
{
  if (features == nullptr || features->empty()) {
    PCL_ERROR(
        "[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!\n",
        getClassName().c_str());
    return;
  }
  input_features_ = features;
}

template <typename PointSource, typename PointTarget, typename FeatureT>
void
SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::setTargetFeatures(
    const FeatureCloudConstPtr& features)
{
  if (features == nullptr || features->empty()) {
    PCL_ERROR(
        "[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!\n",
        getClassName().c_str());
    return;
  }
  target_features_ = features;
  feature_tree_->setInputCloud(target_features_);
}

template <typename PointSource, typename PointTarget, typename FeatureT>
void
SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::selectSamples(
    const PointCloudSource& cloud,
    unsigned int nr_samples,
    float min_sample_distance,
    pcl::Indices& sample_indices)
{
  if (nr_samples > cloud.size()) {
    PCL_ERROR("[pcl::%s::selectSamples] ", getClassName().c_str());
    PCL_ERROR("The number of samples (%u) must not be greater than the number of "
              "points (%zu)!\n",
              nr_samples,
              static_cast<std::size_t>(cloud.size()));
    return;
  }

  // Iteratively draw random samples until nr_samples is reached
  index_t iterations_without_a_sample = 0;
  const auto max_iterations_without_a_sample = 3 * cloud.size();
  sample_indices.clear();
  while (sample_indices.size() < nr_samples) {
    // Choose a sample at random
    const auto sample_index = getRandomIndex(cloud.size());

    // Check to see if the sample is 1) unique and 2) far away from the other samples
    bool valid_sample = true;
    for (const auto& sample_idx : sample_indices) {
      float distance_between_samples =
          euclideanDistance(cloud[sample_index], cloud[sample_idx]);

      if (sample_index == sample_idx ||
          distance_between_samples < min_sample_distance) {
        valid_sample = false;
        break;
      }
    }

    // If the sample is valid, add it to the output
    if (valid_sample) {
      sample_indices.push_back(sample_index);
      iterations_without_a_sample = 0;
    }
    else
      ++iterations_without_a_sample;

    // If no valid samples can be found, relax the inter-sample distance requirements
    if (static_cast<std::size_t>(iterations_without_a_sample) >=
        max_iterations_without_a_sample) {
      PCL_WARN("[pcl::%s::selectSamples] ", getClassName().c_str());
      PCL_WARN("No valid sample found after %zu iterations. Relaxing "
               "min_sample_distance_ to %f\n",
               static_cast<std::size_t>(iterations_without_a_sample),
               0.5 * min_sample_distance);

      min_sample_distance_ *= 0.5f;
      min_sample_distance = min_sample_distance_;
      iterations_without_a_sample = 0;
    }
  }
}

template <typename PointSource, typename PointTarget, typename FeatureT>
void
SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::
    findSimilarFeatures(const FeatureCloud& input_features,
                        const pcl::Indices& sample_indices,
                        pcl::Indices& corresponding_indices)
{
  pcl::Indices nn_indices(k_correspondences_);
  std::vector<float> nn_distances(k_correspondences_);

  corresponding_indices.resize(sample_indices.size());
  for (std::size_t i = 0; i < sample_indices.size(); ++i) {
    // Find the k features nearest to input_features[sample_indices[i]]
    feature_tree_->nearestKSearch(input_features,
                                  sample_indices[i],
                                  k_correspondences_,
                                  nn_indices,
                                  nn_distances);

    // Select one at random and add it to corresponding_indices
    const auto random_correspondence = getRandomIndex(k_correspondences_);
    corresponding_indices[i] = nn_indices[random_correspondence];
  }
}

template <typename PointSource, typename PointTarget, typename FeatureT>
float
SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::computeErrorMetric(
    const PointCloudSource& cloud, float)
{
  pcl::Indices nn_index(1);
  std::vector<float> nn_distance(1);

  const ErrorFunctor& compute_error = *error_functor_;
  float error = 0;

  for (const auto& point : cloud) {
    // Find the distance between point and its nearest neighbor in the target point
    // cloud
    tree_->nearestKSearch(point, 1, nn_index, nn_distance);

    // Compute the error
    error += compute_error(nn_distance[0]);
  }
  return (error);
}

template <typename PointSource, typename PointTarget, typename FeatureT>
void
SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::
    computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess)
{
  // Some sanity checks first
  if (!input_features_) {
    PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
    PCL_ERROR(
        "No source features were given! Call setSourceFeatures before aligning.\n");
    return;
  }
  if (!target_features_) {
    PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
    PCL_ERROR(
        "No target features were given! Call setTargetFeatures before aligning.\n");
    return;
  }

  if (input_->size() != input_features_->size()) {
    PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
    PCL_ERROR("The source points and source feature points need to be in a one-to-one "
              "relationship! Current input cloud sizes: %ld vs %ld.\n",
              input_->size(),
              input_features_->size());
    return;
  }

  if (target_->size() != target_features_->size()) {
    PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
    PCL_ERROR("The target points and target feature points need to be in a one-to-one "
              "relationship! Current input cloud sizes: %ld vs %ld.\n",
              target_->size(),
              target_features_->size());
    return;
  }

  if (!error_functor_)
    error_functor_.reset(new TruncatedError(static_cast<float>(corr_dist_threshold_)));

  pcl::Indices sample_indices(nr_samples_);
  pcl::Indices corresponding_indices(nr_samples_);
  PointCloudSource input_transformed;
  float lowest_error(0);

  final_transformation_ = guess;
  int i_iter = 0;
  converged_ = false;
  if (!guess.isApprox(Eigen::Matrix4f::Identity(), 0.01f)) {
    // If guess is not the Identity matrix we check it.
    transformPointCloud(*input_, input_transformed, final_transformation_);
    lowest_error =
        computeErrorMetric(input_transformed, static_cast<float>(corr_dist_threshold_));
    i_iter = 1;
  }

  for (; i_iter < max_iterations_; ++i_iter) {
    // Draw nr_samples_ random samples
    selectSamples(*input_, nr_samples_, min_sample_distance_, sample_indices);

    // Find corresponding features in the target cloud
    findSimilarFeatures(*input_features_, sample_indices, corresponding_indices);

    // Estimate the transform from the samples to their corresponding points
    transformation_estimation_->estimateRigidTransformation(
        *input_, sample_indices, *target_, corresponding_indices, transformation_);

    // Transform the data and compute the error
    transformPointCloud(*input_, input_transformed, transformation_);
    float error =
        computeErrorMetric(input_transformed, static_cast<float>(corr_dist_threshold_));

    // If the new error is lower, update the final transformation
    if (i_iter == 0 || error < lowest_error) {
      lowest_error = error;
      final_transformation_ = transformation_;
      converged_ = true;
    }
  }

  // Apply the final transformation
  transformPointCloud(*input_, output, final_transformation_);
}

} // namespace pcl

#endif //#ifndef IA_RANSAC_HPP_
