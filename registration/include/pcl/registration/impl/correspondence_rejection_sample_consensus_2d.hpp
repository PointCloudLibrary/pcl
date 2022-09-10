/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_2D_HPP_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_2D_HPP_

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration_2d.h>

#include <unordered_map>

namespace pcl {

namespace registration {

template <typename PointT>
void
CorrespondenceRejectorSampleConsensus2D<PointT>::getRemainingCorrespondences(
    const pcl::Correspondences& original_correspondences,
    pcl::Correspondences& remaining_correspondences)
{
  if (!input_) {
    PCL_ERROR("[pcl::registration::%s::getRemainingCorrespondences] No input cloud "
              "dataset was given!\n",
              getClassName().c_str());
    return;
  }

  if (!target_) {
    PCL_ERROR("[pcl::registration::%s::getRemainingCorrespondences] No input target "
              "dataset was given!\n",
              getClassName().c_str());
    return;
  }

  if (projection_matrix_ == Eigen::Matrix3f::Identity()) {
    PCL_ERROR("[pcl::registration::%s::getRemainingCorrespondences] Intrinsic camera "
              "parameters not given!\n",
              getClassName().c_str());
    return;
  }

  int nr_correspondences = static_cast<int>(original_correspondences.size());
  pcl::Indices source_indices(nr_correspondences);
  pcl::Indices target_indices(nr_correspondences);

  // Copy the query-match indices
  for (std::size_t i = 0; i < original_correspondences.size(); ++i) {
    source_indices[i] = original_correspondences[i].index_query;
    target_indices[i] = original_correspondences[i].index_match;
  }

  // From the set of correspondences found, attempt to remove outliers
  typename pcl::SampleConsensusModelRegistration2D<PointT>::Ptr model(
      new pcl::SampleConsensusModelRegistration2D<PointT>(input_, source_indices));
  // Pass the target_indices
  model->setInputTarget(target_, target_indices);
  model->setProjectionMatrix(projection_matrix_);

  // Create a RANSAC model
  pcl::RandomSampleConsensus<PointT> sac(model, inlier_threshold_);
  sac.setMaxIterations(max_iterations_);

  // Compute the set of inliers
  if (!sac.computeModel()) {
    PCL_ERROR("[pcl::registration::%s::getRemainingCorrespondences] Error computing "
              "model! Returning the original correspondences...\n",
              getClassName().c_str());
    remaining_correspondences = original_correspondences;
    best_transformation_.setIdentity();
    return;
  }
  if (refine_ && !sac.refineModel(2.0))
    PCL_WARN(
        "[pcl::registration::%s::getRemainingCorrespondences] Error refining model!\n",
        getClassName().c_str());

  pcl::Indices inliers;
  sac.getInliers(inliers);

  if (inliers.size() < 3) {
    PCL_ERROR("[pcl::registration::%s::getRemainingCorrespondences] Less than 3 "
              "correspondences found!\n",
              getClassName().c_str());
    remaining_correspondences = original_correspondences;
    best_transformation_.setIdentity();
    return;
  }

  std::unordered_map<int, int> index_to_correspondence;
  for (int i = 0; i < nr_correspondences; ++i)
    index_to_correspondence[original_correspondences[i].index_query] = i;

  remaining_correspondences.resize(inliers.size());
  for (std::size_t i = 0; i < inliers.size(); ++i)
    remaining_correspondences[i] =
        original_correspondences[index_to_correspondence[inliers[i]]];

  // get best transformation
  Eigen::VectorXf model_coefficients;
  sac.getModelCoefficients(model_coefficients);
  best_transformation_.row(0) = model_coefficients.segment<4>(0);
  best_transformation_.row(1) = model_coefficients.segment<4>(4);
  best_transformation_.row(2) = model_coefficients.segment<4>(8);
  best_transformation_.row(3) = model_coefficients.segment<4>(12);
}

} // namespace registration
} // namespace pcl

#endif // PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_2D_HPP_
