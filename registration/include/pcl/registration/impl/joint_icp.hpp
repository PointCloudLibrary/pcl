/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_REGISTRATION_IMPL_JOINT_ICP_HPP_
#define PCL_REGISTRATION_IMPL_JOINT_ICP_HPP_

#include <pcl/console/print.h>
#include <pcl/correspondence.h>

namespace pcl {

template <typename PointSource, typename PointTarget, typename Scalar>
void
JointIterativeClosestPoint<PointSource, PointTarget, Scalar>::computeTransformation(
    PointCloudSource& output, const Matrix4& guess)
{
  // Point clouds containing the correspondences of each point in <input, indices>
  if (sources_.size() != targets_.size() || sources_.empty() || targets_.empty()) {
    PCL_ERROR("[pcl::%s::computeTransformation] Must set InputSources and InputTargets "
              "to the same, nonzero size!\n",
              getClassName().c_str());
    return;
  }
  bool manual_correspondence_estimations_set = true;
  if (correspondence_estimations_.empty()) {
    manual_correspondence_estimations_set = false;
    correspondence_estimations_.resize(sources_.size());
    for (std::size_t i = 0; i < sources_.size(); i++) {
      correspondence_estimations_[i] = correspondence_estimation_->clone();
      KdTreeReciprocalPtr src_tree(new KdTreeReciprocal);
      KdTreePtr tgt_tree(new KdTree);
      correspondence_estimations_[i]->setSearchMethodTarget(tgt_tree);
      correspondence_estimations_[i]->setSearchMethodSource(src_tree);
    }
  }
  if (correspondence_estimations_.size() != sources_.size()) {
    PCL_ERROR("[pcl::%s::computeTransform] Must set CorrespondenceEstimations to be "
              "the same size as the joint\n",
              getClassName().c_str());
    return;
  }
  std::vector<PointCloudSourcePtr> inputs_transformed(sources_.size());
  for (std::size_t i = 0; i < sources_.size(); i++) {
    inputs_transformed[i].reset(new PointCloudSource);
  }

  nr_iterations_ = 0;
  converged_ = false;

  // Initialise final transformation to the guessed one
  final_transformation_ = guess;

  // Make a combined transformed input and output
  std::vector<std::size_t> input_offsets(sources_.size());
  std::vector<std::size_t> target_offsets(targets_.size());
  PointCloudSourcePtr sources_combined(new PointCloudSource);
  PointCloudSourcePtr inputs_transformed_combined(new PointCloudSource);
  PointCloudTargetPtr targets_combined(new PointCloudTarget);
  std::size_t input_offset = 0;
  std::size_t target_offset = 0;
  for (std::size_t i = 0; i < sources_.size(); i++) {
    // If the guessed transformation is non identity
    if (guess != Matrix4::Identity()) {
      // Apply guessed transformation prior to search for neighbours
      this->transformCloud(*sources_[i], *inputs_transformed[i], guess);
    }
    else {
      *inputs_transformed[i] = *sources_[i];
    }
    *sources_combined += *sources_[i];
    *inputs_transformed_combined += *inputs_transformed[i];
    *targets_combined += *targets_[i];
    input_offsets[i] = input_offset;
    target_offsets[i] = target_offset;
    input_offset += inputs_transformed[i]->size();
    target_offset += targets_[i]->size();
  }

  transformation_ = Matrix4::Identity();
  // Make blobs if necessary
  determineRequiredBlobData();
  // Pass in the default target for the Correspondence Estimation/Rejection code
  for (std::size_t i = 0; i < sources_.size(); i++) {
    correspondence_estimations_[i]->setInputTarget(targets_[i]);
    if (correspondence_estimations_[i]->requiresTargetNormals()) {
      PCLPointCloud2::Ptr target_blob(new PCLPointCloud2);
      pcl::toPCLPointCloud2(*targets_[i], *target_blob);
      correspondence_estimations_[i]->setTargetNormals(target_blob);
    }
  }

  PCLPointCloud2::Ptr targets_combined_blob(new PCLPointCloud2);
  if (!correspondence_rejectors_.empty() && need_target_blob_)
    pcl::toPCLPointCloud2(*targets_combined, *targets_combined_blob);

  for (std::size_t i = 0; i < correspondence_rejectors_.size(); ++i) {
    registration::CorrespondenceRejector::Ptr& rej = correspondence_rejectors_[i];
    if (rej->requiresTargetPoints())
      rej->setTargetPoints(targets_combined_blob);
    if (rej->requiresTargetNormals() && target_has_normals_)
      rej->setTargetNormals(targets_combined_blob);
  }

  convergence_criteria_->setMaximumIterations(max_iterations_);
  convergence_criteria_->setRelativeMSE(euclidean_fitness_epsilon_);
  convergence_criteria_->setTranslationThreshold(transformation_epsilon_);
  convergence_criteria_->setRotationThreshold(1.0 - transformation_epsilon_);

  // Repeat until convergence
  std::vector<CorrespondencesPtr> partial_correspondences_(sources_.size());
  for (std::size_t i = 0; i < sources_.size(); i++) {
    partial_correspondences_[i].reset(new pcl::Correspondences);
  }

  do {
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;

    // Set the source each iteration, to ensure the dirty flag is updated
    correspondences_->clear();
    for (std::size_t i = 0; i < correspondence_estimations_.size(); i++) {
      correspondence_estimations_[i]->setInputSource(inputs_transformed[i]);
      // Get blob data if needed
      if (correspondence_estimations_[i]->requiresSourceNormals()) {
        PCLPointCloud2::Ptr input_transformed_blob(new PCLPointCloud2);
        toPCLPointCloud2(*inputs_transformed[i], *input_transformed_blob);
        correspondence_estimations_[i]->setSourceNormals(input_transformed_blob);
      }

      // Estimate correspondences on each cloud pair separately
      if (use_reciprocal_correspondence_) {
        correspondence_estimations_[i]->determineReciprocalCorrespondences(
            *partial_correspondences_[i], corr_dist_threshold_);
      }
      else {
        correspondence_estimations_[i]->determineCorrespondences(
            *partial_correspondences_[i], corr_dist_threshold_);
      }
      PCL_DEBUG("[pcl::%s::computeTransformation] Found %d partial correspondences for "
                "cloud [%d]\n",
                getClassName().c_str(),
                partial_correspondences_[i]->size(),
                i);
      for (std::size_t j = 0; j < partial_correspondences_[i]->size(); j++) {
        pcl::Correspondence corr = partial_correspondences_[i]->at(j);
        // Update the offsets to be for the combined clouds
        corr.index_query += input_offsets[i];
        corr.index_match += target_offsets[i];
        correspondences_->push_back(corr);
      }
    }
    PCL_DEBUG("[pcl::%s::computeTransformation] Total correspondences: %d\n",
              getClassName().c_str(),
              correspondences_->size());

    PCLPointCloud2::Ptr inputs_transformed_combined_blob;
    if (need_source_blob_) {
      inputs_transformed_combined_blob.reset(new PCLPointCloud2);
      toPCLPointCloud2(*inputs_transformed_combined, *inputs_transformed_combined_blob);
    }
    CorrespondencesPtr temp_correspondences(new Correspondences(*correspondences_));
    for (std::size_t i = 0; i < correspondence_rejectors_.size(); ++i) {
      PCL_DEBUG("Applying a correspondence rejector method: %s.\n",
                correspondence_rejectors_[i]->getClassName().c_str());
      registration::CorrespondenceRejector::Ptr& rej = correspondence_rejectors_[i];
      PCL_DEBUG("Applying a correspondence rejector method: %s.\n",
                rej->getClassName().c_str());
      if (rej->requiresSourcePoints())
        rej->setSourcePoints(inputs_transformed_combined_blob);
      if (rej->requiresSourceNormals() && source_has_normals_)
        rej->setSourceNormals(inputs_transformed_combined_blob);
      rej->setInputCorrespondences(temp_correspondences);
      rej->getCorrespondences(*correspondences_);
      // Modify input for the next iteration
      if (i < correspondence_rejectors_.size() - 1)
        *temp_correspondences = *correspondences_;
    }

    // Check whether we have enough correspondences
    if (correspondences_->size() < min_number_correspondences_) {
      PCL_ERROR("[pcl::%s::computeTransformation] Not enough correspondences found. "
                "Relax your threshold parameters.\n",
                getClassName().c_str());
      convergence_criteria_->setConvergenceState(
          pcl::registration::DefaultConvergenceCriteria<
              Scalar>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES);
      converged_ = false;
      break;
    }

    // Estimate the transform jointly, on a combined correspondence set
    transformation_estimation_->estimateRigidTransformation(
        *inputs_transformed_combined,
        *targets_combined,
        *correspondences_,
        transformation_);

    // Transform the combined data
    this->transformCloud(
        *inputs_transformed_combined, *inputs_transformed_combined, transformation_);
    // And all its components
    for (std::size_t i = 0; i < sources_.size(); i++) {
      this->transformCloud(
          *inputs_transformed[i], *inputs_transformed[i], transformation_);
    }

    // Obtain the final transformation
    final_transformation_ = transformation_ * final_transformation_;

    ++nr_iterations_;

    // Update the vizualization of icp convergence
    // if (update_visualizer_ != 0)
    //  update_visualizer_(output, source_indices_good, *target_, target_indices_good );

    converged_ = static_cast<bool>((*convergence_criteria_));
  } while (!converged_);

  PCL_DEBUG("Transformation "
            "is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%"
            "5f\t%5f\t%5f\t%5f\n",
            final_transformation_(0, 0),
            final_transformation_(0, 1),
            final_transformation_(0, 2),
            final_transformation_(0, 3),
            final_transformation_(1, 0),
            final_transformation_(1, 1),
            final_transformation_(1, 2),
            final_transformation_(1, 3),
            final_transformation_(2, 0),
            final_transformation_(2, 1),
            final_transformation_(2, 2),
            final_transformation_(2, 3),
            final_transformation_(3, 0),
            final_transformation_(3, 1),
            final_transformation_(3, 2),
            final_transformation_(3, 3));

  // For fitness checks, etc, we'll use an aggregated cloud for now (should be
  // evaluating independently for correctness, but this requires propagating a few
  // virtual methods from Registration)
  IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputSource(
      sources_combined);
  IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputTarget(
      targets_combined);

  // If we automatically set the correspondence estimators, we should clear them now
  if (!manual_correspondence_estimations_set) {
    correspondence_estimations_.clear();
  }

  // By definition, this method will return an empty cloud (for compliance with the ICP
  // API). We can figure out a better solution, if necessary.
  output = PointCloudSource();
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
JointIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    determineRequiredBlobData()
{
  need_source_blob_ = false;
  need_target_blob_ = false;
  // Check estimators
  for (std::size_t i = 0; i < correspondence_estimations_.size(); i++) {
    CorrespondenceEstimationPtr& ce = correspondence_estimations_[i];

    need_source_blob_ |= ce->requiresSourceNormals();
    need_target_blob_ |= ce->requiresTargetNormals();
    // Add warnings if necessary
    if (ce->requiresSourceNormals() && !source_has_normals_) {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects source normals, "
               "but we can't provide them.\n",
               getClassName().c_str());
    }
    if (ce->requiresTargetNormals() && !target_has_normals_) {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects target normals, "
               "but we can't provide them.\n",
               getClassName().c_str());
    }
  }
  // Check rejectors
  for (std::size_t i = 0; i < correspondence_rejectors_.size(); i++) {
    registration::CorrespondenceRejector::Ptr& rej = correspondence_rejectors_[i];
    need_source_blob_ |= rej->requiresSourcePoints();
    need_source_blob_ |= rej->requiresSourceNormals();
    need_target_blob_ |= rej->requiresTargetPoints();
    need_target_blob_ |= rej->requiresTargetNormals();
    if (rej->requiresSourceNormals() && !source_has_normals_) {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects source "
               "normals, but we can't provide them.\n",
               getClassName().c_str(),
               rej->getClassName().c_str());
    }
    if (rej->requiresTargetNormals() && !target_has_normals_) {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects target "
               "normals, but we can't provide them.\n",
               getClassName().c_str(),
               rej->getClassName().c_str());
    }
  }
}

} // namespace pcl

#endif /* PCL_REGISTRATION_IMPL_JOINT_ICP_HPP_ */
