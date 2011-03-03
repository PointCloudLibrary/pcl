/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_HPP_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_HPP_

template <typename PointT>
inline void pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::applyRejection(pcl::registration::Correspondences &correspondences)
{
  std::vector<int> source_indices;
  std::vector<int> target_indices;
  source_indices.resize(input_correspondences_->size());
  target_indices.resize(input_correspondences_->size());
  for (unsigned int i = 0; i < input_correspondences_->size(); ++i)
  {
    source_indices[i] = input_correspondences_->at(i).indexQuery;
    target_indices[i] = input_correspondences_->at(i).indexMatch;
  }

   // from pcl/registration/icp.hpp:
   std::vector<int> source_indices_good;
   std::vector<int> target_indices_good;
   {
     // From the set of correspondences found, attempt to remove outliers
     // Create the registration model
     typedef typename pcl::SampleConsensusModelRegistration<PointT>::Ptr SampleConsensusModelRegistrationPtr;
     SampleConsensusModelRegistrationPtr model;
     model.reset (new pcl::SampleConsensusModelRegistration<PointT> (input_, source_indices));
     // Pass the target_indices
     model->setInputTarget (target_, target_indices);
     // Create a RANSAC model
     pcl::RandomSampleConsensus<PointT> sac (model, inlier_threshold_);
     sac.setMaxIterations (max_iterations_);

     // Compute the set of inliers
     if (!sac.computeModel ())
     {
       correspondences = *input_correspondences_;
       best_transformation_.setIdentity();
     }
     else
     {
       std::vector<int> inliers;
       sac.getInliers (inliers);
       correspondences.resize(inliers.size());
       for (size_t i = 0; i < inliers.size (); ++i)
         correspondences[i] = input_correspondences_->at(inliers[i]);

       // get best transformation
       Eigen::VectorXf model_coefficients;
       sac.getModelCoefficients(model_coefficients);
       best_transformation_.row (0) = model_coefficients.segment<4>(0);
       best_transformation_.row (1) = model_coefficients.segment<4>(4);
       best_transformation_.row (2) = model_coefficients.segment<4>(8);
       best_transformation_.row (3) = model_coefficients.segment<4>(12);
     }


   }
}

template <typename PointT>
void pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::getCorrespondences(const pcl::registration::Correspondences& original_correspondences, pcl::registration::Correspondences& remaining_correspondences)
{
  std::vector<int> source_indices;
  std::vector<int> target_indices;
  source_indices.resize(original_correspondences.size());
  target_indices.resize(original_correspondences.size());
  for (unsigned int i = 0; i < original_correspondences.size(); ++i)
  {
    source_indices[i] = original_correspondences.at(i).indexQuery;
    target_indices[i] = original_correspondences.at(i).indexMatch;
  }

   // from pcl/registration/icp.hpp:
   std::vector<int> source_indices_good;
   std::vector<int> target_indices_good;
   {
     // From the set of correspondences found, attempt to remove outliers
     // Create the registration model
     typedef typename pcl::SampleConsensusModelRegistration<PointT>::Ptr SampleConsensusModelRegistrationPtr;
     SampleConsensusModelRegistrationPtr model;
     model.reset (new pcl::SampleConsensusModelRegistration<PointT> (input_, source_indices));
     // Pass the target_indices
     model->setInputTarget (target_, target_indices);
     // Create a RANSAC model
     pcl::RandomSampleConsensus<PointT> sac (model, inlier_threshold_);
     sac.setMaxIterations (max_iterations_);

     // Compute the set of inliers
     if (!sac.computeModel ())
     {
       remaining_correspondences = original_correspondences;
       best_transformation_.setIdentity();
     }
     else
     {
       std::vector<int> inliers;
       sac.getInliers (inliers);
       remaining_correspondences.resize(inliers.size());
       for (size_t i = 0; i < inliers.size (); ++i)
         remaining_correspondences[i] = original_correspondences.at(inliers[i]);

       // get best transformation
       Eigen::VectorXf model_coefficients;
       sac.getModelCoefficients(model_coefficients);
       best_transformation_.row (0) = model_coefficients.segment<4>(0);
       best_transformation_.row (1) = model_coefficients.segment<4>(4);
       best_transformation_.row (2) = model_coefficients.segment<4>(8);
       best_transformation_.row (3) = model_coefficients.segment<4>(12);
     }


   }
}

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_HPP_ */
