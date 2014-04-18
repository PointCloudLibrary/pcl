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

#ifndef PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_HPP_
#define PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_HPP_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::setSourceFeatures (const FeatureCloudConstPtr &features)
{
  if (features == NULL || features->empty ())
  {
    PCL_ERROR ("[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  input_features_ = features;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::setTargetFeatures (const FeatureCloudConstPtr &features)
{
  if (features == NULL || features->empty ())
  {
    PCL_ERROR ("[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  target_features_ = features;
  feature_tree_->setInputCloud (target_features_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::selectSamples (
    const PointCloudSource &cloud, int nr_samples, 
    std::vector<int> &sample_indices)
{
  if (nr_samples > static_cast<int> (cloud.points.size ()))
  {
    PCL_ERROR ("[pcl::%s::selectSamples] ", getClassName ().c_str ());
    PCL_ERROR ("The number of samples (%d) must not be greater than the number of points (%lu)!\n",
               nr_samples, cloud.points.size ());
    return;
  }

  // Iteratively draw random samples until nr_samples is reached
  sample_indices.clear ();
  std::vector<bool> sampled_indices (cloud.points.size (), false);
  while (static_cast<int> (sample_indices.size ()) < nr_samples)
  {
    // Choose a unique sample at random
    int sample_index;
    do
    {
      sample_index = getRandomIndex (static_cast<int> (cloud.points.size ()));
    }
    while (sampled_indices[sample_index]);
    
    // Mark index as sampled
    sampled_indices[sample_index] = true;
    
    // Store
    sample_indices.push_back (sample_index);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::findSimilarFeatures (
    const FeatureCloud &input_features, const std::vector<int> &sample_indices, 
    std::vector<int> &corresponding_indices)
{
  std::vector<int> nn_indices (k_correspondences_);
  std::vector<float> nn_distances (k_correspondences_);

  corresponding_indices.resize (sample_indices.size ());
  for (size_t i = 0; i < sample_indices.size (); ++i)
  {
    // Find the k features nearest to input_features.points[sample_indices[i]]
    feature_tree_->nearestKSearch (input_features, sample_indices[i], k_correspondences_, nn_indices, nn_distances);

    // Select one at random and add it to corresponding_indices
    if (k_correspondences_ == 1)
    {
      corresponding_indices[i] = nn_indices[0];
    }
    else
    {
      int random_correspondence = getRandomIndex (k_correspondences_);
      corresponding_indices[i] = nn_indices[random_correspondence];
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
  // Some sanity checks first
  if (!input_features_)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("No source features were given! Call setSourceFeatures before aligning.\n");
    return;
  }
  if (!target_features_)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("No target features were given! Call setTargetFeatures before aligning.\n");
    return;
  }

  if (input_->size () != input_features_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               input_->size (), input_features_->size ());
    return;
  }

  if (target_->size () != target_features_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               target_->size (), target_features_->size ());
    return;
  }

  if (inlier_fraction_ < 0.0f || inlier_fraction_ > 1.0f)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("Illegal inlier fraction %f, must be in [0,1]!\n",
               inlier_fraction_);
    return;
  }
  
  const float similarity_threshold = correspondence_rejector_poly_->getSimilarityThreshold ();
  if (similarity_threshold < 0.0f || similarity_threshold >= 1.0f)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("Illegal prerejection similarity threshold %f, must be in [0,1[!\n",
               similarity_threshold);
    return;
  }
  
  // Initialize prerejector (similarity threshold already set to default value in constructor)
  correspondence_rejector_poly_->setInputSource (input_);
  correspondence_rejector_poly_->setInputTarget (target_);
  correspondence_rejector_poly_->setCardinality (nr_samples_);
  std::vector<bool> accepted (input_->size (), false); // Indices of sampled points that passed prerejection
  int num_rejections = 0; // For debugging
  
  // Initialize results
  final_transformation_ = guess;
  inliers_.clear ();
  float lowest_error = std::numeric_limits<float>::max ();
  converged_ = false;
  
  // Temporaries
  std::vector<int> inliers;
  float inlier_fraction;
  float error;
  
  // If guess is not the Identity matrix we check it
  if (!guess.isApprox (Eigen::Matrix4f::Identity (), 0.01f))
  {
    getFitness (inliers, error);
    inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());
    error /= static_cast<float> (inliers.size ());
    
    if (inlier_fraction >= inlier_fraction_ && error < lowest_error)
    {
      inliers_ = inliers;
      lowest_error = error;
      converged_ = true;
    }
  }
  
  // Start
  for (int i = 0; i < max_iterations_; ++i)
  {
    // Temporary containers
    std::vector<int> sample_indices (nr_samples_);
    std::vector<int> corresponding_indices (nr_samples_);
    
    // Draw nr_samples_ random samples
    selectSamples (*input_, nr_samples_, sample_indices);
    
    // Check if all sampled points already been accepted
    bool samples_accepted = true;
    for (unsigned int j = 0; j < sample_indices.size(); ++j) {
      if (!accepted[j]) {
        samples_accepted = false;
        break;
      }
    }

    // All points have already been accepted, avoid
    if (samples_accepted)
      continue;
    
    // Find corresponding features in the target cloud
    findSimilarFeatures (*input_features_, sample_indices, corresponding_indices);
    
    // Apply prerejection
    if (!correspondence_rejector_poly_->thresholdPolygon (sample_indices, corresponding_indices)) {
      ++num_rejections;
      continue;
    }

    // Estimate the transform from the correspondences, write to transformation_
    transformation_estimation_->estimateRigidTransformation (*input_, sample_indices, *target_, corresponding_indices, transformation_);
    
    // Take a backup of previous result
    const Matrix4 final_transformation_prev = final_transformation_;
    
    // Set final result to current transformation
    final_transformation_ = transformation_;
    
    // Transform the input and compute the error (uses input_ and final_transformation_)
    getFitness (inliers, error);
    
    // Restore previous result
    final_transformation_ = final_transformation_prev;

    // If the new fit is better, update results
    inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());
    
    if (inlier_fraction >= inlier_fraction_) {
      // Mark the sampled points accepted
      for (int j = 0; j < nr_samples_; ++j)
        accepted[j] = true;
      
      // Update result if pose hypothesis is better
      if (error < lowest_error) {
        inliers_ = inliers;
        lowest_error = error;
        converged_ = true;
        final_transformation_ = transformation_;
      }
    }
  }

  // Apply the final transformation
  if (converged_)
    transformPointCloud (*input_, output, final_transformation_);
  
  // Debug output
  PCL_DEBUG("[pcl::%s::computeTransformation] Rejected %i out of %i generated pose hypotheses.\n",
            getClassName ().c_str (), num_rejections, max_iterations_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::getFitness (std::vector<int>& inliers, float& fitness_score)
{
  // Initialize variables
  inliers.clear ();
  inliers.reserve (input_->size ());
  fitness_score = 0.0f;
  
  // Use squared distance for comparison with NN search results
  const float max_range = corr_dist_threshold_ * corr_dist_threshold_;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  input_transformed.resize (input_->size ());
  transformPointCloud (*input_, input_transformed, final_transformation_);
  
  // For each point in the source dataset
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
    
    // Check if point is an inlier
    if (nn_dists[0] < max_range)
    {
      // Errors
      const float dx = input_transformed.points[i].x - target_->points[nn_indices[0]].x;
      const float dy = input_transformed.points[i].y - target_->points[nn_indices[0]].y;
      const float dz = input_transformed.points[i].z - target_->points[nn_indices[0]].z;
      
      // Update inliers
      inliers.push_back (static_cast<int> (i));
      
      // Update fitness score
      fitness_score += dx*dx + dy*dy + dz*dz;
    }
  }

  // Calculate MSE
  if (inliers.size () > 0)
    fitness_score /= static_cast<float> (inliers.size ());
  else
    fitness_score = std::numeric_limits<float>::max ();
}

#endif

