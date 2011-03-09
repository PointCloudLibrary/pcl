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
 * $Id: ia_ransac.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void
pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::setSourceFeatures (
      const FeatureCloudConstPtr &features)
{
  if (features == NULL || features->points.empty ())
  {
    ROS_ERROR ("[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!", getClassName ().c_str ());
    return;
  }
  input_features_ = features;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void
pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::setTargetFeatures (
      const FeatureCloudConstPtr &features)
{
  if (features == NULL || features->points.empty ())
  {
    ROS_ERROR ("[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!", getClassName ().c_str ());
    return;
  }
  target_features_ = features;
  feature_tree_->setInputCloud (target_features_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void
pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::selectSamples (
      const PointCloudSource &cloud, int nr_samples, float min_sample_distance, std::vector<int> &sample_indices)
{
  if (nr_samples > (int) cloud.points.size ())
  {
    ROS_ERROR ("[pcl::%s::selectSamples] The number of samples (%d) must not be greater than the number of points (%d)!",
               getClassName ().c_str (), nr_samples, (int) cloud.points.size ());
    return;
  }

  // Iteratively draw random samples until nr_samples is reached
  int iterations_without_a_sample = 0;
  int max_iterations_without_a_sample = 3 * cloud.points.size ();
  sample_indices.clear ();
  while ((int) sample_indices.size () < nr_samples)
  {
    // Choose a sample at random
    int sample_index = getRandomIndex (cloud.points.size ());

    // Check to see if the sample is 1) unique and 2) far away from the other samples
    bool valid_sample = true;
    for (size_t i = 0; i < sample_indices.size (); ++i)
    {
      float distance_between_samples = euclideanDistance (cloud.points[sample_index], cloud.points[sample_indices[i]]);

      if (sample_index == sample_indices[i] || distance_between_samples < min_sample_distance)
      {
        valid_sample = false;
        break;
      }
    }

    // If the sample is valid, add it to the output
    if (valid_sample)
    {
      sample_indices.push_back (sample_index);
      iterations_without_a_sample = 0;
    }
    else
    {
      ++iterations_without_a_sample;
    }

    // If no valid samples can be found, relax the inter-sample distance requirements
    if (iterations_without_a_sample >= max_iterations_without_a_sample)
    {
      ROS_WARN ("[pcl::%s::selectSamples] No valid sample found after %d iterations. Relaxing min_sample_distance_ to %f", 
                getClassName ().c_str (), iterations_without_a_sample, 0.5*min_sample_distance);
      min_sample_distance_ *= 0.5;
      min_sample_distance = min_sample_distance_;
      iterations_without_a_sample = 0;
    }
  }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void
pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::findSimilarFeatures (
      const FeatureCloud &input_features, const std::vector<int> &sample_indices, std::vector<int> &corresponding_indices)
{
  const int k = 10;
  std::vector<int> nn_indices (k);
  std::vector<float> nn_distances (k);

  corresponding_indices.resize (sample_indices.size ());
  for (size_t i = 0; i < sample_indices.size (); ++i)
  {
    // Find the k features nearest to input_features.points[sample_indices[i]]
    feature_tree_->nearestKSearch (input_features, sample_indices[i], k, nn_indices, nn_distances);

    // Select one at random and add it to corresponding_indices
    int random_correspondence = getRandomIndex (k);
    corresponding_indices[i] = nn_indices[random_correspondence];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> float
pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::computeErrorMetric (
      const PointCloudSource &cloud, float threshold)
{
  std::vector<int> nn_index (1);
  std::vector<float> nn_distance (1);

  float error = 0;

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    // Find the distance between cloud.points[i] and its nearest neighbor in the target point cloud
    tree_->nearestKSearch (cloud, i, 1, nn_index, nn_distance);

    /*
    // Huber penalty measure
    float e = nn_distance[0];
    if (e <= threshold)
      error += 0.5 * e*e; 
    else
      error += 0.5 * threshold * (2.0 * fabs (e) - threshold);
    */

    // Truncated error
    float e = nn_distance[0];
    if (e <= threshold)
      error += e / threshold;
    else
      error += 1.0;
  }
  return (error);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void
pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::computeTransformation (PointCloudSource &output)
{
  if (!input_features_)
  {
    ROS_ERROR ("[pcl::%s::computeTransformation] No source features were given! Call setSourceFeatures before aligning.", 
               getClassName ().c_str ());
    return;
  }
  if (!target_features_)
  {
    ROS_ERROR ("[pcl::%s::computeTransformation] No target features were given! Call setTargetFeatures before aligning", 
               getClassName ().c_str ());
    return;
  }

  std::vector<int> sample_indices (nr_samples_);
  std::vector<int> corresponding_indices (nr_samples_);
  PointCloudSource input_transformed;
  float error, lowest_error (0);

  final_transformation_ = Eigen::Matrix4f::Identity ();

  for (int i_iter = 0; i_iter < max_iterations_; ++i_iter)
  {
    // Draw nr_samples_ random samples
    selectSamples (*input_, nr_samples_, min_sample_distance_, sample_indices);

    // Find corresponding features in the target cloud
    findSimilarFeatures (*input_features_, sample_indices, corresponding_indices);

    // Estimate the transform from the samples to their corresponding points
    estimateRigidTransformationSVD (*input_, sample_indices, *target_, corresponding_indices, transformation_);

    // Tranform the data and compute the error
    transformPointCloud (*input_, input_transformed, transformation_);
    error = computeErrorMetric (input_transformed, corr_dist_threshold_);

    // If the new error is lower, update the final transformation
    if (i_iter == 0 || error < lowest_error)
    {
      lowest_error = error;
      final_transformation_ = transformation_;
    }
  }

  // Apply the final transformation
  transformPointCloud (*input_, output, final_transformation_);
}
