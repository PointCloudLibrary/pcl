////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT>
void SubsetSAC_IA<PointSource, PointTarget, FeatureT>::
selectSamplesSubset (int nr_samples, float min_sample_distance,
                     std::vector<int> &sample_indices_features,
                     std::vector<int> &sample_indices_cloud)
{
  const PointCloudSource &cloud = *input_;
  if (nr_samples > (int) source_indices_->size ())
  {
    PCL_ERROR ("[pcl::%s::selectSamples] ", getClassName ().c_str ());
    PCL_ERROR ("The number of samples (%d) must not be greater than the number of indices (%d)!\n",
               nr_samples, (int) source_indices_->size ());
    return;
  }

  // Iteratively draw random samples until nr_samples is reached
  int iterations_without_a_sample = 0;
  int max_iterations_without_a_sample = 3 * source_indices_->size ();
  sample_indices_features.clear ();
  sample_indices_cloud.clear ();
  while ((int) sample_indices_features.size () < nr_samples)
  {
    // Choose a sample at random
    int sample_index_features = getRandomIndex (source_indices_->size ());
    int sample_index_cloud = (*source_indices_)[sample_index_features];

    // Check to see if the sample is 1) unique and 2) far away from the other samples
    bool valid_sample = true;
    for (size_t i = 0; i < sample_indices_features.size (); ++i)
    {
      float distance_between_samples = euclideanDistance (cloud.points[sample_index_cloud], cloud.points[sample_indices_cloud[i]]);

      if (sample_index_features == sample_indices_features[i] || distance_between_samples < min_sample_distance)
      {
        valid_sample = false;
        break;
      }
    }

    // If the sample is valid, add it to the output
    if (valid_sample)
    {
      sample_indices_features.push_back (sample_index_features);
      sample_indices_cloud.push_back (sample_index_cloud);
      iterations_without_a_sample = 0;
    }
    else
    {
      ++iterations_without_a_sample;
    }

    // If no valid samples can be found, relax the inter-sample distance requirements
    if (iterations_without_a_sample >= max_iterations_without_a_sample)
    {
      PCL_WARN ("[pcl::%s::selectSamples] ", getClassName ().c_str ());
      PCL_WARN ("No valid sample found after %d iterations. Relaxing min_sample_distance_ to %f\n",
                iterations_without_a_sample, 0.5*min_sample_distance);

      min_sample_distance_ *= 0.5;
      min_sample_distance = min_sample_distance_;
      iterations_without_a_sample = 0;
    }
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT>
void SubsetSAC_IA<PointSource, PointTarget, FeatureT>::
findSimilarFeaturesSubset (const std::vector<int> &sample_indices_features,
                           std::vector<int> &corresponding_indices_cloud)
{
  const FeatureCloud &input_features = *input_features_;
  std::vector<int> nn_indices (k_correspondences_);
  std::vector<float> nn_distances (k_correspondences_);

  corresponding_indices_cloud.resize (sample_indices_features.size ());
  for (size_t i = 0; i < sample_indices_features.size (); ++i)
  {
    // Find the k features nearest to input_features.points[sample_indices[i]]
    feature_tree_->nearestKSearch (input_features, sample_indices_features[i], k_correspondences_, nn_indices, nn_distances);

    // Select one at random and add it to corresponding_indices
    int random_correspondence = getRandomIndex (k_correspondences_);
    corresponding_indices_cloud[i] = (*target_indices_)[nn_indices[random_correspondence]];
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT>
void SubsetSAC_IA<PointSource, PointTarget, FeatureT>::
computeTransformation (PointCloudSource &output)
{
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

  if (!error_functor_)
  {
    error_functor_.reset (new TruncatedError (min_sample_distance_));
  }

  std::vector<int> sample_indices_features (nr_samples_);
  std::vector<int> sample_indices_cloud (nr_samples_);
  std::vector<int> corresponding_indices_cloud (nr_samples_);
  PointCloudSource input_transformed;
  float error, lowest_error (0);

  final_transformation_ = Eigen::Matrix4f::Identity ();

  for (int i_iter = 0; i_iter < max_iterations_; ++i_iter)
  {
    // Draw nr_samples_ random samples from source
    selectSamplesSubset (nr_samples_, min_sample_distance_, sample_indices_features, sample_indices_cloud);

    // Find corresponding features in the target cloud
    findSimilarFeaturesSubset (sample_indices_features, corresponding_indices_cloud);

    // Estimate the transform from the samples to their corresponding points
    transformation_estimation_->estimateRigidTransformation (*input_, sample_indices_cloud, *target_, corresponding_indices_cloud, transformation_);

    // Tranform the data and compute the error
    transformPointCloud (*input_, input_transformed, transformation_);
    error = computeErrorMetric (input_transformed, corr_dist_threshold_);

    // If the new error is lower, update the final transformation
    if (i_iter == 0 || error < lowest_error)
    {
      lowest_error = error;
      final_transformation_ = transformation_;
      if (update_visualizer_ != 0)
      {
        update_visualizer_(input_transformed, sample_indices_cloud, *target_, corresponding_indices_cloud );
      }
    }
  }

  // Apply the final transformation
  transformPointCloud (*input_, output, final_transformation_);
}
