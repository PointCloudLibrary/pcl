/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
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
 *  $Id$
 */

#ifndef PCL_FEATURES_IMPL_MULTISCALE_FEATURE_PERSISTENCE_H_
#define PCL_FEATURES_IMPL_MULTISCALE_FEATURE_PERSISTENCE_H_

#include <pcl/features/multiscale_feature_persistence.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature>
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::MultiscaleFeaturePersistence () : 
  scale_values_ (), 
  alpha_ (0), 
  distance_metric_ (L1),
  feature_estimator_ (),
  features_at_scale_ (),
  features_at_scale_vectorized_ (),
  mean_feature_ (),
  feature_representation_ (),
  unique_features_indices_ (),
  unique_features_table_ ()
{
  feature_representation_.reset (new DefaultPointRepresentation<PointFeature>);
  // No input is needed, hack around the initCompute () check from PCLBase
  input_.reset (new pcl::PointCloud<PointSource> ());
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature> bool
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::initCompute ()
{
  if (!PCLBase<PointSource>::initCompute ())
  {
    PCL_ERROR ("[pcl::MultiscaleFeaturePersistence::initCompute] PCLBase::initCompute () failed - no input cloud was given.\n");
    return false;
  }
  if (!feature_estimator_)
  {
    PCL_ERROR ("[pcl::MultiscaleFeaturePersistence::initCompute] No feature estimator was set\n");
    return false;
  }
  if (scale_values_.empty ())
  {
    PCL_ERROR ("[pcl::MultiscaleFeaturePersistence::initCompute] No scale values were given\n");
    return false;
  }

  mean_feature_.resize (feature_representation_->getNumberOfDimensions ());

  return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature> void
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::computeFeaturesAtAllScales ()
{
  features_at_scale_.resize (scale_values_.size ());
  features_at_scale_vectorized_.resize (scale_values_.size ());
  for (size_t scale_i = 0; scale_i < scale_values_.size (); ++scale_i)
  {
    FeatureCloudPtr feature_cloud (new FeatureCloud ());
    computeFeatureAtScale (scale_values_[scale_i], feature_cloud);
    features_at_scale_[scale_i] = feature_cloud;

    // Vectorize each feature and insert it into the vectorized feature storage
    std::vector<std::vector<float> > feature_cloud_vectorized (feature_cloud->points.size ());
    for (size_t feature_i = 0; feature_i < feature_cloud->points.size (); ++feature_i)
    {
      std::vector<float> feature_vectorized (feature_representation_->getNumberOfDimensions ());
      feature_representation_->vectorize (feature_cloud->points[feature_i], feature_vectorized);
      feature_cloud_vectorized[feature_i] = feature_vectorized;
    }
    features_at_scale_vectorized_[scale_i] = feature_cloud_vectorized;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature> void
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::computeFeatureAtScale (float &scale,
                                                                                     FeatureCloudPtr &features)
{
   feature_estimator_->setRadiusSearch (scale);
   feature_estimator_->compute (*features);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature> float
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::distanceBetweenFeatures (const std::vector<float> &a,
                                                                                       const std::vector<float> &b)
{
  return (pcl::selectNorm<std::vector<float> > (a, b, static_cast<int> (a.size ()), distance_metric_));
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature> void
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::calculateMeanFeature ()
{
  // Reset mean feature
  for (int i = 0; i < feature_representation_->getNumberOfDimensions (); ++i)
    mean_feature_[i] = 0.0f;

  float normalization_factor = 0.0f;
  for (std::vector<std::vector<std::vector<float> > >::iterator scale_it = features_at_scale_vectorized_.begin (); scale_it != features_at_scale_vectorized_.end(); ++scale_it) {
    normalization_factor += static_cast<float> (scale_it->size ());
    for (std::vector<std::vector<float> >::iterator feature_it = scale_it->begin (); feature_it != scale_it->end (); ++feature_it)
      for (int dim_i = 0; dim_i < feature_representation_->getNumberOfDimensions (); ++dim_i)
        mean_feature_[dim_i] += (*feature_it)[dim_i];
  }

  for (int dim_i = 0; dim_i < feature_representation_->getNumberOfDimensions (); ++dim_i)
    mean_feature_[dim_i] /= normalization_factor;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature> void
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::extractUniqueFeatures ()
{
  unique_features_indices_.resize (scale_values_.size ());
  unique_features_table_.resize (scale_values_.size ());
  for (size_t scale_i = 0; scale_i < features_at_scale_vectorized_.size (); ++scale_i)
  {
    // Calculate standard deviation within the scale
    float standard_dev = 0.0;
    std::vector<float> diff_vector (features_at_scale_vectorized_[scale_i].size ());
    for (size_t point_i = 0; point_i < features_at_scale_vectorized_[scale_i].size (); ++point_i)
    {
      float diff = distanceBetweenFeatures (features_at_scale_vectorized_[scale_i][point_i], mean_feature_);
      standard_dev += diff * diff;
      diff_vector[point_i] = diff;
    }
    standard_dev = sqrtf (standard_dev / static_cast<float> (features_at_scale_vectorized_[scale_i].size ()));
    PCL_DEBUG ("[pcl::MultiscaleFeaturePersistence::extractUniqueFeatures] Standard deviation for scale %f is %f\n", scale_values_[scale_i], standard_dev);

    // Select only points outside (mean +/- alpha * standard_dev)
    std::list<size_t> indices_per_scale;
    std::vector<bool> indices_table_per_scale (features_at_scale_[scale_i]->points.size (), false);
    for (size_t point_i = 0; point_i < features_at_scale_[scale_i]->points.size (); ++point_i)
    {
      if (diff_vector[point_i] > alpha_ * standard_dev)
      {
        indices_per_scale.push_back (point_i);
        indices_table_per_scale[point_i] = true;
      }
    }
    unique_features_indices_[scale_i] = indices_per_scale;
    unique_features_table_[scale_i] = indices_table_per_scale;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointFeature> void
pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>::determinePersistentFeatures (FeatureCloud &output_features,
                                                                                           boost::shared_ptr<std::vector<int> > &output_indices)
{
  if (!initCompute ())
    return;

  // Compute the features for all scales with the given feature estimator
  PCL_DEBUG ("[pcl::MultiscaleFeaturePersistence::determinePersistentFeatures] Computing features ...\n");
  computeFeaturesAtAllScales ();

  // Compute mean feature
  PCL_DEBUG ("[pcl::MultiscaleFeaturePersistence::determinePersistentFeatures] Calculating mean feature ...\n");
  calculateMeanFeature ();

  // Get the 'unique' features at each scale
  PCL_DEBUG ("[pcl::MultiscaleFeaturePersistence::determinePersistentFeatures] Extracting unique features ...\n");
  extractUniqueFeatures ();

  PCL_DEBUG ("[pcl::MultiscaleFeaturePersistence::determinePersistentFeatures] Determining persistent features between scales ...\n");
  // Determine persistent features between scales

/*
  // Method 1: a feature is considered persistent if it is 'unique' in at least 2 different scales
  for (size_t scale_i = 0; scale_i < features_at_scale_vectorized_.size () - 1; ++scale_i)
    for (std::list<size_t>::iterator feature_it = unique_features_indices_[scale_i].begin (); feature_it != unique_features_indices_[scale_i].end (); ++feature_it)
    {
      if (unique_features_table_[scale_i][*feature_it] == true)
      {
        output_features.points.push_back (features_at_scale[scale_i]->points[*feature_it]);
        output_indices->push_back (feature_estimator_->getIndices ()->at (*feature_it));
      }
    }
*/
  // Method 2: a feature is considered persistent if it is 'unique' in all the scales
  for (std::list<size_t>::iterator feature_it = unique_features_indices_.front ().begin (); feature_it != unique_features_indices_.front ().end (); ++feature_it)
  {
    bool present_in_all = true;
    for (size_t scale_i = 0; scale_i < features_at_scale_.size (); ++scale_i)
      present_in_all = present_in_all && unique_features_table_[scale_i][*feature_it];

    if (present_in_all)
    {
      output_features.points.push_back (features_at_scale_.front ()->points[*feature_it]);
      output_indices->push_back (feature_estimator_->getIndices ()->at (*feature_it));
    }
  }

  // Consider that output cloud is unorganized
  output_features.header = feature_estimator_->getInputCloud ()->header;
  output_features.is_dense = feature_estimator_->getInputCloud ()->is_dense;
  output_features.width = static_cast<uint32_t> (output_features.points.size ());
  output_features.height = 1;
}


#define PCL_INSTANTIATE_MultiscaleFeaturePersistence(InT, Feature) template class PCL_EXPORTS pcl::MultiscaleFeaturePersistence<InT, Feature>;

#endif /* PCL_FEATURES_IMPL_MULTISCALE_FEATURE_PERSISTENCE_H_ */
