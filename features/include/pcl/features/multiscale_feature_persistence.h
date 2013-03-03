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

#ifndef PCL_MULTISCALE_FEATURE_PERSISTENCE_H_
#define PCL_MULTISCALE_FEATURE_PERSISTENCE_H_

#include <pcl/pcl_base.h>
#include <pcl/features/feature.h>
#include <pcl/point_representation.h>
#include <pcl/common/norms.h>
#include <list>

namespace pcl
{
  /** \brief Generic class for extracting the persistent features from an input point cloud
   * It can be given any Feature estimator instance and will compute the features of the input
   * over a multiscale representation of the cloud and output the unique ones over those scales.
   *
   * Please refer to the following publication for more details:
   *    Radu Bogdan Rusu, Zoltan Csaba Marton, Nico Blodow, and Michael Beetz
   *    Persistent Point Feature Histograms for 3D Point Clouds
   *    Proceedings of the 10th International Conference on Intelligent Autonomous Systems (IAS-10)
   *    2008, Baden-Baden, Germany
   *
   * \author Alexandru-Eugen Ichim
   */
  template <typename PointSource, typename PointFeature>
  class MultiscaleFeaturePersistence : public PCLBase<PointSource>
  {
    public:
      typedef boost::shared_ptr<MultiscaleFeaturePersistence<PointSource, PointFeature> > Ptr;
      typedef boost::shared_ptr<const MultiscaleFeaturePersistence<PointSource, PointFeature> > ConstPtr;
      typedef pcl::PointCloud<PointFeature> FeatureCloud;
      typedef typename pcl::PointCloud<PointFeature>::Ptr FeatureCloudPtr;
      typedef typename pcl::Feature<PointSource, PointFeature>::Ptr FeatureEstimatorPtr;
      typedef boost::shared_ptr<const pcl::PointRepresentation <PointFeature> > FeatureRepresentationConstPtr;

      using pcl::PCLBase<PointSource>::input_;

      /** \brief Empty constructor */
      MultiscaleFeaturePersistence ();
      
      /** \brief Empty destructor */
      virtual ~MultiscaleFeaturePersistence () {}

      /** \brief Method that calls computeFeatureAtScale () for each scale parameter */
      void
      computeFeaturesAtAllScales ();

      /** \brief Central function that computes the persistent features
       * \param output_features a cloud containing the persistent features
       * \param output_indices vector containing the indices of the points in the input cloud
       * that have persistent features, under a one-to-one correspondence with the output_features cloud
       */
      void
      determinePersistentFeatures (FeatureCloud &output_features,
                                   boost::shared_ptr<std::vector<int> > &output_indices);

      /** \brief Method for setting the scale parameters for the algorithm
       * \param scale_values vector of scales to determine the characteristic of each scaling step
       */
      inline void
      setScalesVector (std::vector<float> &scale_values) { scale_values_ = scale_values; }

      /** \brief Method for getting the scale parameters vector */
      inline std::vector<float>
      getScalesVector () { return scale_values_; }

      /** \brief Setter method for the feature estimator
       * \param feature_estimator pointer to the feature estimator instance that will be used
       * \note the feature estimator instance should already have the input data given beforehand
       * and everything set, ready to be given the compute () command
       */
      inline void
      setFeatureEstimator (FeatureEstimatorPtr feature_estimator) { feature_estimator_ = feature_estimator; };

      /** \brief Getter method for the feature estimator */
      inline FeatureEstimatorPtr
      getFeatureEstimator () { return feature_estimator_; }

      /** \brief Provide a pointer to the feature representation to use to convert features to k-D vectors.
       * \param feature_representation the const boost shared pointer to a PointRepresentation
       */
      inline void
      setPointRepresentation (const FeatureRepresentationConstPtr& feature_representation) { feature_representation_ = feature_representation; }

      /** \brief Get a pointer to the feature representation used when converting features into k-D vectors. */
      inline FeatureRepresentationConstPtr const
      getPointRepresentation () { return feature_representation_; }

      /** \brief Sets the alpha parameter
       * \param alpha value to replace the current alpha with
       */
      inline void
      setAlpha (float alpha) { alpha_ = alpha; }

      /** \brief Get the value of the alpha parameter */
      inline float
      getAlpha () { return alpha_; }

      /** \brief Method for setting the distance metric that will be used for computing the difference between feature vectors
       * \param distance_metric the new distance metric chosen from the NormType enum
       */
      inline void
      setDistanceMetric (NormType distance_metric) { distance_metric_ = distance_metric; }

      /** \brief Returns the distance metric that is currently used to calculate the difference between feature vectors */
      inline NormType
      getDistanceMetric () { return distance_metric_; }


    private:
      /** \brief Checks if all the necessary input was given and the computations can successfully start */
      bool
      initCompute ();


      /** \brief Method to compute the features for the point cloud at the given scale */
      virtual void
      computeFeatureAtScale (float &scale,
                             FeatureCloudPtr &features);


      /** \brief Function that calculates the scalar difference between two features
       * \return the difference as a floating point type
       */
      float
      distanceBetweenFeatures (const std::vector<float> &a,
                               const std::vector<float> &b);

      /** \brief Method that averages all the features at all scales in order to obtain the global mean feature;
       * this value is stored in the mean_feature field
       */
      void
      calculateMeanFeature ();

      /** \brief Selects the so-called 'unique' features from the cloud of features at each level.
       * These features are the ones that fall outside the standard deviation * alpha_
       */
      void
      extractUniqueFeatures ();


      /** \brief The general parameter for determining each scale level */
      std::vector<float> scale_values_;

      /** \brief Parameter that determines if a feature is to be considered unique or not */
      float alpha_;

      /** \brief Parameter that determines which distance metric is to be usedto calculate the difference between feature vectors */
      NormType distance_metric_;

      /** \brief the feature estimator that will be used to determine the feature set at each scale level */
      FeatureEstimatorPtr feature_estimator_;

      std::vector<FeatureCloudPtr> features_at_scale_;
      std::vector<std::vector<std::vector<float> > > features_at_scale_vectorized_;
      std::vector<float> mean_feature_;
      FeatureRepresentationConstPtr feature_representation_;

      /** \brief Two structures in which to hold the results of the unique feature extraction process.
       * They are superfluous with respect to each other, but improve the time performance of the algorithm
       */
      std::vector<std::list<size_t> > unique_features_indices_;
      std::vector<std::vector<bool> > unique_features_table_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/multiscale_feature_persistence.hpp>
#endif

#endif /* PCL_MULTISCALE_FEATURE_PERSISTENCE_H_ */
