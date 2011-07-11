/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 *  $Id$
 */

#ifndef PCL_MULTISCALE_FEATURE_PERSISTENCE_H_
#define PCL_MULTISCALE_FEATURE_PERSISTENCE_H_

#include "pcl/features/fpfh.h"
#include "pcl/point_representation.h"
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
      typedef PointCloud<PointFeature> FeatureCloud;
      typedef typename PointCloud<PointFeature>::Ptr FeatureCloudPtr;
      typedef typename Feature<PointSource, PointFeature>::Ptr FeatureEstimatorPtr;
      typedef boost::shared_ptr<const PointRepresentation <PointFeature> > FeatureRepresentationConstPtr;

      using PCLBase<PointSource>::input_;

      /** \brief Empty constructor */
      MultiscaleFeaturePersistence ()
        : feature_estimator_ ()
      {
        feature_representation_.reset (new DefaultPointRepresentation<PointFeature>);
        // No input is needed, hack around the initCompute () check from PCLBase
        input_.reset (new PointCloud<PointSource> ());
      };

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
      void
      setScalesVector (std::vector<float> &scale_values) { scale_values_ = scale_values; };

      /** \brief Setter method for the feature estimator
       * \param feature_estimator pointer to the feature estimator instance that will be used
       * \note the feature estimator instance should already have the input data given beforehand
       * and everything set, ready to be given the compute () command
       */
      void
      setFeatureEstimator (FeatureEstimatorPtr feature_estimator) { feature_estimator_ = feature_estimator; };

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

    protected:
      /** \brief checks if all the necessary input was given and the computations can successfully start */
      bool
      initCompute ();

    private:
      /** \brief Method to compute the features for the point cloud at the given scale */
      virtual void
      computeFeatureAtScale (float &scale,
                             FeatureCloudPtr &features);


      /** \brief function that calculates the scalar difference between two features
       * \return the difference as a floating point type
       */
      float
      distanceBetweenFeatures (const std::vector<float> &a,
                               const std::vector<float> &b);

      void
      calculateMeanFeature ();

      void
      extractUniqueFeatures ();


      /** \brief the general parameter for determining each scale level */
      std::vector<float> scale_values_;

      float alpha_;

      /** \brief the feature estimator that will be used to determine the feature set at each scale level */
      FeatureEstimatorPtr feature_estimator_;

      std::vector<FeatureCloudPtr> features_at_scale;
      std::vector<std::vector<std::vector<float> > > features_at_scale_vectorized;
      std::vector<float> mean_feature;
      FeatureRepresentationConstPtr feature_representation_;

      /// two structures in which to hold the results of the unique feature extraction process
      /// they are superfluous wrt to each other, but improve the time performance of the algorithm
      std::vector <std::list<size_t> > unique_features_indices;
      std::vector <std::vector<bool> > unique_features_table;
  };




  template <typename PointT, typename PointNT, typename PointFeature>
  class FPFHMultiscaleFeaturePersistence : public PCLBase<PointT>
  {
    public:
      using PCLBase<PointT>::input_;
      typedef pcl::PointCloud<PointT> InputCloud;
      typedef typename pcl::PointCloud<PointT>::Ptr InputCloudPtr;
      typedef pcl::PointCloud<PointNT> NormalCloud;
      typedef typename pcl::PointCloud<PointNT>::Ptr NormalCloudPtr;
      typedef pcl::PointCloud<PointFeature> FeatureCloud;
      typedef typename pcl::PointCloud<PointFeature>::Ptr FeatureCloudPtr;
      typedef FPFHEstimation<PointT, PointNT, PointFeature> FeatureEstimator;
      typedef typename FPFHEstimation<PointT, PointNT, PointFeature>::Ptr FeatureEstimatorPtr;

      FPFHMultiscaleFeaturePersistence (std::vector<float> &a_scale_values,
                                        float a_alpha = 1.0)
        : scale_values (a_scale_values),
          alpha (a_alpha)
      {};

      /** \brief Setter method for the scale vector
       * \param a_scale_values vector of scales to determine the radius of the feature
       * neighborhood at each scaling step
       */
      void
      setScalesVector (std::vector<float>& a_scale_values) { scale_values = a_scale_values; };

      void
      setInputNormals (NormalCloudPtr &a_normals) { normals_ = a_normals; };


      void
      determinePersistentFeatures (FeatureCloudPtr &output_features,
                                   InputCloudPtr &output_locations);


    private:
      bool
      initCompute ();

      float
      compareFunction (const PointFeature &a,
                       const PointFeature &b);

      void
      calculateMeanFeature (PointFeature &mean);

      void
      computeFeatureAtScale (float &scale,
                             FeatureCloudPtr &output_features);

      void
      extractUniqueFeatures ();


      FeatureEstimatorPtr feature_estimator;
      std::vector<FeatureCloudPtr> features_at_scale;
      PointFeature mean_feature;

      /// two structures in which to hold the results of the unique feature extraction process
      /// they are superfluous wrt to each other, but improve the time performance of the algorithm
      std::vector <std::list<size_t> > unique_features_indices;
      std::vector <std::vector<bool> > unique_features_table;

      std::vector<float> scale_values;
      NormalCloudPtr normals_;
      float alpha;
  };


}


#endif /* PCL_MULTISCALE_FEATURE_PERSISTENCE_H_ */
