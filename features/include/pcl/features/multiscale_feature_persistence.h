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
#include <list>

namespace pcl
{
//  template <typename PointInT, typename PointFeature>
//  class MultiscaleFeaturePersistence : public PCLBase<PointInT>
//  {
//    public:
//      typedef PointCloud<PointFeature> FeatureCloud;
//      typedef typename PointCloud<PointFeature>::Ptr FeatureCloudPtr;
//      typedef typename Feature<PointInT, PointFeature>::Ptr FeatureEstimatorPtr;
//
//      using PCLBase<PointInT>::input_;
//
//      /** \brief Empty constructor */
//      MultiscaleFeaturePersistence ()
//      {
//        feature_estimator = FeatureEstimatorPtr ();
//      };
//
//      /** \brief Constructor for setting the scales
//       * \param a_scale_values vector of scales to determine the characteristic of each scaling step
//       * \note this parameter will have to be individualized for each implementation of the class
//       * */
//      MultiscaleFeaturePersistence (std::vector<float>& a_scale_values)
//        : scale_values (a_scale_values)
//      {
//        feature_estimator = FeatureEstimatorPtr ();
//      };
//
//      /** \brief Setter method for the scale vector
//       * \param a_scale_values vector of scales to determine the characteristic of each scaling step
//       * \note this parameter will have to be individualized for each implementation of the class
//       */
//      void
//      setScalesVector (std::vector<float>& a_scale_values) { scale_values = a_scale_values; };
//
//      /** \brief Setter method for the feature estimator
//       * \param a_feature_estimator pointer to the feature estimator instance that will be used
//       * \note the feature estimator instance should already have the input data given beforehand
//       * and everything set, ready to be given the compute () command
//       */
//      void
//      setFeatureEstimator (FeatureEstimatorPtr a_feature_estimator)
//      { feature_estimator = a_feature_estimator; };
//
//      /** \brief Method that calls computeFeatureAtScale () for each scale parameter */
//      void
//      computeFeaturesAtAllScales ();
//
//      virtual void
//      computeFeatureAtScale (float &scale,
//                             FeatureCloudPtr &features) = 0;
//
//      /**
//       * @todo find a way of dealing with these
//       * void determinePersistentFeatures (output something ???)
//       */
//
//
//    protected:
//      /** \brief checks if all the necessary input was given and the computations can
//       * successfully start
//       */
//      bool
//      initCompute ();
//
//      /** \brief function that calculates the scalar difference between two features
//       * \return the difference as a floating point type
//       */
//      virtual float
//      featureDifference (const PointFeature &a,
//                         const PointFeature &b) = 0;
//
//
//      /** \brief the general parameter for determining each scale level */
//      std::vector<float> scale_values;
//
//      /** \brief the feature estimator that will be used to determine the feature set at each scale level */
//      FeatureEstimatorPtr feature_estimator;
//
//      std::vector<FeatureCloudPtr> features_at_scale;
//  };
//
//
//  template <typename PointInT, typename PointFeature>
//  class MultiscaleLocalFeaturePersistence : public MultiscaleFeaturePersistence<PointInT, PointFeature>
//  {
//    public:
//      using MultiscaleFeaturePersistence<PointInT, PointFeature>::feature_estimator;
//      using MultiscaleFeaturePersistence<PointInT, PointFeature>::features_at_scale;
//      typedef typename PointCloud<PointFeature>::Ptr FeatureCloudPtr;
//      MultiscaleLocalFeaturePersistence ();
//
//    protected:
//      void
//      extractUniqueFeatures ();
//
//      void
//      calculateMeanFeature (PointFeature &mean);
//
//      void
//      computeFeatureAtScale (float &scale,
//                             FeatureCloudPtr &features);
//
//      void
//      determinePersistentFeatures ();
//  };



  template <typename PointT, typename PointNT, typename PointFeature>
  class FPFHMultiscaleFeaturePersistence : public PCLBase<PointT>
  {
    public:
      using PCLBase<PointT>::input_;
      typedef PointCloud<PointT> InputCloud;
      typedef typename PointCloud<PointT>::Ptr InputCloudPtr;
      typedef PointCloud<PointNT> NormalCloud;
      typedef typename PointCloud<PointNT>::Ptr NormalCloudPtr;
      typedef PointCloud<PointFeature> FeatureCloud;
      typedef typename PointCloud<PointFeature>::Ptr FeatureCloudPtr;
      typedef FPFHEstimation<PointT, PointNT, PointFeature> FeatureEstimator;
      typedef typename FPFHEstimation<PointT, PointNT, PointFeature>::Ptr FeatureEstimatorPtr;

      FPFHMultiscaleFeaturePersistence (float a_alpha = 1.0)
        : alpha (a_alpha)
      {};

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
