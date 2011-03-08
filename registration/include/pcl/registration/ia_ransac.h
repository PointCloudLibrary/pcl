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
#ifndef IA_RANSAC_H_
#define IA_RANSAC_H_

// PCL includes
#include "pcl/registration/registration.h"

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b SampleConsensusInitialAlignment is an implementation of the initial alignment algorithm described in
   *  section IV of "Fast Point Feature Histograms (FPFH) for 3D Registration," Rusu et al.
    * \author Radu Bogdan Rusu, Michael Dixon
    */
  template <typename PointSource, typename PointTarget, typename FeatureT>
  class SampleConsensusInitialAlignment : public Registration<PointSource, PointTarget>
  {
    using Registration<PointSource, PointTarget>::reg_name_;
    using Registration<PointSource, PointTarget>::getClassName;
    using Registration<PointSource, PointTarget>::input_;
    using Registration<PointSource, PointTarget>::indices_;
    using Registration<PointSource, PointTarget>::target_;
    using Registration<PointSource, PointTarget>::final_transformation_;
    using Registration<PointSource, PointTarget>::transformation_;
    using Registration<PointSource, PointTarget>::corr_dist_threshold_;
    using Registration<PointSource, PointTarget>::min_number_correspondences_;
    using Registration<PointSource, PointTarget>::max_iterations_;
    using Registration<PointSource, PointTarget>::tree_;

    typedef typename Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;
    typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
    typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

    typedef typename Registration<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;

    typedef PointIndices::Ptr PointIndicesPtr;
    typedef PointIndices::ConstPtr PointIndicesConstPtr;

    typedef pcl::PointCloud<FeatureT> FeatureCloud;
    typedef typename FeatureCloud::Ptr FeatureCloudPtr;
    typedef typename FeatureCloud::ConstPtr FeatureCloudConstPtr;

    typedef typename KdTreeFLANN<FeatureT>::Ptr FeatureKdTreePtr; 

    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor. */
      SampleConsensusInitialAlignment () : nr_samples_(3)
      {
        reg_name_ = "SampleConsensusInitialAlignment";
        feature_tree_ = boost::make_shared<pcl::KdTreeFLANN<FeatureT> > ();
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a boost shared pointer to the source point cloud's feature descriptors
        * \param features the source point cloud's features
        */
      void setSourceFeatures (const FeatureCloudConstPtr &features);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a pointer to the source point cloud's features */
      inline FeatureCloudConstPtr const getSourceFeatures () { return (input_features_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a boost shared pointer to the target point cloud's feature descriptors
        * \param features the target point cloud's features
        */
      void setTargetFeatures (const FeatureCloudConstPtr &features);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a pointer to the target point cloud's features */
      inline FeatureCloudConstPtr const getTargetFeatures () { return (target_features_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the minimum distances between samples
        * \param min_sample_distance the minimum distances between samples
        */
      void setMinSampleDistance (float min_sample_distance) { min_sample_distance_ = min_sample_distance; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the minimum distances between samples, as set by the user */
      float getMinSampleDistance () { return (min_sample_distance_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the number of samples to use during each iteration
        * \param nr_samples the number of samples to use during each iteration
        */
      void setNumberOfSamples (int nr_samples) { nr_samples_ = nr_samples; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the number of samples to use during each iteration, as set by the user */
      int getNumberOfSamples () { return (nr_samples_); }

    private:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Choose a random index between 0 and n-1
        * \param n the number of possible indices to choose from
        */
      inline int getRandomIndex (int n) { return (n * (rand () / (RAND_MAX + 1.0))); };
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Select \a nr_samples sample points from cloud while making sure that their pairwise distances are greater 
        * than a user-defined minimum distance, \a min_sample_distance.
        * \param cloud the input point cloud
        * \param nr_samples the number of samples to select
        * \param min_sample_distance the minimum distance between any two samples
        * \param sample_indices the resulting sample indices
        */
      void selectSamples (const PointCloudSource &cloud, int nr_samples, float min_sample_distance, 
                          std::vector<int> &sample_indices);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief For each of the sample points, find a list of points in the target cloud whose features are similar to 
        * the sample points' features. From these, select one randomly which will be considered that sample point's 
        * correspondence. 
        * \param input_features a cloud of feature descriptors
        * \param sample_indices the indices of each sample point
        * \param corresponding_indices the resulting indices of each sample's corresponding point in the target cloud
        */
      void findSimilarFeatures(const FeatureCloud &input_features, const std::vector<int> &sample_indices, 
                               std::vector<int> &corresponding_indices);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief An error metric for that computes the quality of the alignment between the given cloud and the target.
        * \param cloud the input cloud
        * \param threshold distances greater than this value are capped
        */
      float computeErrorMetric (const PointCloudSource &cloud, float threshold);

    protected:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Rigid transformation computation method.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        */
      virtual void computeTransformation (PointCloudSource &output);

      
      /** \brief The source point cloud's feature descriptors. */
      FeatureCloudConstPtr input_features_;

      /** \brief The target point cloud's feature descriptors. */
      FeatureCloudConstPtr target_features_;  

      /** \brief The number of samples to use during each iteration. */
      int nr_samples_;

      /** \brief The minimum distances between samples. */
      float min_sample_distance_;
     
      /** \brief The KdTree used to compare feature descriptors. */
      FeatureKdTreePtr feature_tree_;               

  };
}

#include "pcl/registration/ia_ransac.hpp"

#endif  //#ifndef IA_RANSAC_H_
