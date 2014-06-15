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

#ifndef PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_H_
#define PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_H_

#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/registration/correspondence_rejection_poly.h>

namespace pcl
{
  /** \brief Pose estimation and alignment class using a prerejective RANSAC routine.
   * 
   * This class inserts a simple, yet effective "prerejection" step into the standard
   * RANSAC pose estimation loop in order to avoid verification of pose hypotheses
   * that are likely to be wrong. This is achieved by local pose-invariant geometric
   * constraints, as also implemented in the class
   * \ref registration::CorrespondenceRejectorPoly "CorrespondenceRejectorPoly".
   * 
   * In order to robustly align partial/occluded models, this routine performs
   * fit error evaluation using only inliers, i.e. points closer than a
   * Euclidean threshold, which is specifiable using \ref setInlierFraction().
   * 
   * The amount of prerejection or "greedyness" of the algorithm can be specified
   * using \ref setSimilarityThreshold() in [0,1[, where a value of 0 means disabled,
   * and 1 is maximally rejective.
   * 
   * If you use this in academic work, please cite:
   * 
   * A. G. Buch, D. Kraft, J.-K. Kämäräinen, H. G. Petersen and N. Krüger.
   * Pose Estimation using Local Structure-Specific Shape and Appearance Context.
   * International Conference on Robotics and Automation (ICRA), 2013.
   *  
   * \author Anders Glent Buch (andersgb1@gmail.com)
   * \ingroup registration
   */
  template <typename PointSource, typename PointTarget, typename FeatureT>
  class SampleConsensusPrerejective : public Registration<PointSource, PointTarget>
  {
    public:
      typedef typename Registration<PointSource, PointTarget>::Matrix4 Matrix4;
      
      using Registration<PointSource, PointTarget>::reg_name_;
      using Registration<PointSource, PointTarget>::getClassName;
      using Registration<PointSource, PointTarget>::input_;
      using Registration<PointSource, PointTarget>::target_;
      using Registration<PointSource, PointTarget>::tree_;
      using Registration<PointSource, PointTarget>::max_iterations_;
      using Registration<PointSource, PointTarget>::corr_dist_threshold_;
      using Registration<PointSource, PointTarget>::transformation_;
      using Registration<PointSource, PointTarget>::final_transformation_;
      using Registration<PointSource, PointTarget>::transformation_estimation_;
      using Registration<PointSource, PointTarget>::getFitnessScore;
      using Registration<PointSource, PointTarget>::converged_;

      typedef typename Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef typename Registration<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      typedef pcl::PointCloud<FeatureT> FeatureCloud;
      typedef typename FeatureCloud::Ptr FeatureCloudPtr;
      typedef typename FeatureCloud::ConstPtr FeatureCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusPrerejective<PointSource, PointTarget, FeatureT> > Ptr;
      typedef boost::shared_ptr<const SampleConsensusPrerejective<PointSource, PointTarget, FeatureT> > ConstPtr;

      typedef typename KdTreeFLANN<FeatureT>::Ptr FeatureKdTreePtr;
      
      typedef pcl::registration::CorrespondenceRejectorPoly<PointSource, PointTarget> CorrespondenceRejectorPoly;
      typedef typename CorrespondenceRejectorPoly::Ptr CorrespondenceRejectorPolyPtr;
      typedef typename CorrespondenceRejectorPoly::ConstPtr CorrespondenceRejectorPolyConstPtr;
      
      /** \brief Constructor */
      SampleConsensusPrerejective ()
        : input_features_ ()
        , target_features_ ()
        , nr_samples_(3)
        , k_correspondences_ (2)
        , feature_tree_ (new pcl::KdTreeFLANN<FeatureT>)
        , correspondence_rejector_poly_ (new CorrespondenceRejectorPoly)
        , inlier_fraction_ (0.0f)
      {
        reg_name_ = "SampleConsensusPrerejective";
        correspondence_rejector_poly_->setSimilarityThreshold (0.6f);
        max_iterations_ = 5000;
        transformation_estimation_.reset (new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget>);
      };
      
      /** \brief Destructor */
      virtual ~SampleConsensusPrerejective ()
      {
      }

      /** \brief Provide a boost shared pointer to the source point cloud's feature descriptors
        * \param features the source point cloud's features
        */
      void 
      setSourceFeatures (const FeatureCloudConstPtr &features);

      /** \brief Get a pointer to the source point cloud's features */
      inline const FeatureCloudConstPtr
      getSourceFeatures () const
      { 
        return (input_features_);
      }

      /** \brief Provide a boost shared pointer to the target point cloud's feature descriptors
        * \param features the target point cloud's features
        */
      void 
      setTargetFeatures (const FeatureCloudConstPtr &features);

      /** \brief Get a pointer to the target point cloud's features */
      inline const FeatureCloudConstPtr 
      getTargetFeatures () const
      {
        return (target_features_);
      }

      /** \brief Set the number of samples to use during each iteration
        * \param nr_samples the number of samples to use during each iteration
        */
      inline void 
      setNumberOfSamples (int nr_samples)
      {
        nr_samples_ = nr_samples;
      }

      /** \brief Get the number of samples to use during each iteration, as set by the user */
      inline int 
      getNumberOfSamples () const
      {
        return (nr_samples_);
      }

      /** \brief Set the number of neighbors to use when selecting a random feature correspondence.  A higher value will
        * add more randomness to the feature matching.
        * \param k the number of neighbors to use when selecting a random feature correspondence.
        */
      inline void
      setCorrespondenceRandomness (int k)
      {
        k_correspondences_ = k;
      }

      /** \brief Get the number of neighbors used when selecting a random feature correspondence, as set by the user */
      inline int
      getCorrespondenceRandomness () const
      {
        return (k_correspondences_);
      }
      
      /** \brief Set the similarity threshold in [0,1[ between edge lengths of the underlying polygonal correspondence rejector object,
       * where 1 is a perfect match
       * \param similarity_threshold edge length similarity threshold
       */
      inline void
      setSimilarityThreshold (float similarity_threshold)
      {
        correspondence_rejector_poly_->setSimilarityThreshold (similarity_threshold);
      }
      
      /** \brief Get the similarity threshold between edge lengths of the underlying polygonal correspondence rejector object,
       * \return edge length similarity threshold
       */
      inline float
      getSimilarityThreshold () const
      {
        return correspondence_rejector_poly_->getSimilarityThreshold ();
      }
      
      /** \brief Set the required inlier fraction (of the input)
       * \param inlier_fraction required inlier fraction, must be in [0,1]
       */
      inline void
      setInlierFraction (float inlier_fraction)
      {
        inlier_fraction_ = inlier_fraction;
      }
      
      /** \brief Get the required inlier fraction
       * \return required inlier fraction in [0,1]
       */
      inline float
      getInlierFraction () const
      {
        return inlier_fraction_;
      }
      
      /** \brief Get the inlier indices of the source point cloud under the final transformation
       * @return inlier indices
       */
      inline const std::vector<int>&
      getInliers () const
      {
        return inliers_;
      }

    protected:
      /** \brief Choose a random index between 0 and n-1
        * \param n the number of possible indices to choose from
        */
      inline int 
      getRandomIndex (int n) const
      {
        return (static_cast<int> (n * (rand () / (RAND_MAX + 1.0))));
      };
      
      /** \brief Select \a nr_samples sample points from cloud while making sure that their pairwise distances are 
        * greater than a user-defined minimum distance, \a min_sample_distance.
        * \param cloud the input point cloud
        * \param nr_samples the number of samples to select
        * \param sample_indices the resulting sample indices
        */
      void 
      selectSamples (const PointCloudSource &cloud, int nr_samples, std::vector<int> &sample_indices);

      /** \brief For each of the sample points, find a list of points in the target cloud whose features are similar to 
        * the sample points' features. From these, select one randomly which will be considered that sample point's 
        * correspondence.
        * \param sample_indices the indices of each sample point
        * \param similar_features correspondence cache, which is used to read/write already computed correspondences
        * \param corresponding_indices the resulting indices of each sample's corresponding point in the target cloud
        */
      void 
      findSimilarFeatures (const std::vector<int> &sample_indices,
              std::vector<std::vector<int> >& similar_features,
              std::vector<int> &corresponding_indices);

      /** \brief Rigid transformation computation method.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess The computed transformation
        */
      void 
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess);

      /** \brief Obtain the fitness of a transformation
        * The following metrics are calculated, based on
        * \b final_transformation_ and \b corr_dist_threshold_:
        *   - Inliers: the number of transformed points which are closer than threshold to NN
        *   - Error score: the MSE of the inliers  
        * \param inliers indices of source point cloud inliers
        * \param fitness_score output fitness score as RMSE 
        */
      void 
      getFitness (std::vector<int>& inliers, float& fitness_score);

      /** \brief The source point cloud's feature descriptors. */
      FeatureCloudConstPtr input_features_;

      /** \brief The target point cloud's feature descriptors. */
      FeatureCloudConstPtr target_features_;  

      /** \brief The number of samples to use during each iteration. */
      int nr_samples_;

      /** \brief The number of neighbors to use when selecting a random feature correspondence. */
      int k_correspondences_;
     
      /** \brief The KdTree used to compare feature descriptors. */
      FeatureKdTreePtr feature_tree_;
      
      /** \brief The polygonal correspondence rejector used for prerejection */
      CorrespondenceRejectorPolyPtr correspondence_rejector_poly_;
      
      /** \brief The fraction [0,1] of inlier points required for accepting a transformation */
      float inlier_fraction_;
      
      /** \brief Inlier points of final transformation as indices into source */
      std::vector<int> inliers_;
  };
}

#include <pcl/registration/impl/sample_consensus_prerejective.hpp>

#endif
