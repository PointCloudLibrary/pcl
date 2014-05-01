/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_FEATURES_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_FEATURES_H_

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/registration/boost.h>

namespace pcl
{
  namespace registration
  {
    /** \brief CorrespondenceRejectorFeatures implements a correspondence rejection method based on a set of feature
      * descriptors. Given an input feature space, the method checks if each feature in the source cloud has a
      * correspondence in the target cloud, either by checking the first K (given) point correspondences, or 
      * by defining a tolerance threshold via a radius in feature space.
      * \todo explain this better.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    class PCL_EXPORTS CorrespondenceRejectorFeatures: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      public:
        typedef boost::shared_ptr<CorrespondenceRejectorFeatures> Ptr;
        typedef boost::shared_ptr<const CorrespondenceRejectorFeatures> ConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceRejectorFeatures () : max_distance_ (std::numeric_limits<float>::max ()), features_map_ ()
        {
          rejection_name_ = "CorrespondenceRejectorFeatures";
        }

        /** \brief Empty destructor. */
        virtual ~CorrespondenceRejectorFeatures () {}

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences);

        /** \brief Provide a pointer to a cloud of feature descriptors associated with the source point cloud
          * \param[in] source_feature a cloud of feature descriptors associated with the source point cloud
          * \param[in] key a string that uniquely identifies the feature
          */
        template <typename FeatureT> inline void 
        setSourceFeature (const typename pcl::PointCloud<FeatureT>::ConstPtr &source_feature, 
                          const std::string &key);

        /** \brief Get a pointer to the source cloud's feature descriptors, specified by the given \a key
          * \param[in] key a string that uniquely identifies the feature (must match the key provided by setSourceFeature)
          */
        template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr 
        getSourceFeature (const std::string &key);

        /** \brief Provide a pointer to a cloud of feature descriptors associated with the target point cloud
          * \param[in] target_feature a cloud of feature descriptors associated with the target point cloud
          * \param[in] key a string that uniquely identifies the feature
          */
        template <typename FeatureT> inline void 
        setTargetFeature (const typename pcl::PointCloud<FeatureT>::ConstPtr &target_feature, 
                          const std::string &key);

        /** \brief Get a pointer to the source cloud's feature descriptors, specified by the given \a key
          * \param[in] key a string that uniquely identifies the feature (must match the key provided by setTargetFeature)
          */
        template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr 
        getTargetFeature (const std::string &key);

        /** \brief Set a hard distance threshold in the feature \a FeatureT space, between source and target
          * features. Any feature correspondence that is above this threshold will be considered bad and will be
          * filtered out.
          * \param[in] thresh the distance threshold
          * \param[in] key a string that uniquely identifies the feature
          */
        template <typename FeatureT> inline void 
        setDistanceThreshold (double thresh, const std::string &key);

        /** \brief Test that all features are valid (i.e., does each key have a valid source cloud, target cloud, 
          * and search method)
          */
        inline bool 
        hasValidFeatures ();

        /** \brief Provide a boost shared pointer to a PointRepresentation to be used when comparing features
          * \param[in] key a string that uniquely identifies the feature
          * \param[in] fr the point feature representation to be used 
          */
        template <typename FeatureT> inline void
        setFeatureRepresentation (const typename pcl::PointRepresentation<FeatureT>::ConstPtr &fr,
                                  const std::string &key);

      protected:

        /** \brief Apply the rejection algorithm.
          * \param[out] correspondences the set of resultant correspondences.
          */
        inline void 
        applyRejection (pcl::Correspondences &correspondences)
        {
          getRemainingCorrespondences (*input_correspondences_, correspondences);
        }

        /** \brief The maximum distance threshold between two correspondent points in source <-> target. If the
          * distance is larger than this threshold, the points will not be ignored in the alignment process.
          */
        float max_distance_;

        class FeatureContainerInterface
        {
          public:
            /** \brief Empty destructor */
            virtual ~FeatureContainerInterface () {}
            virtual bool isValid () = 0;
            virtual double getCorrespondenceScore (int index) = 0;
            virtual bool isCorrespondenceValid (int index) = 0;
        };

        typedef boost::unordered_map<std::string, boost::shared_ptr<FeatureContainerInterface> > FeaturesMap;

        /** \brief An STL map containing features to use when performing the correspondence search.*/
        FeaturesMap features_map_;

        /** \brief An inner class containing pointers to the source and target feature clouds 
          * and the parameters needed to perform the correspondence search.  This class extends 
          * FeatureContainerInterface, which contains abstract methods for any methods that do not depend on the 
          * FeatureT --- these methods can thus be called from a pointer to FeatureContainerInterface without 
          * casting to the derived class.
          */
        template <typename FeatureT>
        class FeatureContainer : public pcl::registration::CorrespondenceRejectorFeatures::FeatureContainerInterface
        {
          public:
            typedef typename pcl::PointCloud<FeatureT>::ConstPtr FeatureCloudConstPtr;
            typedef boost::function<int (const pcl::PointCloud<FeatureT> &, int, std::vector<int> &, 
                                          std::vector<float> &)> SearchMethod;
            
            typedef typename pcl::PointRepresentation<FeatureT>::ConstPtr PointRepresentationConstPtr;

            FeatureContainer () : thresh_(std::numeric_limits<double>::max ()), feature_representation_()
            {
            }
      
            /** \brief Empty destructor */
            virtual ~FeatureContainer () {}

            inline void 
            setSourceFeature (const FeatureCloudConstPtr &source_features)
            {
              source_features_ = source_features;
            }
            
            inline FeatureCloudConstPtr 
            getSourceFeature ()
            {
              return (source_features_);
            }
            
            inline void 
            setTargetFeature (const FeatureCloudConstPtr &target_features)
            {
              target_features_ = target_features;
            }
            
            inline FeatureCloudConstPtr 
            getTargetFeature ()
            {
              return (target_features_);
            }
            
            inline void 
            setDistanceThreshold (double thresh)
            {
              thresh_ = thresh;
            }

            virtual inline bool 
            isValid ()
            {
              if (!source_features_ || !target_features_)
                return (false);
              else
                return (source_features_->points.size () > 0 && 
                        target_features_->points.size () > 0);
            }

            /** \brief Provide a boost shared pointer to a PointRepresentation to be used when comparing features
              * \param[in] fr the point feature representation to be used
              */
            inline void
            setFeatureRepresentation (const PointRepresentationConstPtr &fr)
            {
              feature_representation_ = fr;
            }

            /** \brief Obtain a score between a pair of correspondences.
              * \param[in] index the index to check in the list of correspondences
              * \return score the resultant computed score
              */
            virtual inline double
            getCorrespondenceScore (int index)
            {
              // If no feature representation was given, reset to the default implementation for FeatureT
              if (!feature_representation_)
                feature_representation_.reset (new DefaultFeatureRepresentation<FeatureT>);

              // Get the source and the target feature from the list
              const FeatureT &feat_src = source_features_->points[index];
              const FeatureT &feat_tgt = target_features_->points[index];

              // Check if the representations are valid
              if (!feature_representation_->isValid (feat_src) || !feature_representation_->isValid (feat_tgt))
              {
                PCL_ERROR ("[pcl::registration::%s::getCorrespondenceScore] Invalid feature representation given!\n", this->getClassName ().c_str ());
                return (std::numeric_limits<double>::max ());
              }

              // Set the internal feature point representation of choice
              Eigen::VectorXf feat_src_ptr = Eigen::VectorXf::Zero (feature_representation_->getNumberOfDimensions ());
              feature_representation_->vectorize (FeatureT (feat_src), feat_src_ptr);
              Eigen::VectorXf feat_tgt_ptr = Eigen::VectorXf::Zero (feature_representation_->getNumberOfDimensions ());
              feature_representation_->vectorize (FeatureT (feat_tgt), feat_tgt_ptr);

              // Compute the L2 norm
              return ((feat_src_ptr - feat_tgt_ptr).squaredNorm ());
            }

            /** \brief Check whether the correspondence pair at the given index is valid
              * by computing the score and testing it against the user given threshold 
              * \param[in] index the index to check in the list of correspondences
              * \return true if the correspondence is good, false otherwise
              */
            virtual inline bool
            isCorrespondenceValid (int index)
            {
              if (getCorrespondenceScore (index) < thresh_ * thresh_)
                return (true);
              else
                return (false);
            }
             
          private:
            FeatureCloudConstPtr source_features_, target_features_;
            SearchMethod search_method_;

            /** \brief The L2 squared Euclidean threshold. */
            double thresh_;

            /** \brief The internal point feature representation used. */
            PointRepresentationConstPtr feature_representation_;
        };
    };
  }
}

#include <pcl/registration/impl/correspondence_rejection_features.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_FEATURES_H_ */
