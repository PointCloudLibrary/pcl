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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_FEATURES_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_FEATURES_H_

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
  namespace registration
  {
    /**
      * @b CorrespondenceRejectorFeatures implements a correspondence rejection method based on a set of feature
      * descriptors. Given an input feature space, the method checks if each feature in the source cloud has a
      * correspondence in the target cloud, either by checking the first K (given) point correspondences, or 
      * by defining a tolerance threshold via a radius in feature space.
      * \todo explain this better.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    class CorrespondenceRejectorFeatures: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      public:
        /** \brief Empty constructor. */
        CorrespondenceRejectorFeatures () : max_distance_ (std::numeric_limits<float>::max ())
        {
          rejection_name_ = "CorrespondenceRejectorFeatures";
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences
          * \param original_correspondences the set of initial correspondences given
          * \param remaining_correspondences the resultant filtered set of remaining correspondences
          */
        void 
        getRemainingCorrespondences (const pcl::registration::Correspondences& original_correspondences, 
                                     pcl::registration::Correspondences& remaining_correspondences);

        /** \brief Provide a pointer to a cloud of feature descriptors associated with the source point cloud
          * \param source_feature a cloud of feature descriptors associated with the source point cloud
          * \param key a string that uniquely identifies the feature
          */
        template <typename FeatureType> inline void 
        setSourceFeature (const typename pcl::PointCloud<FeatureType>::ConstPtr &source_feature, 
                          const std::string &key);

        /** \brief Get a pointer to the source cloud's feature descriptors, specified by the given \a key
          * \param key a string that uniquely identifies the feature (must match the key provided by setSourceFeature)
          */
        template <typename FeatureType> inline typename pcl::PointCloud<FeatureType>::ConstPtr 
        getSourceFeature (const std::string &key);

        /** \brief Provide a pointer to a cloud of feature descriptors associated with the target point cloud
          * \param target_feature a cloud of feature descriptors associated with the target point cloud
          * \param key a string that uniquely identifies the feature
          */
        template <typename FeatureType> inline void 
        setTargetFeature (const typename pcl::PointCloud<FeatureType>::ConstPtr &target_feature, 
                          const std::string &key);

        /** \brief Get a pointer to the source cloud's feature descriptors, specified by the given \a key
          * \param key a string that uniquely identifies the feature (must match the key provided by setTargetFeature)
          */
        template <typename FeatureType> inline typename pcl::PointCloud<FeatureType>::ConstPtr 
        getTargetFeature (const std::string &key);

        /** \brief Use radius-search as the search method when finding correspondences for the feature 
          * associated with the provided \a key      
          * \param r the radius to use when performing the correspondence search
          * \param key a string that uniquely identifies the feature
          * \param tree the KdTree to use to compare features
          */
        template <typename FeatureType> inline void 
        setRadiusSearch (float r, const std::string &key, const typename pcl::KdTree<FeatureType>::Ptr &tree);

        /** \brief Use radius-search as the search method when finding correspondences for the feature 
          * associated with the provided \a key      
          * \param r the radius to use when performing the correspondence search
          * \param key a string that uniquely identifies the feature
          * \param tree the KdTree to use to compare features 
          */
        template <typename FeatureType> inline void 
        setRadiusSearch (float r, const std::string &key)
        {
          typename pcl::KdTree<FeatureType>::Ptr tree (new pcl::KdTreeFLANN<FeatureType>);
          setRadiusSearch<FeatureType> (r, key, tree);
        }

        /** \brief Use k-nearest-neighbors as the search method when finding correspondences for the feature 
          * associated with the provided \a key
          * \param k the number of nearest neighbors to return in the correspondence search
          * \param key a string that uniquely identifies the feature
          * \param tree the KdTree to use to compare features
          */
        template <typename FeatureType> inline void 
        setKSearch (int k, const std::string &key, const typename pcl::KdTree<FeatureType>::Ptr &tree);

        /** \brief Use k-nearest-neighbors as the search method when finding correspondences for the feature 
          * associated with the provided \a key
          * \param k the number of nearest neighbors to return in the correspondence search
          * \param key a string that uniquely identifies the feature
          */
        template <typename FeatureType> inline void 
        setKSearch (int k, const std::string &key)
        {
          typename pcl::KdTreeFLANN<FeatureType>::Ptr tree (new pcl::KdTreeFLANN<FeatureType>);
          setKSearch<FeatureType> (k, key, tree);
        }

        /** \brief Test that all features are valid (i.e., does each key have a valid source cloud, target cloud, 
          * and search method)
          */
        inline bool 
        hasValidFeatures ();

        /** \brief Provide a boost shared pointer to a PointRepresentation to be used when comparing features
          * \param key a string that uniquely identifies the feature
          * \param fr the point feature representation to be used by the k-D tree
          */
        template <typename FeatureType> inline void
        setFeatureRepresentation (const std::string &key,
                                  const typename pcl::KdTree<FeatureType>::PointRepresentationConstPtr &fr);

     protected:

        void 
        applyRejection (pcl::registration::Correspondences &correspondences);

        float max_distance_;

        class FeatureContainerInterface
        {
          public:
            virtual bool isValid () = 0;
            virtual void findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices, 
                                                     std::vector<float> &distances) = 0;
        };

        typedef std::map<std::string, boost::shared_ptr<pcl::registration::CorrespondenceRejectorFeatures::FeatureContainerInterface> > FeaturesMap;

        /** \brief An STL map containing features to use when performing the correspondence search.*/
        FeaturesMap features_map_;

        /** \brief An inner class containing pointers to the source and target feature clouds along with the 
          * KdTree and the parameters needed to perform the correspondence search.  This class extends 
          * FeatureContainerInterface, which contains abstract methods for any methods that do not depend on the 
          * FeatureType --- these methods can thus be called from a pointer to FeatureContainerInterface without 
          * casting to the derived class.
          */
        template <typename FeatureType>
        class FeatureContainer : public pcl::registration::CorrespondenceRejectorFeatures::FeatureContainerInterface
        {
          public:
            typedef typename pcl::PointCloud<FeatureType>::ConstPtr FeatureCloudConstPtr;
            typedef typename pcl::KdTree<FeatureType> KdTree;
            typedef typename pcl::KdTree<FeatureType>::Ptr KdTreePtr;
            typedef boost::function<int (const pcl::PointCloud<FeatureType> &, int, std::vector<int> &, 
                                          std::vector<float> &)> SearchMethod;
            
            typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

            FeatureContainer () : k_(0), radius_(0),
                                  feature_representation_()
            {
            }

            void 
            setSourceFeature (const FeatureCloudConstPtr &source_features)
            {
              source_features_ = source_features;
            }
            
            FeatureCloudConstPtr 
            getSourceFeature ()
            {
              return (source_features_);
            }
            
            void 
            setTargetFeature (const FeatureCloudConstPtr &target_features)
            {
              target_features_ = target_features;
              if (tree_)
                tree_->setInputCloud (target_features_);
            }
            
            FeatureCloudConstPtr 
            getTargetFeature ()
            {
              return (target_features_);
            }
            
            void 
            setRadiusSearch (KdTreePtr tree, float r)
            {
              tree_ = tree;
              radius_ = r;
              k_ = 0;
              if (target_features_)
                tree_->setInputCloud (target_features_);
            }

            void 
            setKSearch (KdTreePtr tree, int k)
            {
              tree_ = tree;
              k_ = k;
              radius_ = 0.0;
              if (target_features_)
                tree_->setInputCloud (target_features_);
            }
            
            virtual bool 
            isValid ()
            {
              if (!source_features_ || !target_features_ || !tree_)
                return (false);
              else
                return (source_features_->points.size () > 0 && 
                        target_features_->points.size () > 0 &&
                        (k_ > 0 || radius_ > 0.0));
            }

            /** \brief Provide a boost shared pointer to a PointRepresentation to be used when comparing features
              * \param fr the point feature representation to be used by the k-D tree
              */
            inline void
            setFeatureRepresentation (const PointRepresentationConstPtr &fr)
            {
              feature_representation_ = fr;
            }

            virtual void 
            findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices, 
                                        std::vector<float> &distances)
            {
              // Set the internal feature point representation of choice
              if (feature_representation_)
                tree_->setPointRepresentation (feature_representation_);

              if (k_ > 0)
              {
                correspondence_indices.resize (k_);
                distances.resize (k_);
                tree_->nearestKSearch (*source_features_, index, k_, correspondence_indices, distances);
              }
              else
                tree_->radiusSearch (*source_features_, index, radius_, correspondence_indices, distances);
            }
            
          private:
            FeatureCloudConstPtr source_features_, target_features_;
            KdTreePtr tree_;
            SearchMethod search_method_;
            int k_;
            double radius_;

            /** \brief The internal point feature representation used. */
            PointRepresentationConstPtr feature_representation_;
        };
    };
  }
}

#include "pcl/registration/impl/correspondence_rejection_features.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_FEATURES_H_ */
