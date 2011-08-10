/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_

#include <string>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/unordered_map.hpp>

#include <pcl/pcl_base.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/win32_macros.h"

#include <pcl/registration/correspondence_types.h>

namespace pcl
{
  namespace registration
  {
    /** @b CorrespondenceEstimation represents the base class for determining correspondences between target 
      * and query point sets/features.
      * \author Radu Bogdan Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class CorrespondenceEstimation : public PCLBase<PointSource>
    {
      class FeatureContainerInterface;

      public:
        using PCLBase<PointSource>::initCompute;
        using PCLBase<PointSource>::deinitCompute;
        using PCLBase<PointSource>::input_;
        using PCLBase<PointSource>::indices_;

        typedef typename pcl::KdTree<PointTarget> KdTree;
        typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        typedef boost::unordered_map<std::string, boost::shared_ptr<FeatureContainerInterface> > FeaturesMap;

        /** \brief Empty constructor. */
        CorrespondenceEstimation () : target_ (),
            point_representation_ ()
        {
          tree_.reset (new pcl::KdTreeFLANN<PointTarget>);     // FLANN tree for nearest neighbor search
        }

        /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the 
          * input source to)
          * \param[in] cloud the input point cloud target
          */
        virtual inline void 
        setInputTarget (const PointCloudTargetConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudTargetConstPtr const 
        getInputTarget () { return (target_ ); }

        /** \brief Provide a pointer to a cloud of feature descriptors associated with the source point cloud
          * \param[in] source_feature a cloud of feature descriptors associated with the source point cloud
          * \param[in] key a string that uniquely identifies the feature
          */
        template <typename FeatureT> inline void 
        setSourceFeature (const typename pcl::PointCloud<FeatureT>::ConstPtr &source_feature, 
                          const std::string &key);

        /** \brief Get a pointer to the source cloud's feature descriptors, specified by the given \a key
          * \param key a string that uniquely identifies the feature (must match the key provided by 
          * \ref setSourceFeature)
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
          * \param key a string that uniquely identifies the feature (must match the key provided by 
          * \ref setTargetFeature)
          */
        template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr 
        getTargetFeature (const std::string &key);

        /** \brief Use radius-search as the search method when finding correspondences for the feature 
          * associated with the provided \a key
          * \param[in] tree the KdTree to use to compare features
          * \param[in] r the radius to use when performing the correspondence search
          * \param[in] key a string that uniquely identifies the feature
          */
        template <typename FeatureT> inline void 
        setRadiusSearch (const typename pcl::KdTree<FeatureT>::Ptr &tree, float r, 
                         const std::string &key);

        /** \brief Use k-nearest-neighbors as the search method when finding correspondences for the feature 
          * associated with the provided \a key
          * \param[in] tree the KdTree to use to compare features
          * \param[in] k the number of nearest neighbors to return in the correspondence search
          * \param[in] key a string that uniquely identifies the feature
          */
        template <typename FeatureT> inline void 
        setKSearch (const typename pcl::KdTree<FeatureT>::Ptr &tree, int k, 
                    const std::string &key);

        /** \brief Provide a boost shared pointer to a PointRepresentation to be used when comparing features
          * \param[in] fr the point feature representation to be used by the k-D tree
          * \param[in] key a string that uniquely identifies the feature
          */
        template <typename FeatureT> inline void
        setFeatureRepresentation (const typename pcl::PointRepresentation<FeatureT>::ConstPtr &fr,
                                  const std::string &key);

        /** \brief Provide a boost shared pointer to the PointRepresentation to be used when comparing points
          * \param[in] point_representation the PointRepresentation to be used by the k-D tree
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
        }

        /** \brief Search for the closest nearest neighbor of a given point.
          * \param[in] cloud the point cloud dataset to use for nearest neighbor search
          * \param[in] index the index of the query point
          * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
          * \param[out] distances the resultant distances from the query point to the k-nearest neighbors
          */
        inline bool
        searchForNeighbors (const PointCloudSource &cloud, int index, 
                            std::vector<int> &indices, std::vector<float> &distances)
        {
          int k = tree_->nearestKSearch (cloud, index, 1, indices, distances);
          if (k == 0)
            return (false);
          return (true);
        }

  //      /** brief Deterime the correspondences for the points in a given cloud (closest nearest neighbor).
  //        * param cloud the point cloud dataset to use for nearest neighbor search
  //        * param correspondences the found correspondences (index of query point, index of target point, distance)
  //        */
  //      inline void
  //      determineCorrespondences(const PointCloudSource &cloud, std::vector<pcl::Correspondence> &correspondences)
  //      {
  //        if (cloud.points.size() == 0)
  //          return;
  //
  //        correspondences.resize(cloud.points.size());
  //        std::vector<int> index(1);
  //        std::vector<float> distance(1);
  //        pcl::Correspondence corr;
  //        for (unsigned int i = 0; i < cloud.points.size(); ++i)
  //        {
  //          if ( tree_->nearestKSearch(cloud, i, 1, index, distance) )
  //          {
  //            corr.index_query = i;
  //            corr.index_match = index[0];
  //            corr.distance = distance[0];
  //            correspondences[i] = corr;
  //          }
  //          else
  //          {
  //            correspondences[i] = pcl::Correspondence(i, -1, std::numeric_limits<float>::max());
  //          }
  //        }
  //      }

  //      /** brief Deterime the correspondences for the points in a given cloud (closest nearest neighbor).
  //        * param cloud the point cloud dataset to use for nearest neighbor search
  //        * param indices indices of the points for which correspondences should be determined
  //        * param correspondences the found correspondences (index of query point, index of target point, distance)
  //        */
  //      inline void
  //      determineCorrespondences(const PointCloudSource &cloud, const std::vector<int>& indices, std::vector<pcl::Correspondence> &correspondences)
  //      {
  //        if (cloud.points.size() == 0)
  //          return;
  //
  //        correspondences.resize(cloud.points.size());
  //        std::vector<int> index(1);
  //        std::vector<float> distance(1);
  //        pcl::Correspondence corr;
  //        for (unsigned int i = 0; i < indices.size(); ++i)
  //        {
  //          if ( tree_->nearestKSearch(cloud, indices[i], 1, index, distance) )
  //          {
  //            corr.index_query = i;
  //            corr.index_match = index[0];
  //            corr.distance = distance[0];
  //            correspondences[i] = corr;
  //          }
  //          else
  //          {
  //            correspondences[i] = pcl::Correspondence(i, -1, std::numeric_limits<float>::max());
  //          }
  //        }
  //      }


        /** \brief Determine the correspondences between input and target cloud.
          * \param correspondences the found correspondences (index of query point, index of target point, distance)
          * \param max_distance maximum distance between correspondences
          */
        virtual void 
        determineCorrespondences (std::vector<pcl::Correspondence> &correspondences, 
                                  float max_distance = std::numeric_limits<float>::max ());

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          */
        virtual void 
        determineReciprocalCorrespondences (std::vector<pcl::Correspondence> &correspondences);

        /** \brief Find the indices of the points in the target cloud whose features correspond with the features 
          * of the given point in the source cloud
          * \param correspondence_indices the resultant vector of indices representing the query's corresponding 
          * features (in the target cloud)
          */
        void 
        determineFeatureCorrespondences (std::vector<pcl::Correspondence> &correspondence_indices);

      protected:
        /** \brief The correspondence estimation method name. 
          * (TODO: does this make sense, will we have other correspondence estimators derived from this one?) 
          */
        std::string corr_name_;

        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** \brief The input point cloud dataset target. */
        PointCloudTargetPtr target_;

        /** \brief Test that all features are valid (i.e., does each key have a valid source cloud, target cloud,
          * and search method)
          */
        inline bool 
        hasValidFeatures ();

        /** \brief Abstract class get name method. */
        inline const std::string& 
        getClassName () const { return (corr_name_); }

      private:

        /** \brief The point representation used (internal). */
        PointRepresentationConstPtr point_representation_;

        /** \brief An STL map containing features to use when performing the correspondence search.*/
        FeaturesMap features_map_;

        /** \brief An inner class containing pointers to the source and target feature clouds along with the 
          * KdTree and the parameters needed to perform the correspondence search.  This class extends 
          * FeatureContainerInterface, which contains abstract methods for any methods that do not depend on the 
          * FeatureT --- these methods can thus be called from a pointer to FeatureContainerInterface without 
          * casting to the derived class.
          */
        template <typename FeatureT>
        class FeatureContainer : public pcl::registration::CorrespondenceEstimation<PointSource, PointTarget>::FeatureContainerInterface
        {
          public:
            typedef typename pcl::PointCloud<FeatureT>::ConstPtr FeatureCloudConstPtr;
            typedef typename pcl::KdTree<FeatureT> KdTree;
            typedef typename pcl::KdTree<FeatureT>::Ptr KdTreePtr;
            typedef boost::function<bool (const pcl::PointCloud<FeatureT> &, int, std::vector<int> &,
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
            setSourceFeature ()
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
              * \param[in] fr the point feature representation to be used
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
              // If no feature representation was given, reset to the default implementation for FeatureT
              if (!feature_representation_)
                feature_representation_.reset (new DefaultFeatureRepresentation<FeatureT>);

              // Set the internal feature point representation of choice
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

        class FeatureContainerInterface
        {
          public:
            virtual bool 
            isValid () = 0;

            virtual void 
            findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices,
                                        std::vector<float> &distances) = 0;
        };
     };
  }
}

#include "pcl/registration/impl/correspondence_estimation.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_ */
