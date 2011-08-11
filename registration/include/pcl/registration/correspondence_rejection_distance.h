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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_DISTANCE_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_DISTANCE_H_

#include <pcl/registration/correspondence_rejection.h>

namespace pcl
{
  namespace registration
  {
    /**
      * @b CorrespondenceRejectorDistance implements a simple correspondence
      * rejection method based on thresholding the distances between the
      * correspondences.
      * \author Dirk Holz
      * \ingroup registration
      */
    class CorrespondenceRejectorDistance: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      public:
        CorrespondenceRejectorDistance () : max_distance_(std::numeric_limits<float>::max ()),
                                            data_container_ ()
        {
          rejection_name_ = "CorrespondenceRejectorDistance";
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * \param original_correspondences the set of initial correspondences given
          * \param remaining_correspondences the resultant filtered set of remaining correspondences
          */
        inline void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences);

        /** \brief Set the maximum distance used for thresholding in correspondence rejection.
          * \param distance Distance to be used as maximum distance between correspondences. 
          * Correspondences with larger distances are rejected.
          * \note Internally, the distance will be stored squared.
          */
        virtual inline void 
        setMaximumDistance (float distance) { max_distance_ = distance * distance; };

        /** \brief Get the maximum distance used for thresholding in correspondence rejection. */
        inline float 
        getMaxmimumDistance () { return std::sqrt (max_distance_); };

        /** \brief Provide a pointer to a cloud 
          * \param[in] cloud a cloud 
          */
        template <typename PointT> inline void 
        setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
        {
          data_container_.reset (new DataContainer<PointT>);
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputCloud (cloud);
        }

        /** \brief Get a pointer to the source cloud's feature descriptors, specified by the given \a key
          * \param key a string that uniquely identifies the feature (must match the key provided by setSourceFeature)
          */
//        template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr 
//        getSourceFeature (const std::string &key);

        /** \brief Provide a pointer to a cloud of feature descriptors associated with the target point cloud
          * \param target_feature a cloud of feature descriptors associated with the target point cloud
          * \param key a string that uniquely identifies the feature
          */
        template <typename PointT> inline void 
        setInputTarget (const typename pcl::PointCloud<PointT>::ConstPtr &target)
        {
          boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputTarget (target);
        }

        /** \brief Get a pointer to the source cloud's feature descriptors, specified by the given \a key
          * \param key a string that uniquely identifies the feature (must match the key provided by setTargetFeature)
          */
//        template <typename FeatureT> inline typename pcl::PointCloud<FeatureT>::ConstPtr 
//        getTargetFeature (const std::string &key);

      protected:

        void 
        applyRejection (pcl::Correspondences &correspondences);

        /** \brief The maximum distance threshold between two correspondent points in source <-> target. If the
          * distance is larger than this threshold, the points will not be ignored in the alignment process.
          */
        float max_distance_;

        class DataContainerInterface
        {
          public:
            virtual double getCorrespondenceScore (int index) = 0;
            virtual double getCorrespondenceScore (const pcl::Correspondence &) = 0;
        };

        template <typename PointT>
        class DataContainer : public DataContainerInterface
        {
          typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
          typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;
          
          public:

            DataContainer ()
            {
              tree_.reset (new pcl::KdTreeFLANN<PointT>);
            }

            inline void 
            setInputCloud (const PointCloudConstPtr &cloud)
            {
              input_ = cloud;
            }

            inline void 
            setInputTarget (const PointCloudConstPtr &target)
            {
              target_ = target;
              tree_->setInputCloud (target_);
            }

            inline double 
            getCorrespondenceScore (int index)
            {
              std::vector<int> indices (1);
              std::vector<float> distances (1);
              if (tree_->nearestKSearch (input_->points[index], 1, indices, distances))
              {
                return (distances[0]);
              }
              else
                return (std::numeric_limits<double>::max ());
            }

            inline double 
            getCorrespondenceScore (const pcl::Correspondence &corr)
            {
              // Get the source and the target feature from the list
              const PointT &src = input_->points[corr.index_query];
              const PointT &tgt = target_->points[corr.index_match];

              return ((src.getVector4fMap () - tgt.getVector4fMap ()).squaredNorm ());
            }

          private:
            PointCloudConstPtr input_, target_;
            KdTreePtr tree_;
        };

        typedef boost::shared_ptr<DataContainerInterface> DataContainerPtr;

        DataContainerPtr data_container_;
    };

  }
}

#include "pcl/registration/impl/correspondence_rejection_distance.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_DISTANCE_H_ */
