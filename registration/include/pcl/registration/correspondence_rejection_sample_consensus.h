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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_H_

#include <pcl/registration/correspondence_rejection.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

namespace pcl
{
  namespace registration
  {
    /** \brief CorrespondenceRejectorSampleConsensus implements a correspondence rejection
      * using Random Sample Consensus to identify inliers (and reject outliers)
      * \author Dirk Holz
      * \ingroup registration
      */
    template <typename PointT>
    class CorrespondenceRejectorSampleConsensus: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:

        /** \brief Empty constructor. */
        CorrespondenceRejectorSampleConsensus () :
          inlier_threshold_ (0.05),
          max_iterations_ (0),
          input_ (),
          target_ (),
          best_transformation_ ()
        {
          rejection_name_ = "CorrespondenceRejectorSampleConsensus";
        }

        /** \brief Empty destructor. */
        virtual ~CorrespondenceRejectorSampleConsensus () {}

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        inline void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences);

        /** \brief Provide a source point cloud dataset (must contain XYZ data!)
          * \param[in] cloud a cloud containing XYZ data
          */
        virtual inline void 
        setInputCloud (const PointCloudConstPtr &cloud) { input_ = cloud; }

        /** \brief Provide a target point cloud dataset (must contain XYZ data!)
          * \param[in] cloud a cloud containing XYZ data
          */
        virtual inline void 
        setTargetCloud (const PointCloudConstPtr &cloud) { target_ = cloud; }

        /** \brief Set the maximum distance between corresponding points.
          * Correspondences with distances below the threshold are considered as inliers.
          * \param[in] threshold Distance threshold in the same dimension as source and target data sets.
          */
        inline void 
        setInlierThreshold (double threshold) { inlier_threshold_ = threshold; };

        /** \brief Get the maximum distance between corresponding points.
          * \return Distance threshold in the same dimension as source and target data sets.
          */
        inline double 
        getInlierThreshold() { return inlier_threshold_; };

        /** \brief Set the maximum number of iterations.
          * \param[in] max_iterations Maximum number if iterations to run
          */
        inline void 
        setMaxIterations (int max_iterations) {max_iterations_ = std::max(max_iterations, 0); };

        /** \brief Get the maximum number of iterations.
          * \return max_iterations Maximum number if iterations to run
          */
        inline int 
        getMaxIterations () { return max_iterations_; };

        /** \brief Get the best transformation after RANSAC rejection.
          * \return The homogeneous 4x4 transformation yielding the largest number of inliers.
          */
        inline Eigen::Matrix4f 
        getBestTransformation () { return best_transformation_; };

      protected:

        /** \brief Apply the rejection algorithm.
          * \param[out] correspondences the set of resultant correspondences.
          */
        inline void 
        applyRejection (pcl::Correspondences &correspondences)
        {
          getRemainingCorrespondences (*input_correspondences_, correspondences);
        }

        double inlier_threshold_;

        int max_iterations_;

        PointCloudConstPtr input_;
        PointCloudConstPtr target_;

        Eigen::Matrix4f best_transformation_;
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#include <pcl/registration/impl/correspondence_rejection_sample_consensus.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_H_ */
