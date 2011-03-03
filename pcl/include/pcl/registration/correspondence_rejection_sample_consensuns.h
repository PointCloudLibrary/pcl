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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_H_

#include <pcl/registration/correspondence_rejection.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

namespace pcl
{
  namespace registration
  {

    /**
     * @b CorrespondenceRejectorSampleConsensus implements a correspondence rejection
     * using Random Sample Consensus to identify inliers (and reject outliers)
     * \author Dirk Holz
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
      CorrespondenceRejectorSampleConsensus()
      {
        rejection_name_ = "CorrespondenceRejectorSampleConsensus";
        inlier_threshold_ = 0.05;
      }

      inline void getCorrespondences(const pcl::registration::Correspondences& original_correspondences, pcl::registration::Correspondences& remaining_correspondences);

      virtual inline void setInputCloud (const PointCloudConstPtr &cloud) { input_ = cloud; }
      virtual inline void setTargetCloud (const PointCloudConstPtr &cloud) { target_ = cloud; }

      inline void setInlierThreshold(double threshold) { inlier_threshold_ = threshold; };
      inline double getInlierThreshold() { return inlier_threshold_; };

      inline void setMaxIterations(int max_iterations) {max_iterations_ = max_iterations; };
      inline int getMaxIterations() { return max_iterations_; };

      inline Eigen::Matrix4f getBestTransformation() { return best_transformation_; };

    protected:

      inline void applyRejection(pcl::registration::Correspondences &correspondences);

      double inlier_threshold_;

      int max_iterations_;

      PointCloudConstPtr input_;
      PointCloudConstPtr target_;

      Eigen::Matrix4f best_transformation_;
    };

  }
}

#include "pcl/registration/impl/correspondence_rejection_sample_consensus.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_H_ */
