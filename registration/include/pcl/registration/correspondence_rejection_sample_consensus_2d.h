/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *
 */

#pragma once

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/memory.h>

namespace pcl {
namespace registration {
/** \brief CorrespondenceRejectorSampleConsensus2D implements a pixel-based
 * correspondence rejection using Random Sample Consensus to identify inliers
 * (and reject outliers)
 * \author Radu B. Rusu
 * \ingroup registration
 */
template <typename PointT>
class CorrespondenceRejectorSampleConsensus2D
: public CorrespondenceRejectorSampleConsensus<PointT> {
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  using CorrespondenceRejectorSampleConsensus<PointT>::refine_;
  using CorrespondenceRejectorSampleConsensus<PointT>::input_;
  using CorrespondenceRejectorSampleConsensus<PointT>::target_;
  using CorrespondenceRejectorSampleConsensus<PointT>::input_correspondences_;
  using CorrespondenceRejectorSampleConsensus<PointT>::rejection_name_;
  using CorrespondenceRejectorSampleConsensus<PointT>::getClassName;
  using CorrespondenceRejectorSampleConsensus<PointT>::inlier_threshold_;
  using CorrespondenceRejectorSampleConsensus<PointT>::max_iterations_;
  using CorrespondenceRejectorSampleConsensus<PointT>::best_transformation_;

  using Ptr = shared_ptr<CorrespondenceRejectorSampleConsensus2D<PointT>>;
  using ConstPtr = shared_ptr<const CorrespondenceRejectorSampleConsensus2D<PointT>>;

  /** \brief Empty constructor. Sets the inlier threshold to 5cm (0.05m),
   * and the maximum number of iterations to 1000.
   */
  CorrespondenceRejectorSampleConsensus2D()
  : projection_matrix_(Eigen::Matrix3f::Identity())
  {
    rejection_name_ = "CorrespondenceRejectorSampleConsensus2D";
    // Put the projection matrix together
    // projection_matrix_ (0, 0) = 525.f;
    // projection_matrix_ (1, 1) = 525.f;
    // projection_matrix_ (0, 2) = 320.f;
    // projection_matrix_ (1, 2) = 240.f;
  }

  /** \brief Get a list of valid correspondences after rejection from the original set
   * of correspondences. \param[in] original_correspondences the set of initial
   * correspondences given \param[out] remaining_correspondences the resultant filtered
   * set of remaining correspondences
   */
  inline void
  getRemainingCorrespondences(const pcl::Correspondences& original_correspondences,
                              pcl::Correspondences& remaining_correspondences);

  /** \brief Sets the focal length parameters of the target camera.
   * \param[in] fx the focal length in pixels along the x-axis of the image
   * \param[in] fy the focal length in pixels along the y-axis of the image
   */
  inline void
  setFocalLengths(const float fx, const float fy)
  {
    projection_matrix_(0, 0) = fx;
    projection_matrix_(1, 1) = fy;
  }

  /** \brief Reads back the focal length parameters of the target camera.
   * \param[out] fx the focal length in pixels along the x-axis of the image
   * \param[out] fy the focal length in pixels along the y-axis of the image
   */
  inline void
  getFocalLengths(float& fx, float& fy) const
  {
    fx = projection_matrix_(0, 0);
    fy = projection_matrix_(1, 1);
  }

  /** \brief Sets the camera center parameters of the target camera.
   * \param[in] cx the x-coordinate of the camera center
   * \param[in] cy the y-coordinate of the camera center
   */
  inline void
  setCameraCenters(const float cx, const float cy)
  {
    projection_matrix_(0, 2) = cx;
    projection_matrix_(1, 2) = cy;
  }

  /** \brief Reads back the camera center parameters of the target camera.
   * \param[out] cx the x-coordinate of the camera center
   * \param[out] cy the y-coordinate of the camera center
   */
  inline void
  getCameraCenters(float& cx, float& cy) const
  {
    cx = projection_matrix_(0, 2);
    cy = projection_matrix_(1, 2);
  }

protected:
  /** \brief Apply the rejection algorithm.
   * \param[out] correspondences the set of resultant correspondences.
   */
  inline void
  applyRejection(pcl::Correspondences& correspondences)
  {
    getRemainingCorrespondences(*input_correspondences_, correspondences);
  }

  /** \brief Camera projection matrix. */
  Eigen::Matrix3f projection_matrix_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace registration
} // namespace pcl

#include <pcl/registration/impl/correspondence_rejection_sample_consensus_2d.hpp>
