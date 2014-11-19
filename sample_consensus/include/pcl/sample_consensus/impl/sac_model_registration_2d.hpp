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
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_2D_HPP_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_2D_HPP_

#include <pcl/sample_consensus/sac_model_registration_2d.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/eigen.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistration2D<PointT>::isSampleGood (const std::vector<int>&) const
{
  return (true);
  //using namespace pcl::common;
  //using namespace pcl::traits;

  //PointT p10 = input_->points[samples[1]] - input_->points[samples[0]];
  //PointT p20 = input_->points[samples[2]] - input_->points[samples[0]];
  //PointT p21 = input_->points[samples[2]] - input_->points[samples[1]];

  //return ((p10.x * p10.x + p10.y * p10.y + p10.z * p10.z) > sample_dist_thresh_ && 
  //        (p20.x * p20.x + p20.y * p20.y + p20.z * p20.z) > sample_dist_thresh_ && 
  //        (p21.x * p21.x + p21.y * p21.y + p21.z * p21.z) > sample_dist_thresh_);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration2D<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) 
{
  PCL_INFO ("[pcl::SampleConsensusModelRegistration2D<PointT>::getDistancesToModel]\n");
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration2D::getDistancesToModel] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    distances.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration2D::getDistanceToModel] No target dataset given!\n");
    return;
  }

  distances.resize (indices_->size ());

  // Get the 4x4 transformation
  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src (input_->points[(*indices_)[i]].x, 
                            input_->points[(*indices_)[i]].y, 
                            input_->points[(*indices_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);

    // Project the point on the image plane
    Eigen::Vector3f p_tr3 (p_tr[0], p_tr[1], p_tr[2]);
    Eigen::Vector3f uv (projection_matrix_ * p_tr3);

    if (uv[2] < 0)
      continue;

    uv /= uv[2];

    // Calculate the distance from the transformed point to its correspondence
    // need to compute the real norm here to keep MSAC and friends general
    distances[i] = std::sqrt ((uv[0] - target_->points[(*indices_tgt_)[i]].u) *
                              (uv[0] - target_->points[(*indices_tgt_)[i]].u) +
                              (uv[1] - target_->points[(*indices_tgt_)[i]].v) *
                              (uv[1] - target_->points[(*indices_tgt_)[i]].v));
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration2D<PointT>::selectWithinDistance (const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration2D::selectWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    inliers.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration2D::selectWithinDistance] No target dataset given!\n");
    return;
  }

  double thresh = threshold * threshold;

  int nr_p = 0;
  inliers.resize (indices_->size ());
  error_sqr_dists_.resize (indices_->size ());

  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src (input_->points[(*indices_)[i]].x, 
                            input_->points[(*indices_)[i]].y, 
                            input_->points[(*indices_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);

    // Project the point on the image plane
    Eigen::Vector3f p_tr3 (p_tr[0], p_tr[1], p_tr[2]);
    Eigen::Vector3f uv (projection_matrix_ * p_tr3);

    if (uv[2] < 0)
      continue;

    uv /= uv[2];

    double distance = ((uv[0] - target_->points[(*indices_tgt_)[i]].u) *
                       (uv[0] - target_->points[(*indices_tgt_)[i]].u) +
                       (uv[1] - target_->points[(*indices_tgt_)[i]].v) *
                       (uv[1] - target_->points[(*indices_tgt_)[i]].v));

    // Calculate the distance from the transformed point to its correspondence
    if (distance < thresh)
    {
      inliers[nr_p] = (*indices_)[i];
      error_sqr_dists_[nr_p] = distance;
      ++nr_p;
    }
  }
  inliers.resize (nr_p);
  error_sqr_dists_.resize (nr_p);
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SampleConsensusModelRegistration2D<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold)
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration2D::countWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    return (0);
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration2D::countWithinDistance] No target dataset given!\n");
    return (0);
  }

  double thresh = threshold * threshold;

  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  int nr_p = 0; 
  
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src (input_->points[(*indices_)[i]].x, 
                            input_->points[(*indices_)[i]].y, 
                            input_->points[(*indices_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);

    // Project the point on the image plane
    Eigen::Vector3f p_tr3 (p_tr[0], p_tr[1], p_tr[2]);
    Eigen::Vector3f uv (projection_matrix_ * p_tr3);

    if (uv[2] < 0)
      continue;

    uv /= uv[2];

    // Calculate the distance from the transformed point to its correspondence
    if (((uv[0] - target_->points[(*indices_tgt_)[i]].u) *
         (uv[0] - target_->points[(*indices_tgt_)[i]].u) +
         (uv[1] - target_->points[(*indices_tgt_)[i]].v) *
         (uv[1] - target_->points[(*indices_tgt_)[i]].v)) < thresh)
      ++nr_p;
  }
  return (nr_p);
} 

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_2D_HPP_

