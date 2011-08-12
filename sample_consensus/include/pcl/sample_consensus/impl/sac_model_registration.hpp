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
 * $Id$
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_

#include "pcl/sample_consensus/sac_model_registration.h"
#include "pcl/common/rigid_transforms.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistration<PointT>::isSampleGood(const std::vector<int> &samples) const
{
  Eigen::Array4f p1p0, p2p0, p2p1;

  // Get the values at the points
  pcl::Array4fMapConst p0 = input_->points[samples[0]].getArray4fMap();
  pcl::Array4fMapConst p1 = input_->points[samples[1]].getArray4fMap();
  pcl::Array4fMapConst p2 = input_->points[samples[2]].getArray4fMap();

  // Compute the segment values (in 3d) between p1 and p0
  p1p0 = p1 - p0;
  p2p0 = p2 - p0;
  p2p1 = p2 - p1;
  return ((p1p0.matrix().squaredNorm() > sample_dist_thresh_) && (p2p0.matrix().squaredNorm() > sample_dist_thresh_)
        && (p2p1.matrix().squaredNorm() > sample_dist_thresh_));
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistration<PointT>::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 3 samples
  if (samples.size () != 3)
    return (false);

  std::vector<int> indices_src (3);
  std::vector<int> indices_tgt (3);
  for (int i = 0; i < 3; ++i)
  {
    indices_src[i] = samples[i];
    indices_tgt[i] = correspondences_[samples[i]];
  }

  Eigen::Matrix4f transform;
  estimateRigidTransformationSVD<PointT, PointT> (*input_, indices_src, *target_, indices_tgt, transform);
  model_coefficients.resize (16);
  model_coefficients.segment<4>(0)  = transform.row (0);
  model_coefficients.segment<4>(4)  = transform.row (1);
  model_coefficients.segment<4>(8)  = transform.row (2);
  model_coefficients.segment<4>(12) = transform.row (3);

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::getDistancesToModel] Number of source indices (%lu) differs than number of target indices (%lu)!\n", (unsigned long)indices_->size (), (unsigned long)indices_tgt_->size ());
    distances.clear ();
    return;
  }
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_->size ());

  // Get the 4x4 transformation
  Eigen::Matrix4f transform;
  transform.row (0) = model_coefficients.segment<4>(0);
  transform.row (1) = model_coefficients.segment<4>(4);
  transform.row (2) = model_coefficients.segment<4>(8);
  transform.row (3) = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Vector4fMapConst pt_src = input_->points[(*indices_)[i]].getVector4fMap ();
    Vector4fMapConst pt_tgt = target_->points[(*indices_tgt_)[i]].getVector4fMap ();
    Eigen::Vector4f p_tr = transform * pt_src;
    // Calculate the distance from the transformed point to its correspondence
    // need to compute the real norm here to keep MSAC and friends general
    distances[i] = (p_tr - pt_tgt).norm ();
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::selectWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", (unsigned long)indices_->size (), (unsigned long)indices_tgt_->size ());
    inliers.clear ();
    return;
  }

  double thresh = threshold * threshold;

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }
  
  inliers.resize (indices_->size ());

  Eigen::Matrix4f transform;
  transform.row (0) = model_coefficients.segment<4>(0);
  transform.row (1) = model_coefficients.segment<4>(4);
  transform.row (2) = model_coefficients.segment<4>(8);
  transform.row (3) = model_coefficients.segment<4>(12);

  int nr_p = 0; 
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Vector4fMapConst pt_src = input_->points[(*indices_)[i]].getVector4fMap ();
    Vector4fMapConst pt_tgt = target_->points[(*indices_tgt_)[i]].getVector4fMap ();

    Eigen::Vector4f p_tr  = transform * pt_src;
    // Calculate the distance from the transformed point to its correspondence
    if ((p_tr - pt_tgt).squaredNorm () < thresh)
      inliers[nr_p++] = (*indices_)[i];
  }
  inliers.resize (nr_p);
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::optimizeModelCoefficients] Number of source indices (%lu) differs than number of target indices (%lu)!\n", (unsigned long)indices_->size (), (unsigned long)indices_tgt_->size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    optimized_coefficients = model_coefficients;
    return;
  }

  std::vector<int> indices_src (inliers.size ());
  std::vector<int> indices_tgt (inliers.size ());
  for (size_t i = 0; i < inliers.size (); ++i)
  {
    // NOTE: not tested!
    indices_src[i] = (*indices_)[inliers[i]];
    indices_tgt[i] = (*indices_tgt_)[inliers[i]];
  }

  Eigen::Matrix4f transform;
  estimateRigidTransformationSVD<PointT, PointT> (*input_, indices_src, *target_, indices_tgt, transform);
  optimized_coefficients.resize (16);
  optimized_coefficients.segment<4>(0)  = transform.row (0);
  optimized_coefficients.segment<4>(4)  = transform.row (1);
  optimized_coefficients.segment<4>(8)  = transform.row (2);
  optimized_coefficients.segment<4>(12) = transform.row (3);
}

#define PCL_INSTANTIATE_SampleConsensusModelRegistration(T) template class PCL_EXPORTS pcl::SampleConsensusModelRegistration<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_

