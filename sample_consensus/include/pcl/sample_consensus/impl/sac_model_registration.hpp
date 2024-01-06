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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_

#include <pcl/sample_consensus/sac_model_registration.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistration<PointT>::isSampleGood (const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }
  using namespace pcl::common;
  using namespace pcl::traits;

  PointT p10 = (*input_)[samples[1]] - (*input_)[samples[0]];
  PointT p20 = (*input_)[samples[2]] - (*input_)[samples[0]];
  PointT p21 = (*input_)[samples[2]] - (*input_)[samples[1]];

  return ((p10.x * p10.x + p10.y * p10.y + p10.z * p10.z) > sample_dist_thresh_ && 
          (p20.x * p20.x + p20.y * p20.y + p20.z * p20.z) > sample_dist_thresh_ && 
          (p21.x * p21.x + p21.y * p21.y + p21.z * p21.z) > sample_dist_thresh_);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistration<PointT>::computeModelCoefficients (const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeModelCoefficients] No target dataset given!\n");
    return (false);
  }
  // Need 3 samples
  if (samples.size () != sample_size_)
  {
    return (false);
  }

  Indices indices_tgt (3);
  for (int i = 0; i < 3; ++i)
  {
    const auto it = correspondences_.find (samples[i]);
    if (it == correspondences_.cend ())
    {
      PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeModelCoefficients] Element with key %i is not in map (map contains %lu elements).\n",
                 samples[i], correspondences_.size ());
      return (false);
    }
    indices_tgt[i] = it->second;
  }

  estimateRigidTransformationSVD (*input_, samples, *target_, indices_tgt, model_coefficients);
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::getDistancesToModel] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    distances.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::getDistanceToModel] No target dataset given!\n");
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
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src ((*input_)[(*indices_)[i]].x, 
                            (*input_)[(*indices_)[i]].y, 
                            (*input_)[(*indices_)[i]].z, 1.0f);
    Eigen::Vector4f pt_tgt ((*target_)[(*indices_tgt_)[i]].x, 
                            (*target_)[(*indices_tgt_)[i]].y, 
                            (*target_)[(*indices_tgt_)[i]].z, 1.0f);

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    // need to compute the real norm here to keep MSAC and friends general
    distances[i] = (p_tr - pt_tgt).norm ();
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::selectWithinDistance (const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::selectWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    inliers.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::selectWithinDistance] No target dataset given!\n");
    return;
  }

  double thresh = threshold * threshold;

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }
  
  inliers.clear ();
  error_sqr_dists_.clear ();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src ((*input_)[(*indices_)[i]].x, 
                            (*input_)[(*indices_)[i]].y, 
                            (*input_)[(*indices_)[i]].z, 1); 
    Eigen::Vector4f pt_tgt ((*target_)[(*indices_tgt_)[i]].x, 
                            (*target_)[(*indices_tgt_)[i]].y, 
                            (*target_)[(*indices_tgt_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);
  
    float distance = (p_tr - pt_tgt).squaredNorm (); 
    // Calculate the distance from the transformed point to its correspondence
    if (distance < thresh)
    {
      inliers.push_back ((*indices_)[i]);
      error_sqr_dists_.push_back (static_cast<double> (distance));
    }
  }
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelRegistration<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::countWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    return (0);
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::countWithinDistance] No target dataset given!\n");
    return (0);
  }

  double thresh = threshold * threshold;

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    return (0);
  }
  
  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  std::size_t nr_p = 0;
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src ((*input_)[(*indices_)[i]].x, 
                            (*input_)[(*indices_)[i]].y, 
                            (*input_)[(*indices_)[i]].z, 1); 
    Eigen::Vector4f pt_tgt ((*target_)[(*indices_tgt_)[i]].x, 
                            (*target_)[(*indices_tgt_)[i]].y, 
                            (*target_)[(*indices_tgt_)[i]].z, 1); 

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    if ((p_tr - pt_tgt).squaredNorm () < thresh)
      nr_p++;
  }
  PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::countWithinDistance] %zu inliers of %zu total points, threshold=%g\n", nr_p, indices_->size(), threshold);
  return (nr_p);
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::optimizeModelCoefficients (const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::optimizeModelCoefficients] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients) || !target_)
  {
    optimized_coefficients = model_coefficients;
    return;
  }

  Indices indices_src (inliers.size ());
  Indices indices_tgt (inliers.size ());
  for (std::size_t i = 0; i < inliers.size (); ++i)
  {
    indices_src[i] = inliers[i];
    const auto it = correspondences_.find (indices_src[i]);
    if (it == correspondences_.cend ())
    {
      PCL_ERROR ("[pcl::SampleConsensusModelRegistration::optimizeModelCoefficients] Element with key %i is not in map (map contains %lu elements).\n",
                 indices_src[i], correspondences_.size ());
      optimized_coefficients = model_coefficients;
      return;
    }
    indices_tgt[i] = it->second;
  }

  estimateRigidTransformationSVD (*input_, indices_src, *target_, indices_tgt, optimized_coefficients);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::estimateRigidTransformationSVD (
    const pcl::PointCloud<PointT> &cloud_src,
    const Indices &indices_src,
    const pcl::PointCloud<PointT> &cloud_tgt,
    const Indices &indices_tgt,
    Eigen::VectorXf &transform) const
{
  transform.resize (16);

  Eigen::Matrix<double, 3, Eigen::Dynamic> src (3, indices_src.size ());
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt (3, indices_tgt.size ());

  for (std::size_t i = 0; i < indices_src.size (); ++i)
  {
    src (0, i) = cloud_src[indices_src[i]].x;
    src (1, i) = cloud_src[indices_src[i]].y;
    src (2, i) = cloud_src[indices_src[i]].z;

    tgt (0, i) = cloud_tgt[indices_tgt[i]].x;
    tgt (1, i) = cloud_tgt[indices_tgt[i]].y;
    tgt (2, i) = cloud_tgt[indices_tgt[i]].z;
  }

  // Call Umeyama directly from Eigen
  PCL_DEBUG_STREAM("[pcl::SampleConsensusModelRegistration::estimateRigidTransformationSVD] src and tgt:" << std::endl << src << std::endl << std::endl << tgt << std::endl);
  Eigen::Matrix4d transformation_matrix = pcl::umeyama (src, tgt, false);

  // Return the correct transformation
  transform.segment<4> (0).matrix () = transformation_matrix.cast<float> ().row (0); 
  transform.segment<4> (4).matrix () = transformation_matrix.cast<float> ().row (1);
  transform.segment<4> (8).matrix () = transformation_matrix.cast<float> ().row (2);
  transform.segment<4> (12).matrix () = transformation_matrix.cast<float> ().row (3);
}

#define PCL_INSTANTIATE_SampleConsensusModelRegistration(T) template class PCL_EXPORTS pcl::SampleConsensusModelRegistration<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_

