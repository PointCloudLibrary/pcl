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
 * $Id$
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_

#include <pcl/sample_consensus/sac_model_registration.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistration<PointT>::isSampleGood (const std::vector<int> &samples) const
{
  return ((input_->points[samples[1]].getArray4fMap () - input_->points[samples[0]].getArray4fMap ()).matrix ().squaredNorm () > sample_dist_thresh_ && 
          (input_->points[samples[2]].getArray4fMap () - input_->points[samples[0]].getArray4fMap ()).matrix ().squaredNorm () > sample_dist_thresh_ && 
          (input_->points[samples[2]].getArray4fMap () - input_->points[samples[1]].getArray4fMap ()).matrix ().squaredNorm () > sample_dist_thresh_);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelRegistration<PointT>::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeModelCoefficients] No target dataset given!\n");
    return (false);
  }
  // Need 3 samples
  if (samples.size () != 3)
    return (false);

  std::vector<int> indices_tgt (3);
  for (int i = 0; i < 3; ++i)
    indices_tgt[i] = correspondences_[samples[i]];

  estimateRigidTransformationSVD (*input_, samples, *target_, indices_tgt, model_coefficients);
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::getDistancesToModel] Number of source indices (%zu) differs than number of target indices (%zu)!\n", indices_->size (), indices_tgt_->size ());
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
  transform.row (0) = model_coefficients.segment<4>(0);
  transform.row (1) = model_coefficients.segment<4>(4);
  transform.row (2) = model_coefficients.segment<4>(8);
  transform.row (3) = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src = input_->points[(*indices_)[i]].getVector4fMap ();
    pt_src[3] = 1;
    Eigen::Vector4f pt_tgt = target_->points[(*indices_tgt_)[i]].getVector4fMap ();
    pt_tgt[3] = 1;

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    // need to compute the real norm here to keep MSAC and friends general
    distances[i] = (p_tr - pt_tgt).norm ();
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::selectWithinDistance (const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::selectWithinDistance] Number of source indices (%zu) differs than number of target indices (%zu)!\n", indices_->size (), indices_tgt_->size ());
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
  
  inliers.resize (indices_->size ());

  Eigen::Matrix4f transform;
  transform.row (0) = model_coefficients.segment<4>(0);
  transform.row (1) = model_coefficients.segment<4>(4);
  transform.row (2) = model_coefficients.segment<4>(8);
  transform.row (3) = model_coefficients.segment<4>(12);

  int nr_p = 0; 
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src = input_->points[(*indices_)[i]].getVector4fMap ();
    pt_src[3] = 1;
    Eigen::Vector4f pt_tgt = target_->points[(*indices_tgt_)[i]].getVector4fMap ();
    pt_tgt[3] = 1;

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    if ((p_tr - pt_tgt).squaredNorm () < thresh)
      inliers[nr_p++] = (*indices_)[i];
  }
  inliers.resize (nr_p);
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SampleConsensusModelRegistration<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold)
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::countWithinDistance] Number of source indices (%zu) differs than number of target indices (%zu)!\n", indices_->size (), indices_tgt_->size ());
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
    return (0);
  
  Eigen::Matrix4f transform;
  transform.row (0) = model_coefficients.segment<4>(0);
  transform.row (1) = model_coefficients.segment<4>(4);
  transform.row (2) = model_coefficients.segment<4>(8);
  transform.row (3) = model_coefficients.segment<4>(12);

  int nr_p = 0; 
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt_src = input_->points[(*indices_)[i]].getVector4fMap ();
    pt_src[3] = 1;
    Eigen::Vector4f pt_tgt = target_->points[(*indices_tgt_)[i]].getVector4fMap ();
    pt_tgt[3] = 1;

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    if ((p_tr - pt_tgt).squaredNorm () < thresh)
      nr_p++;
  }
  return (nr_p);
} 

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) 
{
  if (indices_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelRegistration::optimizeModelCoefficients] Number of source indices (%zu) differs than number of target indices (%zu)!\n", indices_->size (), indices_tgt_->size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients) || !target_)
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

  estimateRigidTransformationSVD (*input_, indices_src, *target_, indices_tgt, optimized_coefficients);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelRegistration<PointT>::estimateRigidTransformationSVD (
    const pcl::PointCloud<PointT> &cloud_src, 
    const std::vector<int> &indices_src, 
    const pcl::PointCloud<PointT> &cloud_tgt,
    const std::vector<int> &indices_tgt, 
    Eigen::VectorXf &transform)
{
  transform.resize (16);
  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, indices_src, centroid_src);
  compute3DCentroid (cloud_tgt, indices_tgt, centroid_tgt);

  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, indices_tgt, centroid_tgt, cloud_tgt_demean);

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU ();
  Eigen::Matrix3f v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix3f R = v * u.transpose ();

  // Return the correct transformation
  transform.segment<3> (0) = R.row (0); transform[12]  = 0;
  transform.segment<3> (4) = R.row (1); transform[13]  = 0;
  transform.segment<3> (8) = R.row (2); transform[14] = 0;

  Eigen::Vector3f t = centroid_tgt.head<3> () - R * centroid_src.head<3> ();
  transform[3] = t[0]; transform[7] = t[1]; transform[11] = t[2]; transform[15] = 1.0;
}

#define PCL_INSTANTIATE_SampleConsensusModelRegistration(T) template class PCL_EXPORTS pcl::SampleConsensusModelRegistration<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_

