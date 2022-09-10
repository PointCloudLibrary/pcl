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

#pragma once

#include <pcl/features/principal_curvatures.h>

#include <pcl/common/point_tests.h> // for pcl::isFinite


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>::computePointPrincipalCurvatures (
      const pcl::PointCloud<PointNT> &normals, int p_idx, const pcl::Indices &indices,
      float &pcx, float &pcy, float &pcz, float &pc1, float &pc2)
{
  EIGEN_ALIGN16 Eigen::Matrix3f I = Eigen::Matrix3f::Identity ();
  Eigen::Vector3f n_idx (normals[p_idx].normal[0], normals[p_idx].normal[1], normals[p_idx].normal[2]);
  EIGEN_ALIGN16 Eigen::Matrix3f M = I - n_idx * n_idx.transpose ();    // projection matrix (into tangent plane)

  // Project normals into the tangent plane
  Eigen::Vector3f normal;
  projected_normals_.resize (indices.size ());
  xyz_centroid_.setZero ();
  for (std::size_t idx = 0; idx < indices.size(); ++idx)
  {
    normal[0] = normals[indices[idx]].normal[0];
    normal[1] = normals[indices[idx]].normal[1];
    normal[2] = normals[indices[idx]].normal[2];

    projected_normals_[idx] = M * normal;
    xyz_centroid_ += projected_normals_[idx];
  }

  // Estimate the XYZ centroid
  xyz_centroid_ /= static_cast<float> (indices.size ());

  // Initialize to 0
  covariance_matrix_.setZero ();

  // For each point in the cloud
  for (std::size_t idx = 0; idx < indices.size (); ++idx)
  {
    demean_ = projected_normals_[idx] - xyz_centroid_;

    double demean_xy = demean_[0] * demean_[1];
    double demean_xz = demean_[0] * demean_[2];
    double demean_yz = demean_[1] * demean_[2];

    covariance_matrix_(0, 0) += demean_[0] * demean_[0];
    covariance_matrix_(0, 1) += static_cast<float> (demean_xy);
    covariance_matrix_(0, 2) += static_cast<float> (demean_xz);

    covariance_matrix_(1, 0) += static_cast<float> (demean_xy);
    covariance_matrix_(1, 1) += demean_[1] * demean_[1];
    covariance_matrix_(1, 2) += static_cast<float> (demean_yz);

    covariance_matrix_(2, 0) += static_cast<float> (demean_xz);
    covariance_matrix_(2, 1) += static_cast<float> (demean_yz);
    covariance_matrix_(2, 2) += demean_[2] * demean_[2];
  }

  // Extract the eigenvalues and eigenvectors
  pcl::eigen33 (covariance_matrix_, eigenvalues_);
  pcl::computeCorrespondingEigenVector (covariance_matrix_, eigenvalues_ [2], eigenvector_);

  pcx = eigenvector_ [0];
  pcy = eigenvector_ [1];
  pcz = eigenvector_ [2];
  float indices_size = 1.0f / static_cast<float> (indices.size ());
  pc1 = eigenvalues_ [2] * indices_size;
  pc2 = eigenvalues_ [1] * indices_size;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  pcl::Indices nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output[idx].principal_curvature[0] = output[idx].principal_curvature[1] = output[idx].principal_curvature[2] =
          output[idx].pc1 = output[idx].pc2 = std::numeric_limits<float>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      // Estimate the principal curvatures at each patch
      computePointPrincipalCurvatures (*normals_, (*indices_)[idx], nn_indices,
                                       output[idx].principal_curvature[0], output[idx].principal_curvature[1], output[idx].principal_curvature[2],
                                       output[idx].pc1, output[idx].pc2);
    }
  }
  else
  {
    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (!isFinite ((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output[idx].principal_curvature[0] = output[idx].principal_curvature[1] = output[idx].principal_curvature[2] =
          output[idx].pc1 = output[idx].pc2 = std::numeric_limits<float>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      // Estimate the principal curvatures at each patch
      computePointPrincipalCurvatures (*normals_, (*indices_)[idx], nn_indices,
                                       output[idx].principal_curvature[0], output[idx].principal_curvature[1], output[idx].principal_curvature[2],
                                       output[idx].pc1, output[idx].pc2);
    }
  }
}

#define PCL_INSTANTIATE_PrincipalCurvaturesEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::PrincipalCurvaturesEstimation<T,NT,OutT>;

