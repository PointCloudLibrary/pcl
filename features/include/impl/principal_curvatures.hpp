/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: principal_curvatures.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_PRINCIPAL_CURVATURES_H_
#define PCL_FEATURES_IMPL_PRINCIPAL_CURVATURES_H_

#include "pcl/features/principal_curvatures.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>::computePointPrincipalCurvatures (
      const pcl::PointCloud<PointNT> &normals, int p_idx, const std::vector<int> &indices,
      float &pcx, float &pcy, float &pcz, float &pc1, float &pc2)
{
  EIGEN_ALIGN16 Eigen::Matrix3f I = Eigen::Matrix3f::Identity ();
  Eigen::Vector3f n_idx (normals.points[p_idx].normal[0], normals.points[p_idx].normal[1], normals.points[p_idx].normal[2]);
  EIGEN_ALIGN16 Eigen::Matrix3f M = I - n_idx * n_idx.transpose ();    // projection matrix (into tangent plane)

  // Project normals into the tangent plane
  Eigen::Vector3f normal;
  projected_normals_.resize (indices.size ());
  xyz_centroid_.setZero ();
  for (size_t idx = 0; idx < indices.size(); ++idx)
  {
    normal[0] = normals.points[indices[idx]].normal[0];
    normal[1] = normals.points[indices[idx]].normal[1];
    normal[2] = normals.points[indices[idx]].normal[2];

    projected_normals_[idx] = M * normal;
    xyz_centroid_ += projected_normals_[idx];
  }

  // Estimate the XYZ centroid
  xyz_centroid_ /= indices.size ();

  // Initialize to 0
  covariance_matrix_.setZero ();

  double demean_xy, demean_xz, demean_yz;
  // For each point in the cloud
  for (size_t idx = 0; idx < indices.size (); ++idx)
  {
    demean_ = projected_normals_[idx] - xyz_centroid_;

    demean_xy = demean_[0] * demean_[1];
    demean_xz = demean_[0] * demean_[2];
    demean_yz = demean_[1] * demean_[2];

    covariance_matrix_(0, 0) += demean_[0] * demean_[0];
    covariance_matrix_(0, 1) += demean_xy;
    covariance_matrix_(0, 2) += demean_xz;

    covariance_matrix_(1, 0) += demean_xy;
    covariance_matrix_(1, 1) += demean_[1] * demean_[1];
    covariance_matrix_(1, 2) += demean_yz;

    covariance_matrix_(2, 0) += demean_xz;
    covariance_matrix_(2, 1) += demean_yz;
    covariance_matrix_(2, 2) += demean_[2] * demean_[2];
  }

  // Extract the eigenvalues and eigenvectors
  //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix_);
  //eigenvalues_  = ei_symm.eigenvalues ();
  //eigenvectors_ = ei_symm.eigenvectors ();
  pcl::eigen33 (covariance_matrix_, eigenvectors_, eigenvalues_);
 
  pcx = eigenvectors_ (0, 2);
  pcy = eigenvectors_ (1, 2);
  pcz = eigenvectors_ (2, 2);
  pc1 = eigenvalues_ (2);
  pc2 = eigenvalues_ (1);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    ROS_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    ROS_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset (%zu) differs from the number of points in the dataset containing the normals (%zu)!",
               getClassName ().c_str (), surface_->points.size (), normals_->points.size ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the principal curvatures at each patch
    computePointPrincipalCurvatures (*normals_, (*indices_)[idx], nn_indices,
                                     output.points[idx].principal_curvature[0], output.points[idx].principal_curvature[1], output.points[idx].principal_curvature[2],
                                     output.points[idx].pc1, output.points[idx].pc2);
  }
}

#define PCL_INSTANTIATE_PrincipalCurvaturesEstimation(T,NT,OutT) template class pcl::PrincipalCurvaturesEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_PRINCIPAL_CURVATURES_H_ 
