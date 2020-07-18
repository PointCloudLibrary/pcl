/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/type_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>

/**
  * \file pcl/common/centroid.h
  * Define methods for centroid estimation and covariance matrix calculus
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points and return it as a 3D vector.
    * \param[in] cloud_iterator an iterator over the input point cloud
    * \param[out] centroid the output centroid
    * \return number of valid points used to determine the centroid. In case of dense point clouds, this is the same as the size of input cloud.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * The last component of the vector is set to 1, this allows to transform the centroid vector with 4x4 matrices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  compute3DCentroid (ConstCloudIterator<PointT> &cloud_iterator,
                     Eigen::Matrix<Scalar, 4, 1> &centroid);

  template <typename PointT> inline unsigned int
  compute3DCentroid (ConstCloudIterator<PointT> &cloud_iterator,
                     Eigen::Vector4f &centroid)
  {
    return (compute3DCentroid <PointT, float> (cloud_iterator, centroid));
  }

  template <typename PointT> inline unsigned int
  compute3DCentroid (ConstCloudIterator<PointT> &cloud_iterator,
                     Eigen::Vector4d &centroid)
  {
    return (compute3DCentroid <PointT, double> (cloud_iterator, centroid));
  }

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points and return it as a 3D vector.
    * \param[in] cloud the input point cloud
    * \param[out] centroid the output centroid
    * \return number of valid points used to determine the centroid. In case of dense point clouds, this is the same as the size of input cloud.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * The last component of the vector is set to 1, this allows to transform the centroid vector with 4x4 matrices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::Matrix<Scalar, 4, 1> &centroid);

  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::Vector4f &centroid)
  {
    return (compute3DCentroid <PointT, float> (cloud, centroid));
  }

  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::Vector4d &centroid)
  {
    return (compute3DCentroid <PointT, double> (cloud, centroid));
  }

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and
    * return it as a 3D vector.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[out] centroid the output centroid
    * \return number of valid points used to determine the centroid. In case of dense point clouds, this is the same as the size of input indices.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * The last component of the vector is set to 1, this allows to transform the centroid vector with 4x4 matrices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const Indices &indices,
                     Eigen::Matrix<Scalar, 4, 1> &centroid);

  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const Indices &indices,
                     Eigen::Vector4f &centroid)
  {
    return (compute3DCentroid <PointT, float> (cloud, indices, centroid));
  }

  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const Indices &indices,
                     Eigen::Vector4d &centroid)
  {
    return (compute3DCentroid <PointT, double> (cloud, indices, centroid));
  }

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and
    * return it as a 3D vector.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[out] centroid the output centroid
    * \return number of valid points used to determine the centroid. In case of dense point clouds, this is the same as the size of input indices.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * The last component of the vector is set to 1, this allows to transform the centroid vector with 4x4 matrices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const pcl::PointIndices &indices,
                     Eigen::Matrix<Scalar, 4, 1> &centroid);

  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const pcl::PointIndices &indices,
                     Eigen::Vector4f &centroid)
  {
    return (compute3DCentroid <PointT, float> (cloud, indices, centroid));
  }

  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const pcl::PointIndices &indices,
                     Eigen::Vector4d &centroid)
  {
    return (compute3DCentroid <PointT, double> (cloud, indices, centroid));
  }

  /** \brief Compute the 3x3 covariance matrix of a given set of points.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeCovarianceMatrixNormalized.
    * \param[in] cloud the input point cloud
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \note if return value is 0, the covariance matrix is not changed, thus not valid.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Eigen::Matrix<Scalar, 4, 1> &centroid,
                           Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Eigen::Vector4f &centroid,
                           Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, float> (cloud, centroid, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Eigen::Vector4d &centroid,
                           Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, double> (cloud, centroid, covariance_matrix));
  }

  /** \brief Compute normalized the 3x3 covariance matrix of a given set of points.
    * The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of points in the point cloud.
    * For small number of points, or if you want explicitly the sample-variance, use computeCovarianceMatrix
    * and scale the covariance matrix with 1 / (n-1), where n is the number of points used to calculate
    * the covariance matrix and is returned by the computeCovarianceMatrix function.
    * \param[in] cloud the input point cloud
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const Eigen::Matrix<Scalar, 4, 1> &centroid,
                                     Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const Eigen::Vector4f &centroid,
                                     Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrixNormalized<PointT, float> (cloud, centroid, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const Eigen::Vector4d &centroid,
                                     Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrixNormalized<PointT, double> (cloud, centroid, covariance_matrix));
  }

  /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeCovarianceMatrixNormalized.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Indices &indices,
                           const Eigen::Matrix<Scalar, 4, 1> &centroid,
                           Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Indices &indices,
                           const Eigen::Vector4f &centroid,
                           Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, float> (cloud, indices, centroid, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Indices &indices,
                           const Eigen::Vector4d &centroid,
                           Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, double> (cloud, indices, centroid, covariance_matrix));
  }

  /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeCovarianceMatrixNormalized.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           const Eigen::Matrix<Scalar, 4, 1> &centroid,
                           Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           const Eigen::Vector4f &centroid,
                           Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, float> (cloud, indices, centroid, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           const Eigen::Vector4d &centroid,
                           Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, double> (cloud, indices, centroid, covariance_matrix));
  }

  /** \brief Compute the normalized 3x3 covariance matrix of a given set of points using
    * their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitly the sample-variance, use computeCovarianceMatrix
    * and scale the covariance matrix with 1 / (n-1), where n is the number of points used to calculate
    * the covariance matrix and is returned by the computeCovarianceMatrix function.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const Indices &indices,
                                     const Eigen::Matrix<Scalar, 4, 1> &centroid,
                                     Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const Indices &indices,
                                     const Eigen::Vector4f &centroid,
                                     Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrixNormalized<PointT, float> (cloud, indices, centroid, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const Indices &indices,
                                     const Eigen::Vector4d &centroid,
                                     Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrixNormalized<PointT, double> (cloud, indices, centroid, covariance_matrix));
  }

  /** \brief Compute the normalized 3x3 covariance matrix of a given set of points using
    * their indices. The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitly the sample-variance, use computeCovarianceMatrix
    * and scale the covariance matrix with 1 / (n-1), where n is the number of points used to calculate
    * the covariance matrix and is returned by the computeCovarianceMatrix function.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const pcl::PointIndices &indices,
                                     const Eigen::Matrix<Scalar, 4, 1> &centroid,
                                     Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const pcl::PointIndices &indices,
                                     const Eigen::Vector4f &centroid,
                                     Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrixNormalized<PointT, float> (cloud, indices, centroid, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const pcl::PointIndices &indices,
                                     const Eigen::Vector4d &centroid,
                                     Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrixNormalized<PointT, double> (cloud, indices, centroid, covariance_matrix));
  }

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of valid entries in the point cloud.
    * For small number of points, or if you want explicitly the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \param[out] centroid the centroid of the set of points in the cloud
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  Eigen::Matrix<Scalar, 3, 3> &covariance_matrix,
                                  Eigen::Matrix<Scalar, 4, 1> &centroid);

  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  Eigen::Matrix3f &covariance_matrix,
                                  Eigen::Vector4f &centroid)
  {
    return (computeMeanAndCovarianceMatrix<PointT, float> (cloud, covariance_matrix, centroid));
  }

  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  Eigen::Matrix3d &covariance_matrix,
                                  Eigen::Vector4d &centroid)
  {
    return (computeMeanAndCovarianceMatrix<PointT, double> (cloud, covariance_matrix, centroid));
  }

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitly the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \param[out] centroid the centroid of the set of points in the cloud
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const Indices &indices,
                                  Eigen::Matrix<Scalar, 3, 3> &covariance_matrix,
                                  Eigen::Matrix<Scalar, 4, 1> &centroid);

  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const Indices &indices,
                                  Eigen::Matrix3f &covariance_matrix,
                                  Eigen::Vector4f &centroid)
  {
    return (computeMeanAndCovarianceMatrix<PointT, float> (cloud, indices, covariance_matrix, centroid));
  }

  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const Indices &indices,
                                  Eigen::Matrix3d &covariance_matrix,
                                  Eigen::Vector4d &centroid)
  {
    return (computeMeanAndCovarianceMatrix<PointT, double> (cloud, indices, covariance_matrix, centroid));
  }

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitly the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const pcl::PointIndices &indices,
                                  Eigen::Matrix<Scalar, 3, 3> &covariance_matrix,
                                  Eigen::Matrix<Scalar, 4, 1> &centroid);

  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const pcl::PointIndices &indices,
                                  Eigen::Matrix3f &covariance_matrix,
                                  Eigen::Vector4f &centroid)
  {
    return (computeMeanAndCovarianceMatrix<PointT, float> (cloud, indices, covariance_matrix, centroid));
  }

  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const pcl::PointIndices &indices,
                                  Eigen::Matrix3d &covariance_matrix,
                                  Eigen::Vector4d &centroid)
  {
    return (computeMeanAndCovarianceMatrix<PointT, double> (cloud, indices, covariance_matrix, centroid));
  }

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in the input point cloud.
    * For small number of points, or if you want explicitly the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, float> (cloud, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, double> (cloud, covariance_matrix));
  }

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitly the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Indices &indices,
                           Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Indices &indices,
                           Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, float> (cloud, indices, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Indices &indices,
                           Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, double> (cloud, indices, covariance_matrix));
  }

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitly the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid points used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input indices.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           Eigen::Matrix<Scalar, 3, 3> &covariance_matrix);

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           Eigen::Matrix3f &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, float> (cloud, indices, covariance_matrix));
  }

  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           Eigen::Matrix3d &covariance_matrix)
  {
    return (computeCovarianceMatrix<PointT, double> (cloud, indices, covariance_matrix));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param[in] cloud_iterator an iterator over the input point cloud
    * \param[in] centroid the centroid of the point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] npts the number of samples guaranteed to be left in the input cloud, accessible by the iterator. If not given, it will be calculated.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    pcl::PointCloud<PointT> &cloud_out,
                    int npts = 0);

  template <typename PointT> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Vector4f &centroid,
                    pcl::PointCloud<PointT> &cloud_out,
                    int npts = 0)
  {
    return (demeanPointCloud<PointT, float> (cloud_iterator, centroid, cloud_out, npts));
  }

  template <typename PointT> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Vector4d &centroid,
                    pcl::PointCloud<PointT> &cloud_out,
                    int npts = 0)
  {
    return (demeanPointCloud<PointT, double> (cloud_iterator, centroid, cloud_out, npts));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param[in] cloud_in the input point cloud
    * \param[in] centroid the centroid of the point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    pcl::PointCloud<PointT> &cloud_out);

  template <typename PointT> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Vector4f &centroid,
                    pcl::PointCloud<PointT> &cloud_out)
  {
    return (demeanPointCloud<PointT, float> (cloud_iterator, centroid, cloud_out));
  }

  template <typename PointT> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Vector4d &centroid,
                    pcl::PointCloud<PointT> &cloud_out)
  {
    return (demeanPointCloud<PointT, double> (cloud_iterator, centroid, cloud_out));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] centroid the centroid of the point cloud
    * \param cloud_out the resultant output point cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Indices &indices,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    pcl::PointCloud<PointT> &cloud_out);

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Indices &indices,
                    const Eigen::Vector4f &centroid,
                    pcl::PointCloud<PointT> &cloud_out)
  {
    return (demeanPointCloud<PointT, float> (cloud_in, indices, centroid, cloud_out));
  }

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Indices &indices,
                    const Eigen::Vector4d &centroid,
                    pcl::PointCloud<PointT> &cloud_out)
  {
    return (demeanPointCloud<PointT, double> (cloud_in, indices, centroid, cloud_out));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] centroid the centroid of the point cloud
    * \param cloud_out the resultant output point cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const pcl::PointIndices& indices,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    pcl::PointCloud<PointT> &cloud_out);

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const pcl::PointIndices& indices,
                    const Eigen::Vector4f &centroid,
                    pcl::PointCloud<PointT> &cloud_out)
  {
    return (demeanPointCloud<PointT, float> (cloud_in, indices, centroid, cloud_out));
  }

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const pcl::PointIndices& indices,
                    const Eigen::Vector4d &centroid,
                    pcl::PointCloud<PointT> &cloud_out)
  {
    return (demeanPointCloud<PointT, double> (cloud_in, indices, centroid, cloud_out));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param[in] cloud_iterator an iterator over the input point cloud
    * \param[in] centroid the centroid of the point cloud
    * \param[out] cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    * \param[in] npts the number of samples guaranteed to be left in the input cloud, accessible by the iterator. If not given, it will be calculated.
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out,
                    int npts = 0);

  template <typename PointT> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Vector4f &centroid,
                    Eigen::MatrixXf &cloud_out,
                    int npts = 0)
  {
    return (demeanPointCloud<PointT, float> (cloud_iterator, centroid, cloud_out, npts));
  }

  template <typename PointT> void
  demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                    const Eigen::Vector4d &centroid,
                    Eigen::MatrixXd &cloud_out,
                    int npts = 0)
  {
    return (demeanPointCloud<PointT, double> (cloud_iterator, centroid, cloud_out, npts));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param[in] cloud_in the input point cloud
    * \param[in] centroid the centroid of the point cloud
    * \param[out] cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out);

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Eigen::Vector4f &centroid,
                    Eigen::MatrixXf &cloud_out)
  {
    return (demeanPointCloud<PointT, float> (cloud_in, centroid, cloud_out));
  }

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Eigen::Vector4d &centroid,
                    Eigen::MatrixXd &cloud_out)
  {
    return (demeanPointCloud<PointT, double> (cloud_in, centroid, cloud_out));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[in] centroid the centroid of the point cloud
    * \param[out] cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Indices &indices,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out);

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Indices &indices,
                    const Eigen::Vector4f &centroid,
                    Eigen::MatrixXf &cloud_out)
  {
    return (demeanPointCloud<PointT, float> (cloud_in, indices, centroid, cloud_out));
  }

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Indices &indices,
                    const Eigen::Vector4d &centroid,
                    Eigen::MatrixXd &cloud_out)
  {
    return (demeanPointCloud<PointT, double> (cloud_in, indices, centroid, cloud_out));
  }

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[in] centroid the centroid of the point cloud
    * \param[out] cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const pcl::PointIndices& indices,
                    const Eigen::Matrix<Scalar, 4, 1> &centroid,
                    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out);

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const pcl::PointIndices& indices,
                    const Eigen::Vector4f &centroid,
                    Eigen::MatrixXf &cloud_out)
  {
    return (demeanPointCloud<PointT, float> (cloud_in, indices, centroid, cloud_out));
  }

  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const pcl::PointIndices& indices,
                    const Eigen::Vector4d &centroid,
                    Eigen::MatrixXd &cloud_out)
  {
    return (demeanPointCloud<PointT, double> (cloud_in, indices, centroid, cloud_out));
  }

  /** \brief Helper functor structure for n-D centroid estimation. */
  template<typename PointT, typename Scalar>
  struct NdCentroidFunctor
  {
    using Pod = typename traits::POD<PointT>::type;

    NdCentroidFunctor (const PointT &p, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid)
      : f_idx_ (0),
        centroid_ (centroid),
        p_ (reinterpret_cast<const Pod&>(p)) { }

    template<typename Key> inline void operator() ()
    {
      using T = typename pcl::traits::datatype<PointT, Key>::type;
      const std::uint8_t* raw_ptr = reinterpret_cast<const std::uint8_t*>(&p_) + pcl::traits::offset<PointT, Key>::value;
      const T* data_ptr = reinterpret_cast<const T*>(raw_ptr);

      // Check if the value is invalid
      if (!std::isfinite (*data_ptr))
      {
        f_idx_++;
        return;
      }

      centroid_[f_idx_++] += *data_ptr;
    }

    private:
      int f_idx_;
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid_;
      const Pod &p_;
  };

  /** \brief General, all purpose nD centroid estimation for a set of points using their
    * indices.
    * \param cloud the input point cloud
    * \param centroid the output centroid
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid);

  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::VectorXf &centroid)
  {
    return (computeNDCentroid<PointT, float> (cloud, centroid));
  }

  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::VectorXd &centroid)
  {
    return (computeNDCentroid<PointT, double> (cloud, centroid));
  }

  /** \brief General, all purpose nD centroid estimation for a set of points using their
    * indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                     const Indices &indices,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid);

  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const Indices &indices,
                     Eigen::VectorXf &centroid)
  {
    return (computeNDCentroid<PointT, float> (cloud, indices, centroid));
  }

  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const Indices &indices,
                     Eigen::VectorXd &centroid)
  {
    return (computeNDCentroid<PointT, double> (cloud, indices, centroid));
  }

  /** \brief General, all purpose nD centroid estimation for a set of points using their
    * indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                     const pcl::PointIndices &indices,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid);

  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const pcl::PointIndices &indices,
                     Eigen::VectorXf &centroid)
  {
    return (computeNDCentroid<PointT, float> (cloud, indices, centroid));
  }

  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const pcl::PointIndices &indices,
                     Eigen::VectorXd &centroid)
  {
    return (computeNDCentroid<PointT, double> (cloud, indices, centroid));
  }

}

#include <pcl/common/impl/accumulators.hpp>

namespace pcl
{

  /** A generic class that computes the centroid of points fed to it.
    *
    * Here by "centroid" we denote not just the mean of 3D point coordinates,
    * but also mean of values in the other data fields. The general-purpose
    * \ref computeNDCentroid() function also implements this sort of
    * functionality, however it does it in a "dumb" way, i.e. regardless of the
    * semantics of the data inside a field it simply averages the values. In
    * certain cases (e.g. for \c x, \c y, \c z, \c intensity fields) this
    * behavior is reasonable, however in other cases (e.g. \c rgb, \c rgba,
    * \c label fields) this does not lead to meaningful results.
    *
    * This class is capable of computing the centroid in a "smart" way, i.e.
    * taking into account the meaning of the data inside fields. Currently the
    * following fields are supported:
    *
    *  Data      | Point fields                          | Algorithm
    *  --------- | ------------------------------------- | -------------------------------------------------------------------------------------------
    *  XYZ       | \c x, \c y, \c z                      | Average (separate for each field)
    *  Normal    | \c normal_x, \c normal_y, \c normal_z | Average (separate for each field), resulting vector is normalized
    *  Curvature | \c curvature                          | Average
    *  Color     | \c rgb or \c rgba                     | Average (separate for R, G, B, and alpha channels)
    *  Intensity | \c intensity                          | Average
    *  Label     | \c label                              | Majority vote; if several labels have the same largest support then the  smaller label wins
    *
    * The template parameter defines the type of points that may be accumulated
    * with this class. This may be an arbitrary PCL point type, and centroid
    * computation will happen only for the fields that are present in it and are
    * supported.
    *
    * Current centroid may be retrieved at any time using get(). Note that the
    * function is templated on point type, so it is possible to fetch the
    * centroid into a point type that differs from the type of points that are
    * being accumulated. All the "extra" fields for which the centroid is not
    * being calculated will be left untouched.
    *
    * Example usage:
    *
    * \code
    * // Create and accumulate points
    * CentroidPoint<pcl::PointXYZ> centroid;
    * centroid.add (pcl::PointXYZ (1, 2, 3);
    * centroid.add (pcl::PointXYZ (5, 6, 7);
    * // Fetch centroid using `get()`
    * pcl::PointXYZ c1;
    * centroid.get (c1);
    * // The expected result is: c1.x == 3, c1.y == 4, c1.z == 5
    * // It is also okay to use `get()` with a different point type
    * pcl::PointXYZRGB c2;
    * centroid.get (c2);
    * // The expected result is: c2.x == 3, c2.y == 4, c2.z == 5,
    * // and c2.rgb is left untouched
    * \endcode
    *
    * \note Assumes that the points being inserted are valid.
    *
    * \note This class template can be successfully instantiated for *any*
    * PCL point type. Of course, each of the field averages is computed only if
    * the point type has the corresponding field.
    *
    * \ingroup common
    * \author Sergey Alexandrov */
  template <typename PointT>
  class CentroidPoint
  {

    public:

      CentroidPoint () = default;

      /** Add a new point to the centroid computation.
        *
        * In this function only the accumulators and point counter are updated,
        * actual centroid computation does not happen until get() is called. */
      void
      add (const PointT& point);

      /** Retrieve the current centroid.
        *
        * Computation (division of accumulated values by the number of points
        * and normalization where applicable) happens here. The result is not
        * cached, so any subsequent call to this function will trigger
        * re-computation.
        *
        * If the number of accumulated points is zero, then the point will be
        * left untouched. */
      template <typename PointOutT> void
      get (PointOutT& point) const;

      /** Get the total number of points that were added. */
      inline std::size_t
      getSize () const
      {
        return (num_points_);
      }

      PCL_MAKE_ALIGNED_OPERATOR_NEW

    private:

      std::size_t num_points_ = 0;
      typename pcl::detail::Accumulators<PointT>::type accumulators_;

  };

  /** Compute the centroid of a set of points and return it as a point.
    *
    * Implementation leverages \ref CentroidPoint class and therefore behaves
    * differently from \ref compute3DCentroid() and \ref computeNDCentroid().
    * See \ref CentroidPoint documentation for explanation.
    *
    * \param[in] cloud input point cloud
    * \param[out] centroid output centroid
    *
    * \return number of valid points used to determine the centroid (will be the
    * same as the size of the cloud if it is dense)
    *
    * \note If return value is \c 0, then the centroid is not changed, thus is
    * not valid.
    *
    * \ingroup common */
  template <typename PointInT, typename PointOutT> std::size_t
  computeCentroid (const pcl::PointCloud<PointInT>& cloud,
                   PointOutT& centroid);

  /** Compute the centroid of a set of points and return it as a point.
    * \param[in] cloud
    * \param[in] indices point cloud indices that need to be used
    * \param[out] centroid
    * This is an overloaded function provided for convenience. See the
    * documentation for computeCentroid().
    *
    * \ingroup common */
  template <typename PointInT, typename PointOutT> std::size_t
  computeCentroid (const pcl::PointCloud<PointInT>& cloud,
                   const Indices& indices,
                   PointOutT& centroid);

}
/*@}*/
#include <pcl/common/impl/centroid.hpp>
