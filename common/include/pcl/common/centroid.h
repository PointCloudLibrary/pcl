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
 */

#ifndef PCL_COMMON_CENTROID_H_
#define PCL_COMMON_CENTROID_H_

#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>

/**
  * \file pcl/common/centroid.h
  * Define methods for centroid estimation and covariance matrix calculus
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  /**
    * \brief Compute the 3D (X-Y-Z) centroid of a set of points and return it as a 3D vector.
    * \param[in] cloud the input point cloud
    * \param[out] centroid the output centroid
    * \return number of valid point used to determine the centroid. In case of dense point clouds, this is the same as the size of input cloud.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid);

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and
    * return it as a 3D vector.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[out] centroid the output centroid
    * \return number of valid point used to determine the centroid. In case of dense point clouds, this is the same as the size of input cloud.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const std::vector<int> &indices, Eigen::Vector4f &centroid);

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and
    * return it as a 3D vector.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[out] centroid the output centroid
    * \return number of valid point used to determine the centroid. In case of dense point clouds, this is the same as the size of input cloud.
    * \note if return value is 0, the centroid is not changed, thus not valid.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                     const pcl::PointIndices &indices, Eigen::Vector4f &centroid);

   /** \brief Compute the 3x3 covariance matrix of a given set of points.
     * The result is returned as a Eigen::Matrix3f.
     * Note: the covariance matrix is not normalized with the number of
     * points. For a normalized covariance, please use
     * computeNormalizedCovarianceMatrix.
     * \param[in] cloud the input point cloud
     * \param[in] centroid the centroid of the set of points in the cloud
     * \param[out] covariance_matrix the resultant 3x3 covariance matrix
     * \return number of valid point used to determine the covariance matrix.
     * In case of dense point clouds, this is the same as the size of input cloud.
     * \note if return value is 0, the covariance matrix is not changed, thus not valid.
     * \ingroup common
     */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const Eigen::Vector4f &centroid,
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute normalized the 3x3 covariance matrix of a given set of points.
    * The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of points in the point cloud.
    * For small number of points, or if you want explicitely the sample-variance, use computeCovarianceMatrix
    * and scale the covariance matrix with 1 / (n-1), where n is the number of points used to calculate
    * the covariance matrix and is returned by the computeCovarianceMatrix function.
    * \param[in] cloud the input point cloud
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const Eigen::Vector4f &centroid,
                                     Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeNormalizedCovarianceMatrix.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const std::vector<int> &indices,
                           const Eigen::Vector4f &centroid,
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeNormalizedCovarianceMatrix.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           const Eigen::Vector4f &centroid,
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix of a given set of points using
    * their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, use computeCovarianceMatrix
    * and scale the covariance matrix with 1 / (n-1), where n is the number of points used to calculate
    * the covariance matrix and is returned by the computeCovarianceMatrix function.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const std::vector<int> &indices,
                                     const Eigen::Vector4f &centroid,
                                     Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix of a given set of points using
    * their indices. The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, use computeCovarianceMatrix
    * and scale the covariance matrix with 1 / (n-1), where n is the number of points used to calculate
    * the covariance matrix and is returned by the computeCovarianceMatrix function.
    * \param[in] cloud the input point cloud
    * \param[in] indices the point cloud indices that need to be used
    * \param[in] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                     const pcl::PointIndices &indices,
                                     const Eigen::Vector4f &centroid,
                                     Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \param[out] centroid the centroid of the set of points in the cloud
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  Eigen::Matrix3f &covariance_matrix,
                                  Eigen::Vector4f &centroid);

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \param[out] centroid the centroid of the set of points in the cloud
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const std::vector<int> &indices,
                                  Eigen::Matrix3f &covariance_matrix,
                                  Eigen::Vector4f &centroid);

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const pcl::PointIndices &indices,
                                  Eigen::Matrix3f &covariance_matrix,
                                  Eigen::Vector4f &centroid);

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \param[in] cloud the input point cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \param[out] centroid the centroid of the set of points in the cloud
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  Eigen::Matrix3d &covariance_matrix,
                                  Eigen::Vector4d &centroid);

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \param[out] centroid the centroid of the set of points in the cloud
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const std::vector<int> &indices,
                                  Eigen::Matrix3d &covariance_matrix,
                                  Eigen::Vector4d &centroid);

  /** \brief Compute the normalized 3x3 covariance matrix and the centroid of a given set of points in a single loop.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] centroid the centroid of the set of points in the cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const pcl::PointIndices &indices,
                                  Eigen::Matrix3d &covariance_matrix,
                                  Eigen::Vector4d &centroid);

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                          const std::vector<int> &indices,
                          Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \note This method is theoretically exact. However using float for internal calculations reduces the accuracy but increases the efficiency.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \param[in] cloud the input point cloud
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           Eigen::Matrix3d &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const std::vector<int> &indices,
                           Eigen::Matrix3d &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix for a already demeaned point cloud.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * For small number of points, or if you want explicitely the sample-variance, scale the covariance matrix
    * with n / (n-1), where n is the number of points used to calculate the covariance matrix and is returned by this function.
    * \param[in] cloud the input point cloud
    * \param[in] indices subset of points given by their indices
    * \param[out] covariance_matrix the resultant 3x3 covariance matrix
    * \return number of valid point used to determine the covariance matrix.
    * In case of dense point clouds, this is the same as the size of input cloud.
    * \ingroup common
    */
  template <typename PointT> inline unsigned int
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                           const pcl::PointIndices &indices,
                           Eigen::Matrix3d &covariance_matrix);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param cloud_in the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output point cloud
    * \ingroup common
    */
  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Eigen::Vector4f &centroid,
                    pcl::PointCloud<PointT> &cloud_out);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned representation
    * \param cloud_in the input point cloud
    * \param indices the set of point indices to use from the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output point cloud
    * \ingroup common
    */
  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const std::vector<int> &indices,
                    const Eigen::Vector4f &centroid,
                    pcl::PointCloud<PointT> &cloud_out);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param cloud_in the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    * \ingroup common
    */
  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const Eigen::Vector4f &centroid,
                    Eigen::MatrixXf &cloud_out);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param cloud_in the input point cloud
    * \param indices the set of point indices to use from the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    * \ingroup common
    */
  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const std::vector<int> &indices,
                    const Eigen::Vector4f &centroid,
                    Eigen::MatrixXf &cloud_out);

  /** \brief Subtract a centroid from a point cloud and return the de-meaned
    * representation as an Eigen matrix
    * \param cloud_in the input point cloud
    * \param indices the set of point indices to use from the input point cloud
    * \param centroid the centroid of the point cloud
    * \param cloud_out the resultant output XYZ0 dimensions of \a cloud_in as
    * an Eigen matrix (4 rows, N pts columns)
    * \ingroup common
    */
  template <typename PointT> void
  demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                    const pcl::PointIndices& indices,
                    const Eigen::Vector4f &centroid,
                    Eigen::MatrixXf &cloud_out);

  /** \brief Helper functor structure for n-D centroid estimation. */
  template<typename PointT>
  struct NdCentroidFunctor
  {
    typedef typename traits::POD<PointT>::type Pod;

    NdCentroidFunctor (const PointT &p, Eigen::VectorXf &centroid)
      : f_idx_ (0),
        centroid_ (centroid),
        p_ (reinterpret_cast<const Pod&>(p)) { }

    template<typename Key> inline void operator() ()
    {
      typedef typename pcl::traits::datatype<PointT, Key>::type T;
      const uint8_t* raw_ptr = reinterpret_cast<const uint8_t*>(&p_) + pcl::traits::offset<PointT, Key>::value;
      const T* data_ptr = reinterpret_cast<const T*>(raw_ptr);

      // Check if the value is invalid
      if (!pcl_isfinite (*data_ptr))
      {
        f_idx_++;
        return;
      }

      centroid_[f_idx_++] += *data_ptr;
    }

    private:
      int f_idx_;
      Eigen::VectorXf &centroid_;
      const Pod &p_;
  };

  /** \brief General, all purpose nD centroid estimation for a set of points using their
    * indices.
    * \param cloud the input point cloud
    * \param centroid the output centroid
    * \ingroup common
    */
  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::VectorXf &centroid);

  /** \brief General, all purpose nD centroid estimation for a set of points using their
    * indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    * \ingroup common
    */
  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                     const std::vector<int> &indices, Eigen::VectorXf &centroid);

  /** \brief General, all purpose nD centroid estimation for a set of points using their
    * indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    * \ingroup common
    */
  template <typename PointT> inline void
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                     const pcl::PointIndices &indices, Eigen::VectorXf &centroid);

}
/*@}*/
#include <pcl/common/impl/centroid.hpp>

#endif  //#ifndef PCL_COMMON_CENTROID_H_
