/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-present, Willow Garage, Inc.
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

#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite

#include <boost/fusion/algorithm/transformation/filter_if.hpp> // for boost::fusion::filter_if
#include <boost/fusion/algorithm/iteration/for_each.hpp> // for boost::fusion::for_each
#include <boost/mpl/size.hpp> // for boost::mpl::size


namespace pcl
{

template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (ConstCloudIterator<PointT> &cloud_iterator,
                   Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  Eigen::Matrix<Scalar, 4, 1> accumulator {0, 0, 0, 0};

  unsigned int cp = 0;

  // For each point in the cloud
  // If the data is dense, we don't need to check for NaN
  while (cloud_iterator.isValid ())
  {
    // Check if the point is invalid
    if (pcl::isFinite (*cloud_iterator))
    {
      accumulator[0] += cloud_iterator->x;
      accumulator[1] += cloud_iterator->y;
      accumulator[2] += cloud_iterator->z;
      ++cp;
    }
    ++cloud_iterator;
  }

  if (cp > 0) {
    centroid = accumulator;
    centroid /= static_cast<Scalar> (cp);
    centroid[3] = 1;
  }
  return (cp);
}


template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  if (cloud.empty ())
    return (0);

  // For each point in the cloud
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    // Initialize to 0
    centroid.setZero ();
    for (const auto& point: cloud)
    {
      centroid[0] += point.x;
      centroid[1] += point.y;
      centroid[2] += point.z;
    }
    centroid /= static_cast<Scalar> (cloud.size ());
    centroid[3] = 1;

    return (static_cast<unsigned int> (cloud.size ()));
  }
  // NaN or Inf values could exist => check for them
  unsigned int cp = 0;
  Eigen::Matrix<Scalar, 4, 1> accumulator {0, 0, 0, 0};
  for (const auto& point: cloud)
  {
    // Check if the point is invalid
    if (!isFinite (point))
      continue;

    accumulator[0] += point.x;
    accumulator[1] += point.y;
    accumulator[2] += point.z;
    ++cp;
  }
  if (cp > 0) {
    centroid = accumulator;
    centroid /= static_cast<Scalar> (cp);
    centroid[3] = 1;
  }

  return (cp);
}


template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const Indices &indices,
                   Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  if (indices.empty ())
    return (0);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    // Initialize to 0
    centroid.setZero ();
    for (const auto& index : indices)
    {
      centroid[0] += cloud[index].x;
      centroid[1] += cloud[index].y;
      centroid[2] += cloud[index].z;
    }
    centroid /= static_cast<Scalar> (indices.size ());
    centroid[3] = 1;
    return (static_cast<unsigned int> (indices.size ()));
  }
  // NaN or Inf values could exist => check for them
  Eigen::Matrix<Scalar, 4, 1> accumulator {0, 0, 0, 0};
  unsigned int cp = 0;
  for (const auto& index : indices)
  {
    // Check if the point is invalid
    if (!isFinite (cloud [index]))
      continue;

    accumulator[0] += cloud[index].x;
    accumulator[1] += cloud[index].y;
    accumulator[2] += cloud[index].z;
    ++cp;
  }
  if (cp > 0) {
    centroid = accumulator;
    centroid /= static_cast<Scalar> (cp);
    centroid[3] = 1;
  }
  return (cp);
}


template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const pcl::PointIndices &indices,
                   Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  return (pcl::compute3DCentroid (cloud, indices.indices, centroid));
}


template <typename PointT, typename Scalar> inline unsigned
computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                         const Eigen::Matrix<Scalar, 4, 1> &centroid,
                         Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  if (cloud.empty ())
    return (0);

  unsigned point_count;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    covariance_matrix.setZero ();
    point_count = static_cast<unsigned> (cloud.size ());
    // For each point in the cloud
    for (const auto& point: cloud)
    {
      Eigen::Matrix<Scalar, 4, 1> pt;
      pt[0] = point.x - centroid[0];
      pt[1] = point.y - centroid[1];
      pt[2] = point.z - centroid[2];

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    Eigen::Matrix<Scalar, 3, 3> temp_covariance_matrix;
    temp_covariance_matrix.setZero();
    point_count = 0;
    // For each point in the cloud
    for (const auto& point: cloud)
    {
      // Check if the point is invalid
      if (!isFinite (point))
        continue;

      Eigen::Matrix<Scalar, 4, 1> pt;
      pt[0] = point.x - centroid[0];
      pt[1] = point.y - centroid[1];
      pt[2] = point.z - centroid[2];

      temp_covariance_matrix (1, 1) += pt.y () * pt.y ();
      temp_covariance_matrix (1, 2) += pt.y () * pt.z ();

      temp_covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      temp_covariance_matrix (0, 0) += pt.x ();
      temp_covariance_matrix (0, 1) += pt.y ();
      temp_covariance_matrix (0, 2) += pt.z ();
      ++point_count;
    }
    if (point_count > 0) {
      covariance_matrix = temp_covariance_matrix;
    }
  }
  if (point_count == 0) { 
    return 0; 
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);

  return (point_count);
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                   const Eigen::Matrix<Scalar, 4, 1> &centroid,
                                   Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  unsigned point_count = pcl::computeCovarianceMatrix (cloud, centroid, covariance_matrix);
  if (point_count != 0)
    covariance_matrix /= static_cast<Scalar> (point_count);
  return (point_count);
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                         const Indices &indices,
                         const Eigen::Matrix<Scalar, 4, 1> &centroid,
                         Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  if (indices.empty ())
    return (0);

  std::size_t point_count;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    covariance_matrix.setZero ();
    point_count = indices.size ();
    // For each point in the cloud
    for (const auto& idx: indices)
    {
      Eigen::Matrix<Scalar, 4, 1> pt;
      pt[0] = cloud[idx].x - centroid[0];
      pt[1] = cloud[idx].y - centroid[1];
      pt[2] = cloud[idx].z - centroid[2];

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    Eigen::Matrix<Scalar, 3, 3> temp_covariance_matrix;
    temp_covariance_matrix.setZero ();
    point_count = 0;
    // For each point in the cloud
    for (const auto &index : indices)
    {
      // Check if the point is invalid
      if (!isFinite (cloud[index]))
        continue;

      Eigen::Matrix<Scalar, 4, 1> pt;
      pt[0] = cloud[index].x - centroid[0];
      pt[1] = cloud[index].y - centroid[1];
      pt[2] = cloud[index].z - centroid[2];

      temp_covariance_matrix (1, 1) += pt.y () * pt.y ();
      temp_covariance_matrix (1, 2) += pt.y () * pt.z ();

      temp_covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      temp_covariance_matrix (0, 0) += pt.x ();
      temp_covariance_matrix (0, 1) += pt.y ();
      temp_covariance_matrix (0, 2) += pt.z ();
      ++point_count;
    }
    if (point_count > 0) {
      covariance_matrix = temp_covariance_matrix;
    }
  }
  if (point_count == 0) { 
    return 0; 
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);
  return (static_cast<unsigned int> (point_count));
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                         const pcl::PointIndices &indices,
                         const Eigen::Matrix<Scalar, 4, 1> &centroid,
                         Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  return (pcl::computeCovarianceMatrix (cloud, indices.indices, centroid, covariance_matrix));
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                   const Indices &indices,
                                   const Eigen::Matrix<Scalar, 4, 1> &centroid,
                                   Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  unsigned point_count = pcl::computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix);
  if (point_count != 0)
    covariance_matrix /= static_cast<Scalar> (point_count);

  return (point_count);
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                   const pcl::PointIndices &indices,
                                   const Eigen::Matrix<Scalar, 4, 1> &centroid,
                                   Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  return computeCovarianceMatrixNormalized(cloud, indices.indices, centroid, covariance_matrix);
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                         Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
  Eigen::Matrix<Scalar, 1, 6, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 6, Eigen::RowMajor>::Zero ();

  unsigned int point_count;
  if (cloud.is_dense)
  {
    point_count = static_cast<unsigned int> (cloud.size ());
    // For each point in the cloud
    for (const auto& point: cloud)
    {
      accu [0] += point.x * point.x;
      accu [1] += point.x * point.y;
      accu [2] += point.x * point.z;
      accu [3] += point.y * point.y;
      accu [4] += point.y * point.z;
      accu [5] += point.z * point.z;
    }
  }
  else
  {
    point_count = 0;
    for (const auto& point: cloud)
    {
      if (!isFinite (point))
        continue;

      accu [0] += point.x * point.x;
      accu [1] += point.x * point.y;
      accu [2] += point.x * point.z;
      accu [3] += point.y * point.y;
      accu [4] += point.y * point.z;
      accu [5] += point.z * point.z;
      ++point_count;
    }
  }

  if (point_count != 0)
  {
    accu /= static_cast<Scalar> (point_count);
    covariance_matrix.coeffRef (0) = accu [0];
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = accu [1];
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = accu [2];
    covariance_matrix.coeffRef (4) = accu [3];
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = accu [4];
    covariance_matrix.coeffRef (8) = accu [5];
  }
  return (point_count);
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                         const Indices &indices,
                         Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
  Eigen::Matrix<Scalar, 1, 6, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 6, Eigen::RowMajor>::Zero ();

  unsigned int point_count;
  if (cloud.is_dense)
  {
    point_count = static_cast<unsigned int> (indices.size ());
    for (const auto &index : indices)
    {
      //const PointT& point = cloud[*iIt];
      accu [0] += cloud[index].x * cloud[index].x;
      accu [1] += cloud[index].x * cloud[index].y;
      accu [2] += cloud[index].x * cloud[index].z;
      accu [3] += cloud[index].y * cloud[index].y;
      accu [4] += cloud[index].y * cloud[index].z;
      accu [5] += cloud[index].z * cloud[index].z;
    }
  }
  else
  {
    point_count = 0;
    for (const auto &index : indices)
    {
      if (!isFinite (cloud[index]))
        continue;

      ++point_count;
      accu [0] += cloud[index].x * cloud[index].x;
      accu [1] += cloud[index].x * cloud[index].y;
      accu [2] += cloud[index].x * cloud[index].z;
      accu [3] += cloud[index].y * cloud[index].y;
      accu [4] += cloud[index].y * cloud[index].z;
      accu [5] += cloud[index].z * cloud[index].z;
    }
  }
  if (point_count != 0)
  {
    accu /= static_cast<Scalar> (point_count);
    covariance_matrix.coeffRef (0) = accu [0];
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = accu [1];
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = accu [2];
    covariance_matrix.coeffRef (4) = accu [3];
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = accu [4];
    covariance_matrix.coeffRef (8) = accu [5];
  }
  return (point_count);
}


template <typename PointT, typename Scalar> inline unsigned int
computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                         const pcl::PointIndices &indices,
                         Eigen::Matrix<Scalar, 3, 3> &covariance_matrix)
{
  return (computeCovarianceMatrix (cloud, indices.indices, covariance_matrix));
}


template <typename PointT, typename Scalar> inline unsigned int
computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                Eigen::Matrix<Scalar, 3, 3> &covariance_matrix,
                                Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  // Shifted data/with estimate of mean. This gives very good accuracy and good performance.
  // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
  Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 3, 1> K(0.0, 0.0, 0.0);
  for(const auto& point: cloud)
    if(isFinite(point)) {
      K.x() = point.x; K.y() = point.y; K.z() = point.z; break;
    }
  std::size_t point_count;
  if (cloud.is_dense)
  {
    point_count = cloud.size ();
    // For each point in the cloud
    for (const auto& point: cloud)
    {
      Scalar x = point.x - K.x(), y = point.y - K.y(), z = point.z - K.z();
      accu [0] += x * x;
      accu [1] += x * y;
      accu [2] += x * z;
      accu [3] += y * y;
      accu [4] += y * z;
      accu [5] += z * z;
      accu [6] += x;
      accu [7] += y;
      accu [8] += z;
    }
  }
  else
  {
    point_count = 0;
    for (const auto& point: cloud)
    {
      if (!isFinite (point))
        continue;

      Scalar x = point.x - K.x(), y = point.y - K.y(), z = point.z - K.z();
      accu [0] += x * x;
      accu [1] += x * y;
      accu [2] += x * z;
      accu [3] += y * y;
      accu [4] += y * z;
      accu [5] += z * z;
      accu [6] += x;
      accu [7] += y;
      accu [8] += z;
      ++point_count;
    }
  }
  if (point_count != 0)
  {
    accu /= static_cast<Scalar> (point_count);
    centroid[0] = accu[6] + K.x(); centroid[1] = accu[7] + K.y(); centroid[2] = accu[8] + K.z();
    centroid[3] = 1;
    covariance_matrix.coeffRef (0) = accu [0] - accu [6] * accu [6];
    covariance_matrix.coeffRef (1) = accu [1] - accu [6] * accu [7];
    covariance_matrix.coeffRef (2) = accu [2] - accu [6] * accu [8];
    covariance_matrix.coeffRef (4) = accu [3] - accu [7] * accu [7];
    covariance_matrix.coeffRef (5) = accu [4] - accu [7] * accu [8];
    covariance_matrix.coeffRef (8) = accu [5] - accu [8] * accu [8];
    covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
    covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
    covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);
  }
  return (static_cast<unsigned int> (point_count));
}


template <typename PointT, typename Scalar> inline unsigned int
computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                const Indices &indices,
                                Eigen::Matrix<Scalar, 3, 3> &covariance_matrix,
                                Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  // Shifted data/with estimate of mean. This gives very good accuracy and good performance.
  // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
  Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 3, 1> K(0.0, 0.0, 0.0);
  for(const auto& index : indices)
    if(isFinite(cloud[index])) {
      K.x() = cloud[index].x; K.y() = cloud[index].y; K.z() = cloud[index].z; break;
    }
  std::size_t point_count;
  if (cloud.is_dense)
  {
    point_count = indices.size ();
    for (const auto &index : indices)
    {
      Scalar x = cloud[index].x - K.x(), y = cloud[index].y - K.y(), z = cloud[index].z - K.z();
      accu [0] += x * x;
      accu [1] += x * y;
      accu [2] += x * z;
      accu [3] += y * y;
      accu [4] += y * z;
      accu [5] += z * z;
      accu [6] += x;
      accu [7] += y;
      accu [8] += z;
    }
  }
  else
  {
    point_count = 0;
    for (const auto &index : indices)
    {
      if (!isFinite (cloud[index]))
        continue;

      ++point_count;
      Scalar x = cloud[index].x - K.x(), y = cloud[index].y - K.y(), z = cloud[index].z - K.z();
      accu [0] += x * x;
      accu [1] += x * y;
      accu [2] += x * z;
      accu [3] += y * y;
      accu [4] += y * z;
      accu [5] += z * z;
      accu [6] += x;
      accu [7] += y;
      accu [8] += z;
    }
  }

  if (point_count != 0)
  {
    accu /= static_cast<Scalar> (point_count);
    centroid[0] = accu[6] + K.x(); centroid[1] = accu[7] + K.y(); centroid[2] = accu[8] + K.z();
    centroid[3] = 1;
    covariance_matrix.coeffRef (0) = accu [0] - accu [6] * accu [6];
    covariance_matrix.coeffRef (1) = accu [1] - accu [6] * accu [7];
    covariance_matrix.coeffRef (2) = accu [2] - accu [6] * accu [8];
    covariance_matrix.coeffRef (4) = accu [3] - accu [7] * accu [7];
    covariance_matrix.coeffRef (5) = accu [4] - accu [7] * accu [8];
    covariance_matrix.coeffRef (8) = accu [5] - accu [8] * accu [8];
    covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
    covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
    covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);
  }
  return (static_cast<unsigned int> (point_count));
}


template <typename PointT, typename Scalar> inline unsigned int
computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                const pcl::PointIndices &indices,
                                Eigen::Matrix<Scalar, 3, 3> &covariance_matrix,
                                Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  return (computeMeanAndCovarianceMatrix (cloud, indices.indices, covariance_matrix, centroid));
}


template <typename PointT, typename Scalar> void
demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  pcl::PointCloud<PointT> &cloud_out,
                  int npts)
{
  // Calculate the number of points if not given
  if (npts == 0)
  {
    while (cloud_iterator.isValid ())
    {
      ++npts;
      ++cloud_iterator;
    }
    cloud_iterator.reset ();
  }

  int i = 0;
  cloud_out.resize (npts);
  // Subtract the centroid from cloud_in
  while (cloud_iterator.isValid ())
  {
    cloud_out[i].x = cloud_iterator->x - centroid[0];
    cloud_out[i].y = cloud_iterator->y - centroid[1];
    cloud_out[i].z = cloud_iterator->z - centroid[2];
    ++i;
    ++cloud_iterator;
  }
  cloud_out.width = cloud_out.size ();
  cloud_out.height = 1;
}


template <typename PointT, typename Scalar> void
demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  pcl::PointCloud<PointT> &cloud_out)
{
  cloud_out = cloud_in;

  // Subtract the centroid from cloud_in
  for (auto& point: cloud_out)
  {
    point.x -= static_cast<float> (centroid[0]);
    point.y -= static_cast<float> (centroid[1]);
    point.z -= static_cast<float> (centroid[2]);
  }
}


template <typename PointT, typename Scalar> void
demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const Indices &indices,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  pcl::PointCloud<PointT> &cloud_out)
{
  cloud_out.header = cloud_in.header;
  cloud_out.is_dense = cloud_in.is_dense;
  if (indices.size () == cloud_in.size ())
  {
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
  }
  else
  {
    cloud_out.width    = indices.size ();
    cloud_out.height   = 1;
  }
  cloud_out.resize (indices.size ());

  // Subtract the centroid from cloud_in
  for (std::size_t i = 0; i < indices.size (); ++i)
  {
    cloud_out[i].x = static_cast<float> (cloud_in[indices[i]].x - centroid[0]);
    cloud_out[i].y = static_cast<float> (cloud_in[indices[i]].y - centroid[1]);
    cloud_out[i].z = static_cast<float> (cloud_in[indices[i]].z - centroid[2]);
  }
}


template <typename PointT, typename Scalar> void
demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const pcl::PointIndices& indices,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  pcl::PointCloud<PointT> &cloud_out)
{
  return (demeanPointCloud (cloud_in, indices.indices, centroid, cloud_out));
}


template <typename PointT, typename Scalar> void
demeanPointCloud (ConstCloudIterator<PointT> &cloud_iterator,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out,
                  int npts)
{
  // Calculate the number of points if not given
  if (npts == 0)
  {
    while (cloud_iterator.isValid ())
    {
      ++npts;
      ++cloud_iterator;
    }
    cloud_iterator.reset ();
  }

  cloud_out = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Zero (4, npts);        // keep the data aligned

  int i = 0;
  while (cloud_iterator.isValid ())
  {
    cloud_out (0, i) = cloud_iterator->x - centroid[0];
    cloud_out (1, i) = cloud_iterator->y - centroid[1];
    cloud_out (2, i) = cloud_iterator->z - centroid[2];
    ++i;
    ++cloud_iterator;
  }
}


template <typename PointT, typename Scalar> void
demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out)
{
  std::size_t npts = cloud_in.size ();

  cloud_out = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Zero (4, npts);        // keep the data aligned

  for (std::size_t i = 0; i < npts; ++i)
  {
    cloud_out (0, i) = cloud_in[i].x - centroid[0];
    cloud_out (1, i) = cloud_in[i].y - centroid[1];
    cloud_out (2, i) = cloud_in[i].z - centroid[2];
    // One column at a time
    //cloud_out.block<4, 1> (0, i) = cloud_in[i].getVector4fMap () - centroid;
  }

  // Make sure we zero the 4th dimension out (1 row, N columns)
  //cloud_out.block (3, 0, 1, npts).setZero ();
}


template <typename PointT, typename Scalar> void
demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const Indices &indices,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out)
{
  std::size_t npts = indices.size ();

  cloud_out = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Zero (4, npts);        // keep the data aligned

  for (std::size_t i = 0; i < npts; ++i)
  {
    cloud_out (0, i) = cloud_in[indices[i]].x - centroid[0];
    cloud_out (1, i) = cloud_in[indices[i]].y - centroid[1];
    cloud_out (2, i) = cloud_in[indices[i]].z - centroid[2];
    // One column at a time
    //cloud_out.block<4, 1> (0, i) = cloud_in[indices[i]].getVector4fMap () - centroid;
  }

  // Make sure we zero the 4th dimension out (1 row, N columns)
  //cloud_out.block (3, 0, 1, npts).setZero ();
}


template <typename PointT, typename Scalar> void
demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const pcl::PointIndices &indices,
                  const Eigen::Matrix<Scalar, 4, 1> &centroid,
                  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_out)
{
  return (pcl::demeanPointCloud (cloud_in, indices.indices, centroid, cloud_out));
}


template <typename PointT, typename Scalar> inline void
computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                   Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid)
{
  using FieldList = typename pcl::traits::fieldList<PointT>::type;

  // Get the size of the fields
  centroid.setZero (boost::mpl::size<FieldList>::value);

  if (cloud.empty ())
    return;

  // Iterate over each point
  for (const auto& pt: cloud)
  {
    // Iterate over each dimension
    pcl::for_each_type<FieldList> (NdCentroidFunctor<PointT, Scalar> (pt, centroid));
  }
  centroid /= static_cast<Scalar> (cloud.size ());
}


template <typename PointT, typename Scalar> inline void
computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                   const Indices &indices,
                   Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid)
{
  using FieldList = typename pcl::traits::fieldList<PointT>::type;

  // Get the size of the fields
  centroid.setZero (boost::mpl::size<FieldList>::value);

  if (indices.empty ())
    return;

  // Iterate over each point
  for (const auto& index: indices)
  {
    // Iterate over each dimension
    pcl::for_each_type<FieldList> (NdCentroidFunctor<PointT, Scalar> (cloud[index], centroid));
  }
  centroid /= static_cast<Scalar> (indices.size ());
}


template <typename PointT, typename Scalar> inline void
computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                   const pcl::PointIndices &indices,
                   Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &centroid)
{
  return (pcl::computeNDCentroid (cloud, indices.indices, centroid));
}

template <typename PointT> void
CentroidPoint<PointT>::add (const PointT& point)
{
  // Invoke add point on each accumulator
  boost::fusion::for_each (accumulators_, detail::AddPoint<PointT> (point));
  ++num_points_;
}

template <typename PointT>
template <typename PointOutT> void
CentroidPoint<PointT>::get (PointOutT& point) const
{
  if (num_points_ != 0)
  {
    // Filter accumulators so that only those that are compatible with
    // both PointT and requested point type remain
    auto ca = boost::fusion::filter_if<detail::IsAccumulatorCompatible<PointT, PointOutT>> (accumulators_);
    // Invoke get point on each accumulator in filtered list
    boost::fusion::for_each (ca, detail::GetPoint<PointOutT> (point, num_points_));
  }
}


template <typename PointInT, typename PointOutT> std::size_t
computeCentroid (const pcl::PointCloud<PointInT>& cloud,
                      PointOutT& centroid)
{
  pcl::CentroidPoint<PointInT> cp;

  if (cloud.is_dense)
    for (const auto& point: cloud)
      cp.add (point);
  else
    for (const auto& point: cloud)
      if (pcl::isFinite (point))
        cp.add (point);

  cp.get (centroid);
  return (cp.getSize ());
}


template <typename PointInT, typename PointOutT> std::size_t
computeCentroid (const pcl::PointCloud<PointInT>& cloud,
                      const Indices& indices,
                      PointOutT& centroid)
{
  pcl::CentroidPoint<PointInT> cp;

  if (cloud.is_dense)
    for (const auto &index : indices)
      cp.add (cloud[index]);
  else
    for (const auto &index : indices)
      if (pcl::isFinite (cloud[index]))
        cp.add (cloud[index]);

  cp.get (centroid);
  return (cp.getSize ());
}

} // namespace pcl

