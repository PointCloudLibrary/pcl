/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009-present, Willow Garage, Inc.
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

#ifndef PCL_COMMON_IMPL_CENTROID_H_
#define PCL_COMMON_IMPL_CENTROID_H_

#include <pcl/ros/conversions.h>
#include <boost/mpl/size.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid)
{
  if (cloud.points.empty ())
    return (0);

  // Initialize to 0
  centroid.setZero ();
  // For each point in the cloud
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      centroid += cloud.points[i].getVector4fMap ();
    centroid[3] = 0;
    centroid /= static_cast<float> (cloud.points.size ());

    return (static_cast<unsigned int> (cloud.points.size ()));
  }
  // NaN or Inf values could exist => check for them
  else
  {
    unsigned cp = 0;
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!isFinite (cloud [i]))
        continue;

      centroid += cloud[i].getVector4fMap ();
      ++cp;
    }
    centroid[3] = 0;
    centroid /= static_cast<float> (cp);

    return (cp);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                        Eigen::Vector4f &centroid)
{
  if (indices.empty ())
    return (0);

  // Initialize to 0
  centroid.setZero ();
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < indices.size (); ++i)
      centroid += cloud.points[indices[i]].getVector4fMap ();
    centroid[3] = 0;
    centroid /= static_cast<float> (indices.size ());
    return (static_cast<unsigned int> (indices.size ()));
  }
  // NaN or Inf values could exist => check for them
  else
  {
    unsigned cp = 0;
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!isFinite (cloud [indices[i]]))
        continue;

      centroid += cloud[indices[i]].getVector4fMap ();
      ++cp;
    }
    centroid[3] = 0.0f;
    centroid /= static_cast<float> (cp);
    return (cp);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                        const pcl::PointIndices &indices, Eigen::Vector4f &centroid)
{
  return (pcl::compute3DCentroid<PointT> (cloud, indices.indices, centroid));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const Eigen::Vector4f &centroid,
                              Eigen::Matrix3f &covariance_matrix)
{
  if (cloud.points.empty ())
    return (0);

  // Initialize to 0
  covariance_matrix.setZero ();

  unsigned point_count;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    point_count = static_cast<unsigned> (cloud.size ());
    // For each point in the cloud
    for (size_t i = 0; i < point_count; ++i)
    {
      Eigen::Vector4f pt = cloud [i].getVector4fMap () - centroid;

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
    point_count = 0;
    // For each point in the cloud
    for (size_t i = 0; i < cloud.size (); ++i)
    {
      // Check if the point is invalid
      if (!isFinite (cloud [i]))
        continue;

      Eigen::Vector4f pt = cloud [i].getVector4fMap () - centroid;

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
      ++point_count;
    }
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);

  return (point_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                        const Eigen::Vector4f &centroid,
                                        Eigen::Matrix3f &covariance_matrix)
{
  unsigned point_count = pcl::computeCovarianceMatrix (cloud, centroid, covariance_matrix);
  if (point_count != 0)
    covariance_matrix /= static_cast<float> (point_count);
  return (point_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const std::vector<int> &indices,
                              const Eigen::Vector4f &centroid,
                              Eigen::Matrix3f &covariance_matrix)
{
  if (indices.empty ())
    return (0);

  // Initialize to 0
  covariance_matrix.setZero ();

  size_t point_count;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    point_count = indices.size ();
    // For each point in the cloud
    for (size_t i = 0; i < point_count; ++i)
    {
      Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - centroid;

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
    point_count = 0;
    // For each point in the cloud
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!isFinite (cloud[indices[i]]))
        continue;

      Eigen::Vector4f pt = cloud[indices[i]].getVector4fMap () - centroid;

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
      ++point_count;
    }
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);
  return (static_cast<unsigned int> (point_count));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const pcl::PointIndices &indices,
                              const Eigen::Vector4f &centroid,
                              Eigen::Matrix3f &covariance_matrix)
{
  return (pcl::computeCovarianceMatrix<PointT> (cloud, indices.indices, centroid, covariance_matrix));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                        const std::vector<int> &indices,
                                        const Eigen::Vector4f &centroid,
                                        Eigen::Matrix3f &covariance_matrix)
{
  unsigned point_count = pcl::computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix);
  if (point_count != 0)
    covariance_matrix /= static_cast<float> (point_count);

  return (point_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                        const pcl::PointIndices &indices,
                                        const Eigen::Vector4f &centroid,
                                        Eigen::Matrix3f &covariance_matrix)
{
  unsigned int point_count = pcl::computeCovarianceMatrix (cloud, indices.indices, centroid, covariance_matrix);
  if (point_count != 0)
    covariance_matrix /= static_cast<int>(point_count);

  return point_count;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              Eigen::Matrix3f &covariance_matrix)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<float, 1, 6, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 6, Eigen::RowMajor>::Zero ();

  unsigned int point_count;
  if (cloud.is_dense)
  {
    point_count = static_cast<unsigned int> (cloud.size ());
    // For each point in the cloud
    for (size_t i = 0; i < point_count; ++i)
    {
      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y;
      accu [4] += cloud[i].y * cloud[i].z;
      accu [5] += cloud[i].z * cloud[i].z;
    }
  }
  else
  {
    point_count = 0;
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (!isFinite (cloud[i]))
        continue;

      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y;
      accu [4] += cloud[i].y * cloud[i].z;
      accu [5] += cloud[i].z * cloud[i].z;
      ++point_count;
    }
  }

  if (point_count != 0)
  {
    accu /= static_cast<float> (point_count);
    covariance_matrix.coeffRef (0) = accu [0];
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = accu [1];
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = accu [2];
    covariance_matrix.coeffRef (4) = accu [3];
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = accu [4];
    covariance_matrix.coeffRef (8) = accu [5];
  }
  return (point_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const std::vector<int> &indices,
                              Eigen::Matrix3f &covariance_matrix)
{
  Eigen::Matrix<float, 1, 6, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 6, Eigen::RowMajor>::Zero ();

  unsigned int point_count;
  if (cloud.is_dense)
  {
    point_count = static_cast<unsigned int> (indices.size ());
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      //const PointT& point = cloud[*iIt];
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y;
      accu [4] += cloud[*iIt].y * cloud[*iIt].z;
      accu [5] += cloud[*iIt].z * cloud[*iIt].z;
    }
  }
  else
  {
    point_count = 0;
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      if (!isFinite (cloud[*iIt]))
        continue;

      ++point_count;
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y;
      accu [4] += cloud[*iIt].y * cloud[*iIt].z;
      accu [5] += cloud[*iIt].z * cloud[*iIt].z;
    }
  }
  if (point_count != 0)
  {
    accu /= static_cast<float> (point_count);
    covariance_matrix.coeffRef (0) = accu [0];
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = accu [1];
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = accu [2];
    covariance_matrix.coeffRef (4) = accu [3];
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = accu [4];
    covariance_matrix.coeffRef (8) = accu [5];
  }
  return (point_count);
}

template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const pcl::PointIndices &indices,
                              Eigen::Matrix3f &covariance_matrix)
{
  return computeCovarianceMatrix (cloud, indices.indices, covariance_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              Eigen::Matrix3d &covariance_matrix)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<double, 1, 6, Eigen::RowMajor> accu = Eigen::Matrix<double, 1, 6, Eigen::RowMajor>::Zero ();

  unsigned int point_count;
  if (cloud.is_dense)
  {
    point_count = cloud.size ();
    // For each point in the cloud
    for (size_t i = 0; i < point_count; ++i)
    {
      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y;
      accu [4] += cloud[i].y * cloud[i].z;
      accu [5] += cloud[i].z * cloud[i].z;
    }
  }
  else
  {
    point_count = 0;
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (!isFinite (cloud[i]))
        continue;

      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y;
      accu [4] += cloud[i].y * cloud[i].z;
      accu [5] += cloud[i].z * cloud[i].z;
      ++point_count;
    }
  }

  if (point_count != 0)
  {
    accu /= static_cast<double> (point_count);
    covariance_matrix.coeffRef (0) = accu [0];
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = accu [1];
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = accu [2];
    covariance_matrix.coeffRef (4) = accu [3];
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = accu [4];
    covariance_matrix.coeffRef (8) = accu [5];
  }
  return (point_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const std::vector<int> &indices,
                              Eigen::Matrix3d &covariance_matrix)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<double, 1, 6, Eigen::RowMajor> accu = Eigen::Matrix<double, 1, 6, Eigen::RowMajor>::Zero ();

  unsigned int point_count;
  if (cloud.is_dense)
  {
    point_count = indices.size ();
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      //const PointT& point = cloud[*iIt];
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y;
      accu [4] += cloud[*iIt].y * cloud[*iIt].z;
      accu [5] += cloud[*iIt].z * cloud[*iIt].z;
    }
  }
  else
  {
    point_count = 0;
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      if (!isFinite (cloud[*iIt]))
        continue;

      ++point_count;
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y;
      accu [4] += cloud[*iIt].y * cloud[*iIt].z;
      accu [5] += cloud[*iIt].z * cloud[*iIt].z;
    }
  }
  if (point_count != 0)
  {
    accu /= static_cast<double> (point_count);
    covariance_matrix.coeffRef (0) = accu [0];
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = accu [1];
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = accu [2];
    covariance_matrix.coeffRef (4) = accu [3];
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = accu [4];
    covariance_matrix.coeffRef (8) = accu [5];
  }
  return (point_count);
}

template <typename PointT> inline unsigned int
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const pcl::PointIndices &indices,
                              Eigen::Matrix3d &covariance_matrix)
{
  return computeCovarianceMatrix (cloud, indices.indices, covariance_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                     Eigen::Matrix3f &covariance_matrix,
                                     Eigen::Vector4f &centroid)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero ();
  size_t point_count;
  if (cloud.is_dense)
  {
    point_count = cloud.size ();
    // For each point in the cloud
    for (size_t i = 0; i < point_count; ++i)
    {
      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y; // 4
      accu [4] += cloud[i].y * cloud[i].z; // 5
      accu [5] += cloud[i].z * cloud[i].z; // 8
      accu [6] += cloud[i].x;
      accu [7] += cloud[i].y;
      accu [8] += cloud[i].z;
    }
  }
  else
  {
    point_count = 0;
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (!isFinite (cloud[i]))
        continue;

      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y;
      accu [4] += cloud[i].y * cloud[i].z;
      accu [5] += cloud[i].z * cloud[i].z;
      accu [6] += cloud[i].x;
      accu [7] += cloud[i].y;
      accu [8] += cloud[i].z;
      ++point_count;
    }
  }
  accu /= static_cast<float> (point_count);
  if (point_count != 0)
  {
    //centroid.head<3> () = accu.tail<3> ();    -- does not compile with Clang 3.0
    centroid[0] = accu[6]; centroid[1] = accu[7]; centroid[2] = accu[8];
    centroid[3] = 0;
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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                const std::vector<int> &indices,
                                Eigen::Matrix3f &covariance_matrix,
                                Eigen::Vector4f &centroid)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero ();
  size_t point_count;
  if (cloud.is_dense)
  {
    point_count = indices.size ();
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      //const PointT& point = cloud[*iIt];
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y;
      accu [4] += cloud[*iIt].y * cloud[*iIt].z;
      accu [5] += cloud[*iIt].z * cloud[*iIt].z;
      accu [6] += cloud[*iIt].x;
      accu [7] += cloud[*iIt].y;
      accu [8] += cloud[*iIt].z;
    }
  }
  else
  {
    point_count = 0;
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      if (!isFinite (cloud[*iIt]))
        continue;

      ++point_count;
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y; // 4
      accu [4] += cloud[*iIt].y * cloud[*iIt].z; // 5
      accu [5] += cloud[*iIt].z * cloud[*iIt].z; // 8
      accu [6] += cloud[*iIt].x;
      accu [7] += cloud[*iIt].y;
      accu [8] += cloud[*iIt].z;
    }
  }

  accu /= static_cast<float> (point_count);
  //Eigen::Vector3f vec = accu.tail<3> ();
  //centroid.head<3> () = vec;//= accu.tail<3> ();
  //centroid.head<3> () = accu.tail<3> ();    -- does not compile with Clang 3.0
  centroid[0] = accu[6]; centroid[1] = accu[7]; centroid[2] = accu[8];
  centroid[3] = 0;
  covariance_matrix.coeffRef (0) = accu [0] - accu [6] * accu [6];
  covariance_matrix.coeffRef (1) = accu [1] - accu [6] * accu [7];
  covariance_matrix.coeffRef (2) = accu [2] - accu [6] * accu [8];
  covariance_matrix.coeffRef (4) = accu [3] - accu [7] * accu [7];
  covariance_matrix.coeffRef (5) = accu [4] - accu [7] * accu [8];
  covariance_matrix.coeffRef (8) = accu [5] - accu [8] * accu [8];
  covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
  covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
  covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);

  return (static_cast<unsigned int> (point_count));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                const pcl::PointIndices &indices,
                                Eigen::Matrix3f &covariance_matrix,
                                Eigen::Vector4f &centroid)
{
  return (computeMeanAndCovarianceMatrix (cloud, indices.indices, covariance_matrix, centroid));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                     Eigen::Matrix3d &covariance_matrix,
                                     Eigen::Vector4d &centroid)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<double, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<double, 1, 9, Eigen::RowMajor>::Zero ();
  unsigned int point_count;
  if (cloud.is_dense)
  {
    point_count = cloud.size ();
    // For each point in the cloud
    for (size_t i = 0; i < point_count; ++i)
    {
      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y;
      accu [4] += cloud[i].y * cloud[i].z;
      accu [5] += cloud[i].z * cloud[i].z;
      accu [6] += cloud[i].x;
      accu [7] += cloud[i].y;
      accu [8] += cloud[i].z;
    }
  }
  else
  {
    point_count = 0;
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (!isFinite (cloud[i]))
        continue;

      accu [0] += cloud[i].x * cloud[i].x;
      accu [1] += cloud[i].x * cloud[i].y;
      accu [2] += cloud[i].x * cloud[i].z;
      accu [3] += cloud[i].y * cloud[i].y;
      accu [4] += cloud[i].y * cloud[i].z;
      accu [5] += cloud[i].z * cloud[i].z;
      accu [6] += cloud[i].x;
      accu [7] += cloud[i].y;
      accu [8] += cloud[i].z;
      ++point_count;
    }
  }

  if (point_count != 0)
  {
    accu /= static_cast<double> (point_count);
    centroid.head<3> () = accu.tail<3> ();
    centroid[3] = 0;
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
  return (point_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                const std::vector<int> &indices,
                                Eigen::Matrix3d &covariance_matrix,
                                Eigen::Vector4d &centroid)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<double, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<double, 1, 9, Eigen::RowMajor>::Zero ();
  unsigned point_count;
  if (cloud.is_dense)
  {
    point_count = indices.size ();
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      //const PointT& point = cloud[*iIt];
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y;
      accu [4] += cloud[*iIt].y * cloud[*iIt].z;
      accu [5] += cloud[*iIt].z * cloud[*iIt].z;
      accu [6] += cloud[*iIt].x;
      accu [7] += cloud[*iIt].y;
      accu [8] += cloud[*iIt].z;
    }
  }
  else
  {
    point_count = 0;
    for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
    {
      if (!isFinite (cloud[*iIt]))
        continue;

      ++point_count;
      accu [0] += cloud[*iIt].x * cloud[*iIt].x;
      accu [1] += cloud[*iIt].x * cloud[*iIt].y;
      accu [2] += cloud[*iIt].x * cloud[*iIt].z;
      accu [3] += cloud[*iIt].y * cloud[*iIt].y; // 4
      accu [4] += cloud[*iIt].y * cloud[*iIt].z; // 5
      accu [5] += cloud[*iIt].z * cloud[*iIt].z; // 8
      accu [6] += cloud[*iIt].x;
      accu [7] += cloud[*iIt].y;
      accu [8] += cloud[*iIt].z;
    }
  }

  if (point_count != 0)
  {
    accu /= static_cast<double> (point_count);
    Eigen::Vector3f vec = accu.tail<3> ();
    centroid.head<3> () = vec;//= accu.tail<3> ();
    centroid[3] = 0;
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
  return (point_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                  const pcl::PointIndices &indices,
                                  Eigen::Matrix3d &covariance_matrix,
                                  Eigen::Vector4d &centroid)
{
  return (computeMeanAndCovarianceMatrix (cloud, indices.indices, covariance_matrix, centroid));
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                       const Eigen::Vector4f &centroid,
                       pcl::PointCloud<PointT> &cloud_out)
{
  cloud_out = cloud_in;

  // Subtract the centroid from cloud_in
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
    cloud_out.points[i].getVector4fMap () -= centroid;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                       const std::vector<int> &indices,
                       const Eigen::Vector4f &centroid,
                       pcl::PointCloud<PointT> &cloud_out)
{
  cloud_out.header = cloud_in.header;
  cloud_out.is_dense = cloud_in.is_dense;
  if (indices.size () == cloud_in.points.size ())
  {
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
  }
  else
  {
    cloud_out.width    = static_cast<uint32_t> (indices.size ());
    cloud_out.height   = 1;
  }
  cloud_out.points.resize (indices.size ());

  // Subtract the centroid from cloud_in
  for (size_t i = 0; i < indices.size (); ++i)
    cloud_out.points[i].getVector4fMap () = cloud_in.points[indices[i]].getVector4fMap () -
                                            centroid;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                       const Eigen::Vector4f &centroid,
                       Eigen::MatrixXf &cloud_out)
{
  size_t npts = cloud_in.points.size ();

  cloud_out = Eigen::MatrixXf::Zero (4, npts);        // keep the data aligned

  for (size_t i = 0; i < npts; ++i)
    // One column at a time
    cloud_out.block<4, 1> (0, i) = cloud_in.points[i].getVector4fMap () - centroid;

  // Make sure we zero the 4th dimension out (1 row, N columns)
  cloud_out.block (3, 0, 1, npts).setZero ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                       const std::vector<int> &indices,
                       const Eigen::Vector4f &centroid,
                       Eigen::MatrixXf &cloud_out)
{
  size_t npts = indices.size ();

  cloud_out = Eigen::MatrixXf::Zero (4, npts);        // keep the data aligned

  for (size_t i = 0; i < npts; ++i)
    // One column at a time
    cloud_out.block<4, 1> (0, i) = cloud_in.points[indices[i]].getVector4fMap () - centroid;

  // Make sure we zero the 4th dimension out (1 row, N columns)
  cloud_out.block (3, 0, 1, npts).setZero ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                       const pcl::PointIndices &indices,
                       const Eigen::Vector4f &centroid,
                       Eigen::MatrixXf &cloud_out)
{
  return (pcl::demeanPointCloud (cloud_in, indices.indices, centroid, cloud_out));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::VectorXf &centroid)
{
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  // Get the size of the fields
  centroid.setZero (boost::mpl::size<FieldList>::value);

  if (cloud.points.empty ())
    return;
  // Iterate over each point
  int size = static_cast<int> (cloud.points.size ());
  for (int i = 0; i < size; ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (NdCentroidFunctor <PointT> (cloud.points[i], centroid));
  }
  centroid /= static_cast<float> (size);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                        Eigen::VectorXf &centroid)
{
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  // Get the size of the fields
  centroid.setZero (boost::mpl::size<FieldList>::value);

  if (indices.empty ())
    return;
  // Iterate over each point
  int nr_points = static_cast<int> (indices.size ());
  for (int i = 0; i < nr_points; ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (NdCentroidFunctor <PointT> (cloud.points[indices[i]], centroid));
  }
  centroid /= static_cast<float> (nr_points);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud,
                        const pcl::PointIndices &indices, Eigen::VectorXf &centroid)
{
  return (pcl::computeNDCentroid<PointT> (cloud, indices.indices, centroid));
}

#endif  //#ifndef PCL_COMMON_IMPL_CENTROID_H_

