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
 * $Id: centroid.hpp 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#ifndef PCL_COMMON_IMPL_CENTROID_H_
#define PCL_COMMON_IMPL_CENTROID_H_

#include "pcl/ros/conversions.h"
#include <boost/mpl/size.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid)
{
  // Initialize to 0
  centroid.setZero ();
  if (cloud.points.empty ()) 
    return;
  // For each point in the cloud
  int cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      centroid += cloud.points[i].getVector4fMap ();
    centroid[3] = 0;
    centroid /= cloud.points.size ();
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || 
          !pcl_isfinite (cloud.points[i].y) || 
          !pcl_isfinite (cloud.points[i].z))
        continue;

      centroid += cloud.points[i].getVector4fMap ();
      cp++;
    }
    centroid[3] = 0;
    centroid /= cp;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                        Eigen::Vector4f &centroid)
{
  // Initialize to 0
  centroid.setZero ();
  if (indices.empty ()) 
    return;
  // For each point in the cloud
  int cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < indices.size (); ++i)
      centroid += cloud.points[indices[i]].getVector4fMap ();
    centroid[3] = 0;
    centroid /= indices.size ();
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices[i]].x) || 
          !pcl_isfinite (cloud.points[indices[i]].y) || 
          !pcl_isfinite (cloud.points[indices[i]].z))
        continue;

      centroid += cloud.points[indices[i]].getVector4fMap ();
      cp++;
    }
    centroid[3] = 0;
    centroid /= cp;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                        const pcl::PointIndices &indices, Eigen::Vector4f &centroid)
{
  return (pcl::compute3DCentroid<PointT> (cloud, indices.indices, centroid));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                              const Eigen::Vector4f &centroid, 
                              Eigen::Matrix3f &covariance_matrix)
{
  // Initialize to 0
  covariance_matrix.setZero ();

  if (cloud.points.empty ())
    return;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    // For each point in the cloud
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      Eigen::Vector4f pt = cloud.points[i].getVector4fMap () - centroid;

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
    // For each point in the cloud
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || 
          !pcl_isfinite (cloud.points[i].y) || 
          !pcl_isfinite (cloud.points[i].z))
        continue;

      Eigen::Vector4f pt = cloud.points[i].getVector4fMap () - centroid;

      covariance_matrix (1, 1) += pt.y () * pt.y ();
      covariance_matrix (1, 2) += pt.y () * pt.z ();

      covariance_matrix (2, 2) += pt.z () * pt.z ();

      pt *= pt.x ();
      covariance_matrix (0, 0) += pt.x ();
      covariance_matrix (0, 1) += pt.y ();
      covariance_matrix (0, 2) += pt.z ();
    }
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
                                        const Eigen::Vector4f &centroid, 
                                        Eigen::Matrix3f &covariance_matrix)
{
  pcl::computeCovarianceMatrix (cloud, centroid, covariance_matrix);
  covariance_matrix /= cloud.points.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud, 
                              const std::vector<int> &indices,
                              const Eigen::Vector4f &centroid, 
                              Eigen::Matrix3f &covariance_matrix)
{
  // Initialize to 0
  covariance_matrix.setZero ();

  if (indices.empty ())
    return;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    // For each point in the cloud
    for (size_t i = 0; i < indices.size (); ++i)
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
    // For each point in the cloud
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices[i]].x) || 
          !pcl_isfinite (cloud.points[indices[i]].y) || 
          !pcl_isfinite (cloud.points[indices[i]].z))
        continue;

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
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  covariance_matrix (2, 0) = covariance_matrix (0, 2);
  covariance_matrix (2, 1) = covariance_matrix (1, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud, 
                              const pcl::PointIndices &indices,
                              const Eigen::Vector4f &centroid, 
                              Eigen::Matrix3f &covariance_matrix)
{
  return (pcl::computeCovarianceMatrix<PointT> (cloud, indices.indices, centroid));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud, 
                                        const std::vector<int> &indices,
                                        const Eigen::Vector4f &centroid, 
                                        Eigen::Matrix3f &covariance_matrix)
{
  pcl::computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix);
  covariance_matrix /= indices.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud, 
                                        const pcl::PointIndices &indices,
                                        const Eigen::Vector4f &centroid, 
                                        Eigen::Matrix3f &covariance_matrix)
{
  return (pcl::computeCovarianceMatrix (cloud, indices.indices, centroid, covariance_matrix));
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
    cloud_out.width    = indices.size ();
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
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::VectorXf &centroid)
{
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  // Get the size of the fields
  centroid.setZero (boost::mpl::size<FieldList>::value);

  if (cloud.points.empty ())
    return;
  // Iterate over each point
  int size = cloud.points.size ();
  for (int i = 0; i < size; ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (NdCentroidFunctor <PointT> (cloud.points[i], centroid));
  }
  centroid /= size;
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
  int nr_points = indices.size ();
  for (int i = 0; i < nr_points; ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (NdCentroidFunctor <PointT> (cloud.points[indices[i]], centroid));
  }
  centroid /= nr_points;
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                        const pcl::PointIndices &indices, Eigen::VectorXf &centroid)
{
  return (pcl::computeNDCentroid<PointT> (cloud, indices.indices, centroid));
}

#endif  //#ifndef PCL_COMMON_IMPL_CENTROID_H_

