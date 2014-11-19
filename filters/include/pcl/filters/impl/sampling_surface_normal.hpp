/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_FILTERS_IMPL_SAMPLING_SURFACE_NORMAL_H_
#define PCL_FILTERS_IMPL_SAMPLING_SURFACE_NORMAL_H_

#include <iostream>
#include <vector>
#include <pcl/common/eigen.h>
#include <pcl/filters/sampling_surface_normal.h>

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::SamplingSurfaceNormal<PointT>::applyFilter (PointCloud &output)
{
  std::vector <int> indices;
  size_t npts = input_->points.size ();
  for (unsigned int i = 0; i < npts; i++)
    indices.push_back (i);

  Vector max_vec (3, 1);
  Vector min_vec (3, 1);
  findXYZMaxMin (*input_, max_vec, min_vec);
  PointCloud data = *input_;
  partition (data, 0, npts, min_vec, max_vec, indices, output);
  output.width = 1;
  output.height = uint32_t (output.points.size ());
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void 
pcl::SamplingSurfaceNormal<PointT>::findXYZMaxMin (const PointCloud& cloud, Vector& max_vec, Vector& min_vec)
{
  float maxval = cloud.points[0].x;
  float minval = cloud.points[0].x;

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    if (cloud.points[i].x > maxval)
    {
      maxval = cloud.points[i].x;
    }
    if (cloud.points[i].x < minval)
    {
      minval = cloud.points[i].x;
    }
  }
  max_vec (0) = maxval;
  min_vec (0) = minval;

  maxval = cloud.points[0].y;
  minval = cloud.points[0].y;

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    if (cloud.points[i].y > maxval)
    {
      maxval = cloud.points[i].y;
    }
    if (cloud.points[i].y < minval)
    {
      minval = cloud.points[i].y;
    }
  }
  max_vec (1) = maxval;
  min_vec (1) = minval;

  maxval = cloud.points[0].z;
  minval = cloud.points[0].z;

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    if (cloud.points[i].z > maxval)
    {
      maxval = cloud.points[i].z;
    }
    if (cloud.points[i].z < minval)
    {
      minval = cloud.points[i].z;
    }
  }
  max_vec (2) = maxval;
  min_vec (2) = minval;
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void 
pcl::SamplingSurfaceNormal<PointT>::partition (
    const PointCloud& cloud, const int first, const int last,
    const Vector min_values, const Vector max_values, 
    std::vector<int>& indices, PointCloud&  output)
{
	const int count (last - first);
  if (count <= static_cast<int> (sample_))
  {
    samplePartition (cloud, first, last, indices, output);
    return;
  }
	int cutDim = 0;
  (max_values - min_values).maxCoeff (&cutDim);

	const int rightCount (count / 2);
	const int leftCount (count - rightCount);
	assert (last - rightCount == first + leftCount);
	
	// sort, hack std::nth_element
	std::nth_element (indices.begin () + first, indices.begin () + first + leftCount,
                    indices.begin () + last, CompareDim (cutDim, cloud));

	const int cutIndex (indices[first+leftCount]);
	const float cutVal = findCutVal (cloud, cutDim, cutIndex);
	
	// update bounds for left
	Vector leftMaxValues (max_values);
	leftMaxValues[cutDim] = cutVal;
	// update bounds for right
	Vector rightMinValues (min_values);
	rightMinValues[cutDim] = cutVal;
	
	// recurse
	partition (cloud, first, first + leftCount, min_values, leftMaxValues, indices, output);
	partition (cloud, first + leftCount, last, rightMinValues, max_values, indices, output);
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void 
pcl::SamplingSurfaceNormal<PointT>::samplePartition (
    const PointCloud& data, const int first, const int last,
    std::vector <int>& indices, PointCloud& output)
{
  pcl::PointCloud <PointT> cloud;
  
  for (int i = first; i < last; i++)
  {
    PointT pt;
    pt.x = data.points[indices[i]].x;
    pt.y = data.points[indices[i]].y;
    pt.z = data.points[indices[i]].z;
    cloud.points.push_back (pt);
  }
  cloud.width = 1;
  cloud.height = uint32_t (cloud.points.size ());

  Eigen::Vector4f normal;
  float curvature = 0;
  //pcl::computePointNormal<PointT> (cloud, normal, curvature);

  computeNormal (cloud, normal, curvature);

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    // TODO: change to Boost random number generators!
    const float r = float (std::rand ()) / float (RAND_MAX);

    if (r < ratio_)
    {
      PointT pt = cloud.points[i];
      pt.normal[0] = normal (0);
      pt.normal[1] = normal (1);
      pt.normal[2] = normal (2);
      pt.curvature = curvature;

      output.points.push_back (pt);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::SamplingSurfaceNormal<PointT>::computeNormal (const PointCloud& cloud, Eigen::Vector4f &normal, float& curvature)
{
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;
  float nx = 0.0;
  float ny = 0.0;
  float nz = 0.0;

  if (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, xyz_centroid) == 0)
  {
    nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
    return;
  }

  // Get the plane normal and surface curvature
  solvePlaneParameters (covariance_matrix, nx, ny, nz, curvature);
  normal (0) = nx;
  normal (1) = ny;
  normal (2) = nz;

  normal (3) = 0;
  // Hessian form (D = nc . p_plane (centroid here) + p)
  normal (3) = -1 * normal.dot (xyz_centroid);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline unsigned int
pcl::SamplingSurfaceNormal<PointT>::computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                                                    Eigen::Matrix3f &covariance_matrix,
                                                                    Eigen::Vector4f &centroid)
{
  // create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
  Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero ();
  unsigned int point_count = 0;
  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    if (!isFinite (cloud[i]))
    {
      continue;
    }

    ++point_count;
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

  accu /= static_cast<float> (point_count);
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
template <typename PointT> void
pcl::SamplingSurfaceNormal<PointT>::solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                                                          float &nx, float &ny, float &nz, float &curvature)
{
  // Extract the smallest eigenvalue and its eigenvector
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

  nx = eigen_vector [0];
  ny = eigen_vector [1];
  nz = eigen_vector [2];

  // Compute the curvature surface change
  float eig_sum = covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8);
  if (eig_sum != 0)
    curvature = fabsf (eigen_value / eig_sum);
  else
    curvature = 0;
}

///////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::SamplingSurfaceNormal<PointT>::findCutVal (
    const PointCloud& cloud, const int cut_dim, const int cut_index)
{
  if (cut_dim == 0)
    return (cloud.points[cut_index].x);
  else if (cut_dim == 1)
    return (cloud.points[cut_index].y);
  else if (cut_dim == 2)
    return (cloud.points[cut_index].z);

  return (0.0f);
}


#define PCL_INSTANTIATE_SamplingSurfaceNormal(T) template class PCL_EXPORTS pcl::SamplingSurfaceNormal<T>;

#endif    // PCL_FILTERS_IMPL_NORMAL_SPACE_SAMPLE_H_
