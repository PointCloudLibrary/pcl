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
 */

#ifndef PCL_HARRIS_KEYPOINT_3D_IMPL_H_
#define PCL_HARRIS_KEYPOINT_3D_IMPL_H_

#include <pcl/keypoints/harris_3d.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#ifdef __SSE__
#include <xmmintrin.h>
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::setMethod (ResponseMethod method)
{
  method_ = method;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::setThreshold (float threshold)
{
  threshold_= threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::setRadius (float radius)
{
  search_radius_ = radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::setRefine (bool do_refine)
{
  refine_ = do_refine;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::setNonMaxSupression (bool nonmax)
{
  nonmax_ = nonmax;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::setNormals (const PointCloudNConstPtr &normals)
{
  normals_ = normals;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::calculateNormalCovar (const std::vector<int>& neighbors, float* coefficients) const
{
  unsigned count = 0;
  // indices        0   1   2   3   4   5   6   7
  // coefficients: xx  xy  xz  ??  yx  yy  yz  ??
#ifdef __SSE__
  // accumulator for xx, xy, xz
  __m128 vec1 = _mm_setzero_ps();
  // accumulator for yy, yz, zz
  __m128 vec2 = _mm_setzero_ps();

  __m128 norm1;

  __m128 norm2;

  float zz = 0;

  for (std::vector<int>::const_iterator iIt = neighbors.begin(); iIt != neighbors.end(); ++iIt)
  {
    if (pcl_isfinite (normals_->points[*iIt].normal_x))
    {
      // nx, ny, nz, h
      norm1 = _mm_load_ps (&(normals_->points[*iIt].normal_x));

      // nx, nx, nx, nx
      norm2 = _mm_set1_ps (normals_->points[*iIt].normal_x);

      // nx * nx, nx * ny, nx * nz, nx * h
      norm2 = _mm_mul_ps (norm1, norm2);

      // accumulate
      vec1 = _mm_add_ps (vec1, norm2);

      // ny, ny, ny, ny
      norm2 = _mm_set1_ps (normals_->points[*iIt].normal_y);

      // ny * nx, ny * ny, ny * nz, ny * h
      norm2 = _mm_mul_ps (norm1, norm2);

      // accumulate
      vec2 = _mm_add_ps (vec2, norm2);

      zz += normals_->points[*iIt].normal_z * normals_->points[*iIt].normal_z;
      ++count;
    }
  }
  if (count > 0)
  {
    norm2 = _mm_set1_ps (float(count));
    vec1 = _mm_div_ps (vec1, norm2);
    vec2 = _mm_div_ps (vec2, norm2);
    _mm_store_ps (coefficients, vec1);
    _mm_store_ps (coefficients + 4, vec2);
    coefficients [7] = zz / float(count);
  }
  else
    memset (coefficients, 0, sizeof (float) * 8);
#else
  memset (coefficients, 0, sizeof (float) * 8);
  for (std::vector<int>::const_iterator iIt = neighbors.begin(); iIt != neighbors.end(); ++iIt)
  {
    if (pcl_isfinite (normals_->points[*iIt].normal_x))
    {
      coefficients[0] += normals_->points[*iIt].normal_x * normals_->points[*iIt].normal_x;
      coefficients[1] += normals_->points[*iIt].normal_x * normals_->points[*iIt].normal_y;
      coefficients[2] += normals_->points[*iIt].normal_x * normals_->points[*iIt].normal_z;

      coefficients[5] += normals_->points[*iIt].normal_y * normals_->points[*iIt].normal_y;
      coefficients[6] += normals_->points[*iIt].normal_y * normals_->points[*iIt].normal_z;
      coefficients[7] += normals_->points[*iIt].normal_z * normals_->points[*iIt].normal_z;

      ++count;
    }
  }
  if (count > 0)
  {
    float norm = 1.0 / float (count);
    coefficients[0] *= norm;
    coefficients[1] *= norm;
    coefficients[2] *= norm;
    coefficients[5] *= norm;
    coefficients[6] *= norm;
    coefficients[7] *= norm;
  }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> bool
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }

  if (method_ < 1 || method_ > 5)
  {
    PCL_ERROR ("[pcl::%s::initCompute] method (%d) must be in [1..5]!\n", name_.c_str (), method_);
    return (false);
  }

  if (!normals_)
  {
    PointCloudNPtr normals (new PointCloudN ());
    normals->reserve (normals->size ());
    if (!surface_->isOrganized ())
    {
      pcl::NormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setInputCloud (surface_);
      normal_estimation.setRadiusSearch (search_radius_);
      normal_estimation.compute (*normals);
    }
    else
    {
      IntegralImageNormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointInT, NormalT>::SIMPLE_3D_GRADIENT);
      normal_estimation.setInputCloud (surface_);
      normal_estimation.setNormalSmoothingSize (5.0);
      normal_estimation.compute (*normals);
    }
    normals_ = normals;
  }
  if (normals_->size () != surface_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] normals given, but the number of normals does not match the number of input points!\n", name_.c_str (), method_);
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::detectKeypoints (PointCloudOut &output)
{
  boost::shared_ptr<pcl::PointCloud<PointOutT> > response (new pcl::PointCloud<PointOutT> ());

  response->points.reserve (input_->points.size());

  switch (method_)
  {
    case HARRIS:
      responseHarris(*response);
      break;
    case NOBLE:
      responseNoble(*response);
      break;
    case LOWE:
      responseLowe(*response);
      break;
    case CURVATURE:
      responseCurvature(*response);
      break;
    case TOMASI:
      responseTomasi(*response);
      break;
  }

  if (!nonmax_)
  {
    output = *response;
    // we do not change the denseness in this case
    output.is_dense = input_->is_dense;
    for (size_t i = 0; i < response->size (); ++i)
      keypoints_indices_->indices.push_back (i);
  }
  else
  {
    output.points.clear ();
    output.points.reserve (response->points.size());

#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads(threads_)   
#endif
    for (int idx = 0; idx < static_cast<int> (response->points.size ()); ++idx)
    {
      if (!isFinite (response->points[idx]) ||
          !pcl_isfinite (response->points[idx].intensity) ||
          response->points[idx].intensity < threshold_)
        continue;

      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);
      bool is_maxima = true;
      for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
      {
        if (response->points[idx].intensity < response->points[*iIt].intensity)
        {
          is_maxima = false;
          break;
        }
      }
      if (is_maxima)
#ifdef _OPENMP
#pragma omp critical
#endif
      {
        output.points.push_back (response->points[idx]);
        keypoints_indices_->indices.push_back (idx);
      }
    }

    if (refine_)
      refineCorners (output);

    output.height = 1;
    output.width = static_cast<uint32_t> (output.points.size());
    output.is_dense = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::responseHarris (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [8];
  output.resize (input_->size ());
#ifdef _OPENMP
  #pragma omp parallel for shared (output) private (covar) num_threads(threads_)
#endif
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0; //std::numeric_limits<float>::quiet_NaN ();
    if (isFinite (pointIn))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (pointIn, search_radius_, nn_indices, nn_dists);
      calculateNormalCovar (nn_indices, covar);

      float trace = covar [0] + covar [5] + covar [7];
      if (trace != 0)
      {
        float det = covar [0] * covar [5] * covar [7] + 2.0f * covar [1] * covar [2] * covar [6]
                  - covar [2] * covar [2] * covar [5]
                  - covar [1] * covar [1] * covar [7]
                  - covar [6] * covar [6] * covar [0];

        output [pIdx].intensity = 0.04f + det - 0.04f * trace * trace;
      }
    }
    output [pIdx].x = pointIn.x;
    output [pIdx].y = pointIn.y;
    output [pIdx].z = pointIn.z;
  }
  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::responseNoble (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [8];
  output.resize (input_->size ());
#ifdef _OPENMP
  #pragma omp parallel for shared (output) private (covar) num_threads(threads_)
#endif
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0;
    if (isFinite (pointIn))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (pointIn, search_radius_, nn_indices, nn_dists);
      calculateNormalCovar (nn_indices, covar);
      float trace = covar [0] + covar [5] + covar [7];
      if (trace != 0)
      {
        float det = covar [0] * covar [5] * covar [7] + 2.0f * covar [1] * covar [2] * covar [6]
                  - covar [2] * covar [2] * covar [5]
                  - covar [1] * covar [1] * covar [7]
                  - covar [6] * covar [6] * covar [0];

        output [pIdx].intensity = det / trace;
      }
    }
    output [pIdx].x = pointIn.x;
    output [pIdx].y = pointIn.y;
    output [pIdx].z = pointIn.z;
  }
  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::responseLowe (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [8];
  output.resize (input_->size ());
#ifdef _OPENMP
  #pragma omp parallel for shared (output) private (covar) num_threads(threads_)
#endif
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0;
    if (isFinite (pointIn))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (pointIn, search_radius_, nn_indices, nn_dists);
      calculateNormalCovar (nn_indices, covar);
      float trace = covar [0] + covar [5] + covar [7];
      if (trace != 0)
      {
        float det = covar [0] * covar [5] * covar [7] + 2.0f * covar [1] * covar [2] * covar [6]
                  - covar [2] * covar [2] * covar [5]
                  - covar [1] * covar [1] * covar [7]
                  - covar [6] * covar [6] * covar [0];

        output [pIdx].intensity = det / (trace * trace);
      }
    }
    output [pIdx].x = pointIn.x;
    output [pIdx].y = pointIn.y;
    output [pIdx].z = pointIn.z;
  }
  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::responseCurvature (PointCloudOut &output) const
{
  PointOutT point;
  for (unsigned idx = 0; idx < input_->points.size(); ++idx)
  {
    point.x = input_->points[idx].x;
    point.y = input_->points[idx].y;
    point.z = input_->points[idx].z;
    point.intensity = normals_->points[idx].curvature;
    output.points.push_back(point);
  }
  // does not change the order
  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::responseTomasi (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [8];
  Eigen::Matrix3f covariance_matrix;
  output.resize (input_->size ());
#ifdef _OPENMP
  #pragma omp parallel for shared (output) private (covar, covariance_matrix) num_threads(threads_)
#endif
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0;
    if (isFinite (pointIn))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (pointIn, search_radius_, nn_indices, nn_dists);
      calculateNormalCovar (nn_indices, covar);
      float trace = covar [0] + covar [5] + covar [7];
      if (trace != 0)
      {
        covariance_matrix.coeffRef (0) = covar [0];
        covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = covar [1];
        covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = covar [2];
        covariance_matrix.coeffRef (4) = covar [5];
        covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = covar [6];
        covariance_matrix.coeffRef (8) = covar [7];

        EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
        pcl::eigen33(covariance_matrix, eigen_values);
        output [pIdx].intensity = eigen_values[0];
      }
    }
    output [pIdx].x = pointIn.x;
    output [pIdx].y = pointIn.y;
    output [pIdx].z = pointIn.z;
  }
  output.height = input_->height;
  output.width = input_->width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::refineCorners (PointCloudOut &corners) const
{
  Eigen::Matrix3f nnT;
  Eigen::Matrix3f NNT;
  Eigen::Matrix3f NNTInv;
  Eigen::Vector3f NNTp;
  float diff;
  const unsigned max_iterations = 10;
#ifdef _OPENMP
  #pragma omp parallel for shared (corners) private (nnT, NNT, NNTInv, NNTp, diff) num_threads(threads_)
#endif
  for (int cIdx = 0; cIdx < static_cast<int> (corners.size ()); ++cIdx)
  {
    unsigned iterations = 0;
    do {
      NNT.setZero();
      NNTp.setZero();
      PointInT corner;
      corner.x = corners[cIdx].x;
      corner.y = corners[cIdx].y;
      corner.z = corners[cIdx].z;
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (corner, search_radius_, nn_indices, nn_dists);
      for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
      {
        if (!pcl_isfinite (normals_->points[*iIt].normal_x))
          continue;

        nnT = normals_->points[*iIt].getNormalVector3fMap () * normals_->points[*iIt].getNormalVector3fMap ().transpose();
        NNT += nnT;
        NNTp += nnT * surface_->points[*iIt].getVector3fMap ();
      }
      if (invert3x3SymMatrix (NNT, NNTInv) != 0)
        corners[cIdx].getVector3fMap () = NNTInv * NNTp;

      diff = (corners[cIdx].getVector3fMap () - corner.getVector3fMap()).squaredNorm ();
    } while (diff > 1e-6 && ++iterations < max_iterations);
  }
}

#define PCL_INSTANTIATE_HarrisKeypoint3D(T,U,N) template class PCL_EXPORTS pcl::HarrisKeypoint3D<T,U,N>;
#endif // #ifndef PCL_HARRIS_KEYPOINT_3D_IMPL_H_

