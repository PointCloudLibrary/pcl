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
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/centroid.h>
#ifdef __SSE__
#include <xmmintrin.h>
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::setInputCloud (const PointCloudInConstPtr &cloud)
{
  if (normals_ && input_ && (cloud != input_))
    normals_.reset ();
  input_ = cloud;
}

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
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::calculateNormalCovar (const pcl::Indices& neighbors, float* coefficients) const
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

  for (const auto &neighbor : neighbors)
  {
    if (std::isfinite ((*normals_)[neighbor].normal_x))
    {
      // nx, ny, nz, h
      norm1 = _mm_load_ps (&((*normals_)[neighbor].normal_x));

      // nx, nx, nx, nx
      norm2 = _mm_set1_ps ((*normals_)[neighbor].normal_x);

      // nx * nx, nx * ny, nx * nz, nx * h
      norm2 = _mm_mul_ps (norm1, norm2);

      // accumulate
      vec1 = _mm_add_ps (vec1, norm2);

      // ny, ny, ny, ny
      norm2 = _mm_set1_ps ((*normals_)[neighbor].normal_y);

      // ny * nx, ny * ny, ny * nz, ny * h
      norm2 = _mm_mul_ps (norm1, norm2);

      // accumulate
      vec2 = _mm_add_ps (vec2, norm2);

      zz += (*normals_)[neighbor].normal_z * (*normals_)[neighbor].normal_z;
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
    std::fill_n(coefficients, 8, 0);
#else
  std::fill_n(coefficients, 8, 0);
  for (const auto& index : neighbors)
  {
    if (std::isfinite ((*normals_)[index].normal_x))
    {
      coefficients[0] += (*normals_)[index].normal_x * (*normals_)[index].normal_x;
      coefficients[1] += (*normals_)[index].normal_x * (*normals_)[index].normal_y;
      coefficients[2] += (*normals_)[index].normal_x * (*normals_)[index].normal_z;

      coefficients[5] += (*normals_)[index].normal_y * (*normals_)[index].normal_y;
      coefficients[6] += (*normals_)[index].normal_y * (*normals_)[index].normal_z;
      coefficients[7] += (*normals_)[index].normal_z * (*normals_)[index].normal_z;

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
  typename pcl::PointCloud<PointOutT>::Ptr response (new pcl::PointCloud<PointOutT>);

  response->points.reserve (input_->size());

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
    for (std::size_t i = 0; i < response->size (); ++i)
      keypoints_indices_->indices.push_back (i);
  }
  else
  {
    output.clear ();
    output.reserve (response->size());

#pragma omp parallel for \
  default(none) \
  shared(output, response) \
  num_threads(threads_)
    for (int idx = 0; idx < static_cast<int> (response->size ()); ++idx)
    {
      if (!isFinite ((*response)[idx]) ||
          !std::isfinite ((*response)[idx].intensity) ||
          (*response)[idx].intensity < threshold_)
        continue;

      pcl::Indices nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);
      bool is_maxima = true;
      for (const auto& index : nn_indices)
      {
        if ((*response)[idx].intensity < (*response)[index].intensity)
        {
          is_maxima = false;
          break;
        }
      }
      if (is_maxima)
#pragma omp critical
      {
        output.push_back ((*response)[idx]);
        keypoints_indices_->indices.push_back (idx);
      }
    }

    if (refine_)
      refineCorners (output);

    output.height = 1;
    output.width = output.size();
    output.is_dense = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint3D<PointInT, PointOutT, NormalT>::responseHarris (PointCloudOut &output) const
{
  PCL_ALIGN (16) float covar [8];
  output.resize (input_->size ());
#pragma omp parallel for \
  default(none) \
  shared(output) \
  firstprivate(covar) \
  num_threads(threads_)
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0; //std::numeric_limits<float>::quiet_NaN ();
    if (isFinite (pointIn))
    {
      pcl::Indices nn_indices;
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
#pragma omp parallel \
  for default(none) \
  shared(output) \
  firstprivate(covar) \
  num_threads(threads_)
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0;
    if (isFinite (pointIn))
    {
      pcl::Indices nn_indices;
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
#pragma omp parallel for \
  default(none) \
  shared(output) \
  firstprivate(covar) \
  num_threads(threads_)
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0;
    if (isFinite (pointIn))
    {
      pcl::Indices nn_indices;
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
  for (std::size_t idx = 0; idx < input_->size(); ++idx)
  {
    point.x = (*input_)[idx].x;
    point.y = (*input_)[idx].y;
    point.z = (*input_)[idx].z;
    point.intensity = (*normals_)[idx].curvature;
    output.push_back(point);
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
#pragma omp parallel for \
  default(none) \
  shared(output) \
  firstprivate(covar, covariance_matrix) \
  num_threads(threads_)
  for (int pIdx = 0; pIdx < static_cast<int> (input_->size ()); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    output [pIdx].intensity = 0.0;
    if (isFinite (pointIn))
    {
      pcl::Indices nn_indices;
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
  Eigen::Vector3f NNTp;
  const unsigned max_iterations = 10;
#pragma omp parallel for \
  default(none) \
  shared(corners) \
  firstprivate(nnT, NNT, NNTp) \
  num_threads(threads_)
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
      pcl::Indices nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (corner, search_radius_, nn_indices, nn_dists);
      for (const auto& index : nn_indices)
      {
        if (!std::isfinite ((*normals_)[index].normal_x))
          continue;

        nnT = (*normals_)[index].getNormalVector3fMap () * (*normals_)[index].getNormalVector3fMap ().transpose();
        NNT += nnT;
        NNTp += nnT * (*surface_)[index].getVector3fMap ();
      }
      const Eigen::LDLT<Eigen::Matrix3f> ldlt(NNT);
      if (ldlt.rcond() > 1e-4)
        corners[cIdx].getVector3fMap () = ldlt.solve(NNTp);

      const auto diff = (corners[cIdx].getVector3fMap () - corner.getVector3fMap()).squaredNorm ();
      if (diff <= 1e-6) {
        break;
      }
    } while (++iterations < max_iterations);
  }
}

#define PCL_INSTANTIATE_HarrisKeypoint3D(T,U,N) template class PCL_EXPORTS pcl::HarrisKeypoint3D<T,U,N>;
#endif // #ifndef PCL_HARRIS_KEYPOINT_3D_IMPL_H_
