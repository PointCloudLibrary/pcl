/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_LINEAR_LEAST_SQUARES_NORMAL_HPP_
#define PCL_FEATURES_LINEAR_LEAST_SQUARES_NORMAL_HPP_
#define EIGEN_II_METHOD 1

#include <pcl/features/linear_least_squares_normal.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::LinearLeastSquaresNormalEstimation<PointInT, PointOutT>::~LinearLeastSquaresNormalEstimation () = default;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::LinearLeastSquaresNormalEstimation<PointInT, PointOutT>::computePointNormal (
    const int pos_x, const int pos_y, PointOutT &normal)
{
  const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const int width = input_->width;
  const int height = input_->height;

  const int x = pos_x;
  const int y = pos_y;

  const int index = y * width + x;

  const float px = (*input_)[index].x;
  const float py = (*input_)[index].y;
  const float pz = (*input_)[index].z;

  if (std::isnan (px)) 
  {
    normal.normal_x = bad_point;
    normal.normal_y = bad_point;
    normal.normal_z = bad_point;
    normal.curvature = bad_point;

    return;
  }

  float smoothingSize = normal_smoothing_size_;
  if (use_depth_dependent_smoothing_) smoothingSize = smoothingSize*(pz+0.5f);

  const int smoothingSizeInt = static_cast<int> (smoothingSize);

  float matA0 = 0.0f;
  float matA1 = 0.0f;
  float matA3 = 0.0f;

  float vecb0 = 0.0f;
  float vecb1 = 0.0f;

  for (int v = y - smoothingSizeInt; v <= y + smoothingSizeInt; v += smoothingSizeInt)
  {
    for (int u = x - smoothingSizeInt; u <= x + smoothingSizeInt; u += smoothingSizeInt)
    {
      if (u < 0 || u >= width || v < 0 || v >= height) continue;

      const int index2 = v * width + u;

      const float qx = (*input_)[index2].x;
      const float qy = (*input_)[index2].y;
      const float qz = (*input_)[index2].z;

      if (std::isnan (qx)) continue;

      const float delta = qz - pz;
      const float i = qx - px;
      const float j = qy - py;

      float depthChangeThreshold = pz*pz * 0.05f * max_depth_change_factor_;
      if (use_depth_dependent_smoothing_) depthChangeThreshold *= pz;

      const float f = std::fabs (delta) > depthChangeThreshold ? 0 : 1;

      matA0 += f * i * i;
      matA1 += f * i * j;
      matA3 += f * j * j;
      vecb0 += f * i * delta;
      vecb1 += f * j * delta;
    }
  }

  const float det = matA0 * matA3 - matA1 * matA1;
  const float ddx = matA3 * vecb0 - matA1 * vecb1;
  const float ddy = -matA1 * vecb0 + matA0 * vecb1;

  const float nx = ddx;
  const float ny = ddy;
  const float nz = -det * pz;

  const float length = nx * nx + ny * ny + nz * nz;

  if (length <= 0.01f)
  {
    normal.normal_x = bad_point;
    normal.normal_y = bad_point;
    normal.normal_z = bad_point;
    normal.curvature = bad_point;
  }
  else
  {
    const float normInv = 1.0f / std::sqrt (length);

    normal.normal_x = -nx * normInv;
    normal.normal_y = -ny * normInv;
    normal.normal_z = -nz * normInv;
    normal.curvature = bad_point;
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::LinearLeastSquaresNormalEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const int width = input_->width;
  const int height = input_->height;

  // we compute the normals as follows:
  // ----------------------------------
  // 
  // for the depth-gradient you can make the following first-order Taylor approximation:
  //   D(x + dx) - D(x) = dx^T \Delta D + h.o.t.
  //     
  // build linear system by stacking up equation for 8 neighbor points:
  //   Y = X \Delta D
  // 
  // => \Delta D = (X^T X)^{-1} X^T Y
  // => \Delta D = (A)^{-1} b

  //const float smoothingSize = 30.0f;
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const int index = y * width + x;

      const float px = (*input_)[index].x;
      const float py = (*input_)[index].y;
      const float pz = (*input_)[index].z;

      if (std::isnan(px)) continue;

      //float depthDependentSmoothingSize = smoothingSize + pz / 10.0f;

      float smoothingSize = normal_smoothing_size_;
      //if (use_depth_dependent_smoothing_) smoothingSize *= pz;
      //if (use_depth_dependent_smoothing_) smoothingSize += pz*5;
      //if (smoothingSize < 1.0f) smoothingSize += 1.0f;

      const int smoothingSizeInt = static_cast<int>(smoothingSize);

      float matA0 = 0.0f;
      float matA1 = 0.0f;
      float matA3 = 0.0f;

      float vecb0 = 0.0f;
      float vecb1 = 0.0f;

      for (int v = y - smoothingSizeInt; v <= y + smoothingSizeInt; v += smoothingSizeInt)
      {
        for (int u = x - smoothingSizeInt; u <= x + smoothingSizeInt; u += smoothingSizeInt)
        {
          if (u < 0 || u >= width || v < 0 || v >= height) continue;

          const int index2 = v * width + u;

          const float qx = (*input_)[index2].x;
          const float qy = (*input_)[index2].y;
          const float qz = (*input_)[index2].z;

          if (std::isnan(qx)) continue;

          const float delta = qz - pz;
          const float i = qx - px;
          const float j = qy - py;

          const float depthDependendDepthChange = (max_depth_change_factor_ * (std::abs (pz) + 1.0f) * 2.0f);
          const float f = std::fabs(delta) > depthDependendDepthChange ? 0 : 1;

          //float f = std::abs(delta) > (pz * 0.05f - 0.3f) ? 0 : 1;
          //const float f = std::abs(delta) > (pz*pz * 0.05f * max_depth_change_factor_) ? 0 : 1;
          //float f = Math.Abs(delta) > (depth * Math.Log(depth + 1.0) * 0.02f - 0.2f) ? 0 : 1;

          matA0 += f * i * i;
          matA1 += f * i * j;
          matA3 += f * j * j;
          vecb0 += f * i * delta;
          vecb1 += f * j * delta;
        }
      }

      const float det = matA0 * matA3 - matA1 * matA1;
      const float ddx = matA3 * vecb0 - matA1 * vecb1;
      const float ddy = -matA1 * vecb0 + matA0 * vecb1;

      const float nx = ddx;
      const float ny = ddy;
      const float nz = -det * pz;

      const float length = nx * nx + ny * ny + nz * nz;

      if (length <= 0.0f)
      {
        output[index].normal_x = bad_point;
        output[index].normal_y = bad_point;
        output[index].normal_z = bad_point;
        output[index].curvature = bad_point;
      }
      else
      {
        const float normInv = 1.0f / std::sqrt (length);

        output[index].normal_x = nx * normInv;
        output[index].normal_y = ny * normInv;
        output[index].normal_z = nz * normInv;
        output[index].curvature = bad_point;
      }
    }
  }
}

#define PCL_INSTANTIATE_LinearLeastSquaresNormalEstimation(T,NT) template class PCL_EXPORTS pcl::LinearLeastSquaresNormalEstimation<T,NT>;
//#define LinearLeastSquaresNormalEstimation(T,NT) template class PCL_EXPORTS pcl::LinearLeastSquaresNormalEstimation<T,NT>;

#endif

