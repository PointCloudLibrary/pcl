/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
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

#include <pcl/filters/convolution.h>

namespace pcl {
namespace filters {
template <>
pcl::PointXYZRGB
Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneRowDense(int i, int j)
{
  pcl::PointXYZRGB result;
  float r = 0, g = 0, b = 0;
  for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l) {
    result.x += (*input_)(l, j).x * kernel_[k];
    result.y += (*input_)(l, j).y * kernel_[k];
    result.z += (*input_)(l, j).z * kernel_[k];
    r += kernel_[k] * static_cast<float>((*input_)(l, j).r);
    g += kernel_[k] * static_cast<float>((*input_)(l, j).g);
    b += kernel_[k] * static_cast<float>((*input_)(l, j).b);
  }
  result.r = static_cast<std::uint8_t>(r);
  result.g = static_cast<std::uint8_t>(g);
  result.b = static_cast<std::uint8_t>(b);
  return (result);
}

template <>
pcl::PointXYZRGB
Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneColDense(int i, int j)
{
  pcl::PointXYZRGB result;
  float r = 0, g = 0, b = 0;
  for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l) {
    result.x += (*input_)(i, l).x * kernel_[k];
    result.y += (*input_)(i, l).y * kernel_[k];
    result.z += (*input_)(i, l).z * kernel_[k];
    r += kernel_[k] * static_cast<float>((*input_)(i, l).r);
    g += kernel_[k] * static_cast<float>((*input_)(i, l).g);
    b += kernel_[k] * static_cast<float>((*input_)(i, l).b);
  }
  result.r = static_cast<std::uint8_t>(r);
  result.g = static_cast<std::uint8_t>(g);
  result.b = static_cast<std::uint8_t>(b);
  return (result);
}

template <>
pcl::PointXYZRGB
Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneRowNonDense(int i, int j)
{
  pcl::PointXYZRGB result;
  float weight = 0;
  float r = 0, g = 0, b = 0;
  for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l) {
    if (!isFinite((*input_)(l, j)))
      continue;
    if (pcl::squaredEuclideanDistance((*input_)(i, j), (*input_)(l, j)) <
        distance_threshold_) {
      result.x += (*input_)(l, j).x * kernel_[k];
      result.y += (*input_)(l, j).y * kernel_[k];
      result.z += (*input_)(l, j).z * kernel_[k];
      r += kernel_[k] * static_cast<float>((*input_)(l, j).r);
      g += kernel_[k] * static_cast<float>((*input_)(l, j).g);
      b += kernel_[k] * static_cast<float>((*input_)(l, j).b);
      weight += kernel_[k];
    }
  }

  if (weight == 0)
    result.x = result.y = result.z = std::numeric_limits<float>::quiet_NaN();
  else {
    weight = 1.f / weight;
    r *= weight;
    g *= weight;
    b *= weight;
    result.x *= weight;
    result.y *= weight;
    result.z *= weight;
    result.r = static_cast<std::uint8_t>(r);
    result.g = static_cast<std::uint8_t>(g);
    result.b = static_cast<std::uint8_t>(b);
  }
  return (result);
}

template <>
pcl::PointXYZRGB
Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneColNonDense(int i, int j)
{
  pcl::PointXYZRGB result;
  float weight = 0;
  float r = 0, g = 0, b = 0;
  for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l) {
    if (!isFinite((*input_)(i, l)))
      continue;
    if (pcl::squaredEuclideanDistance((*input_)(i, j), (*input_)(i, l)) <
        distance_threshold_) {
      result.x += (*input_)(i, l).x * kernel_[k];
      result.y += (*input_)(i, l).y * kernel_[k];
      result.z += (*input_)(i, l).z * kernel_[k];
      r += kernel_[k] * static_cast<float>((*input_)(i, l).r);
      g += kernel_[k] * static_cast<float>((*input_)(i, l).g);
      b += kernel_[k] * static_cast<float>((*input_)(i, l).b);
      weight += kernel_[k];
    }
  }
  if (weight == 0)
    result.x = result.y = result.z = std::numeric_limits<float>::quiet_NaN();
  else {
    weight = 1.f / weight;
    r *= weight;
    g *= weight;
    b *= weight;
    result.x *= weight;
    result.y *= weight;
    result.z *= weight;
    result.r = static_cast<std::uint8_t>(r);
    result.g = static_cast<std::uint8_t>(g);
    result.b = static_cast<std::uint8_t>(b);
  }
  return (result);
}

template <>
pcl::RGB
Convolution<pcl::RGB, pcl::RGB>::convolveOneRowDense(int i, int j)
{
  pcl::RGB result;
  float r = 0, g = 0, b = 0;
  for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l) {
    r += kernel_[k] * static_cast<float>((*input_)(l, j).r);
    g += kernel_[k] * static_cast<float>((*input_)(l, j).g);
    b += kernel_[k] * static_cast<float>((*input_)(l, j).b);
  }
  result.r = static_cast<std::uint8_t>(r);
  result.g = static_cast<std::uint8_t>(g);
  result.b = static_cast<std::uint8_t>(b);
  return (result);
}

template <>
pcl::RGB
Convolution<pcl::RGB, pcl::RGB>::convolveOneColDense(int i, int j)
{
  pcl::RGB result;
  float r = 0, g = 0, b = 0;
  for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l) {
    r += kernel_[k] * static_cast<float>((*input_)(i, l).r);
    g += kernel_[k] * static_cast<float>((*input_)(i, l).g);
    b += kernel_[k] * static_cast<float>((*input_)(i, l).b);
  }
  result.r = static_cast<std::uint8_t>(r);
  result.g = static_cast<std::uint8_t>(g);
  result.b = static_cast<std::uint8_t>(b);
  return (result);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

PCL_INSTANTIATE_PRODUCT(
    Convolution, ((pcl::RGB))((pcl::RGB)))

PCL_INSTANTIATE_PRODUCT(
    Convolution, ((pcl::PointXYZRGB))((pcl::PointXYZRGB)))
#endif // PCL_NO_PRECOMPILE

} // namespace filters
} // namespace pcl
