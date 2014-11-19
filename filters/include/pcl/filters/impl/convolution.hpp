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

#ifndef PCL_FILTERS_CONVOLUTION_IMPL_HPP
#define PCL_FILTERS_CONVOLUTION_IMPL_HPP

#include <pcl/pcl_config.h>
#include <pcl/common/distances.h>

template <typename PointIn, typename PointOut>
pcl::filters::Convolution<PointIn, PointOut>::Convolution ()
  : borders_policy_ (BORDERS_POLICY_IGNORE)
  , distance_threshold_ (std::numeric_limits<float>::infinity ())
  , input_ ()
  , kernel_ ()
  , half_width_ ()
  , kernel_width_ ()
  , threads_ (1)
{}

template <typename PointIn, typename PointOut> void
pcl::filters::Convolution<PointIn, PointOut>::initCompute (PointCloud<PointOut>& output)
{
  if (borders_policy_ != BORDERS_POLICY_IGNORE &&
      borders_policy_ != BORDERS_POLICY_MIRROR &&
      borders_policy_ != BORDERS_POLICY_DUPLICATE)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::filters::Convolution::initCompute] unknown borders policy.");

  if(kernel_.size () % 2 == 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::filters::Convolution::initCompute] convolving element width must be odd.");

  if (distance_threshold_ != std::numeric_limits<float>::infinity ())
    distance_threshold_ *= static_cast<float> (kernel_.size () % 2) * distance_threshold_;

  half_width_ = static_cast<int> (kernel_.size ()) / 2;
  kernel_width_ = static_cast<int> (kernel_.size () - 1);

  if (&(*input_) != &output)
  {
    if (output.height != input_->height || output.width != input_->width)
    {
      output.resize (input_->width * input_->height);
      output.width = input_->width;
      output.height = input_->height;
    }
  }
  output.is_dense = input_->is_dense;
}

template <typename PointIn, typename PointOut> inline void
pcl::filters::Convolution<PointIn, PointOut>::convolveRows (PointCloudOut& output)
{
  try
  {
    initCompute (output);
    switch (borders_policy_)
    {
      case BORDERS_POLICY_MIRROR : convolve_rows_mirror (output);
      case BORDERS_POLICY_DUPLICATE : convolve_rows_duplicate (output);
      case BORDERS_POLICY_IGNORE : convolve_rows (output);
    }
  }
  catch (InitFailedException& e)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::filters::Convolution::convolveRows] init failed " << e.what ());
  }
}

template <typename PointIn, typename PointOut> inline void
pcl::filters::Convolution<PointIn, PointOut>::convolveCols (PointCloudOut& output)
{
  try
  {
    initCompute (output);
    switch (borders_policy_)
    {
      case BORDERS_POLICY_MIRROR : convolve_cols_mirror (output);
      case BORDERS_POLICY_DUPLICATE : convolve_cols_duplicate (output);
      case BORDERS_POLICY_IGNORE : convolve_cols (output);
    }
  }
  catch (InitFailedException& e)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::filters::Convolution::convolveCols] init failed " << e.what ());
  }
}

template <typename PointIn, typename PointOut> inline void
pcl::filters::Convolution<PointIn, PointOut>::convolve (const Eigen::ArrayXf& h_kernel,
                                                       const Eigen::ArrayXf& v_kernel,
                                                       PointCloud<PointOut>& output)
{
  try
  {
    PointCloudInPtr tmp (new PointCloud<PointIn> ());
    setKernel (h_kernel);
    convolveRows (*tmp);
    setInputCloud (tmp);
    setKernel (v_kernel);
    convolveCols (output);
  }
  catch (InitFailedException& e)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::filters::Convolution::convolve] init failed " << e.what ());
  }
}

template <typename PointIn, typename PointOut> inline void
pcl::filters::Convolution<PointIn, PointOut>::convolve (PointCloud<PointOut>& output)
{
  try
  {
    PointCloudInPtr tmp (new PointCloud<PointIn> ());
    convolveRows (*tmp);
    setInputCloud (tmp);
    convolveCols (output);
  }
  catch (InitFailedException& e)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::filters::Convolution::convolve] init failed " << e.what ());
  }
}

template <typename PointIn, typename PointOut> inline PointOut
pcl::filters::Convolution<PointIn, PointOut>::convolveOneRowDense (int i, int j)
{
  using namespace pcl::common;
  PointOut result;
  for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
    result+= (*input_) (l,j) * kernel_[k];
  return (result);
}

template <typename PointIn, typename PointOut> inline PointOut
pcl::filters::Convolution<PointIn, PointOut>::convolveOneColDense (int i, int j)
{
  using namespace pcl::common;
  PointOut result;
  for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
    result+= (*input_) (i,l) * kernel_[k];
  return (result);
}

template <typename PointIn, typename PointOut> inline PointOut
pcl::filters::Convolution<PointIn, PointOut>::convolveOneRowNonDense (int i, int j)
{
  using namespace pcl::common;
  PointOut result;
  float weight = 0;
  for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
  {
    if (!isFinite ((*input_) (l,j)))
      continue;
    if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
    {
      result+= (*input_) (l,j) * kernel_[k];
      weight += kernel_[k];
    }
  }
  if (weight == 0)
    result.x = result.y = result.z = std::numeric_limits<float>::quiet_NaN ();
  else
  {
    weight = 1.f/weight;
    result*= weight;
  }
  return (result);
}

template <typename PointIn, typename PointOut> inline PointOut
pcl::filters::Convolution<PointIn, PointOut>::convolveOneColNonDense (int i, int j)
{
  using namespace pcl::common;
  PointOut result;
  float weight = 0;
  for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
  {
    if (!isFinite ((*input_) (i,l)))
      continue;
    if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
    {
      result+= (*input_) (i,l) * kernel_[k];
      weight += kernel_[k];
    }
  }
  if (weight == 0)
    result.x = result.y = result.z = std::numeric_limits<float>::quiet_NaN ();
  else
  {
    weight = 1.f/weight;
    result*= weight;
  }
  return (result);
}

namespace pcl
{
  namespace filters
  {
    template<> pcl::PointXYZRGB
    Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneRowDense (int i, int j)
    {
      pcl::PointXYZRGB result;
      float r = 0, g = 0, b = 0;
      for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
      {
        result.x += (*input_) (l,j).x * kernel_[k];
        result.y += (*input_) (l,j).y * kernel_[k];
        result.z += (*input_) (l,j).z * kernel_[k];
        r += kernel_[k] * static_cast<float> ((*input_) (l,j).r);
        g += kernel_[k] * static_cast<float> ((*input_) (l,j).g);
        b += kernel_[k] * static_cast<float> ((*input_) (l,j).b);
      }
      result.r = static_cast<pcl::uint8_t> (r);
      result.g = static_cast<pcl::uint8_t> (g);
      result.b = static_cast<pcl::uint8_t> (b);
      return (result);
    }

    template<> pcl::PointXYZRGB
    Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneColDense (int i, int j)
    {
      pcl::PointXYZRGB result;
      float r = 0, g = 0, b = 0;
      for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
      {
        result.x += (*input_) (i,l).x * kernel_[k];
        result.y += (*input_) (i,l).y * kernel_[k];
        result.z += (*input_) (i,l).z * kernel_[k];
        r += kernel_[k] * static_cast<float> ((*input_) (i,l).r);
        g += kernel_[k] * static_cast<float> ((*input_) (i,l).g);
        b += kernel_[k] * static_cast<float> ((*input_) (i,l).b);
      }
      result.r = static_cast<pcl::uint8_t> (r);
      result.g = static_cast<pcl::uint8_t> (g);
      result.b = static_cast<pcl::uint8_t> (b);
      return (result);
    }

    template<> pcl::PointXYZRGB
    Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneRowNonDense (int i, int j)
    {
      pcl::PointXYZRGB result;
      float weight = 0;
      float r = 0, g = 0, b = 0;
      for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
      {
        if (!isFinite ((*input_) (l,j)))
          continue;
        if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
        {
          result.x += (*input_) (l,j).x * kernel_[k]; result.y += (*input_) (l,j).y * kernel_[k]; result.z += (*input_) (l,j).z * kernel_[k];
          r+= kernel_[k] * static_cast<float> ((*input_) (l,j).r);
          g+= kernel_[k] * static_cast<float> ((*input_) (l,j).g);
          b+= kernel_[k] * static_cast<float> ((*input_) (l,j).b);
          weight += kernel_[k];
        }
      }

      if (weight == 0)
        result.x = result.y = result.z = std::numeric_limits<float>::quiet_NaN ();
      else
      {
        weight = 1.f/weight;
        r*= weight; g*= weight; b*= weight;
        result.x*= weight; result.y*= weight; result.z*= weight;
        result.r = static_cast<pcl::uint8_t> (r);
        result.g = static_cast<pcl::uint8_t> (g);
        result.b = static_cast<pcl::uint8_t> (b);
      }
      return (result);
    }

    template<> pcl::PointXYZRGB
    Convolution<pcl::PointXYZRGB, pcl::PointXYZRGB>::convolveOneColNonDense (int i, int j)
    {
      pcl::PointXYZRGB result;
      float weight = 0;
      float r = 0, g = 0, b = 0;
      for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
      {
        if (!isFinite ((*input_) (i,l)))
          continue;
        if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
        {
          result.x += (*input_) (i,l).x * kernel_[k]; result.y += (*input_) (i,l).y * kernel_[k]; result.z += (*input_) (i,l).z * kernel_[k];
          r+= kernel_[k] * static_cast<float> ((*input_) (i,l).r);
          g+= kernel_[k] * static_cast<float> ((*input_) (i,l).g);
          b+= kernel_[k] * static_cast<float> ((*input_) (i,l).b);
          weight+= kernel_[k];
        }
      }
      if (weight == 0)
        result.x = result.y = result.z = std::numeric_limits<float>::quiet_NaN ();
      else
      {
        weight = 1.f/weight;
        r*= weight; g*= weight; b*= weight;
        result.x*= weight; result.y*= weight; result.z*= weight;
        result.r = static_cast<pcl::uint8_t> (r);
        result.g = static_cast<pcl::uint8_t> (g);
        result.b = static_cast<pcl::uint8_t> (b);
      }
      return (result);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    template<> pcl::RGB
    Convolution<pcl::RGB, pcl::RGB>::convolveOneRowDense (int i, int j)
    {
      pcl::RGB result;
      float r = 0, g = 0, b = 0;
      for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
      {
        r += kernel_[k] * static_cast<float> ((*input_) (l,j).r);
        g += kernel_[k] * static_cast<float> ((*input_) (l,j).g);
        b += kernel_[k] * static_cast<float> ((*input_) (l,j).b);
      }
      result.r = static_cast<pcl::uint8_t> (r);
      result.g = static_cast<pcl::uint8_t> (g);
      result.b = static_cast<pcl::uint8_t> (b);
      return (result);
    }

    template<> pcl::RGB
    Convolution<pcl::RGB, pcl::RGB>::convolveOneColDense (int i, int j)
    {
      pcl::RGB result;
      float r = 0, g = 0, b = 0;
      for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
      {
        r += kernel_[k] * static_cast<float> ((*input_) (i,l).r);
        g += kernel_[k] * static_cast<float> ((*input_) (i,l).g);
        b += kernel_[k] * static_cast<float> ((*input_) (i,l).b);
      }
      result.r = static_cast<pcl::uint8_t> (r);
      result.g = static_cast<pcl::uint8_t> (g);
      result.b = static_cast<pcl::uint8_t> (b);
      return (result);
    }

    template<> pcl::RGB
    Convolution<pcl::RGB, pcl::RGB>::convolveOneRowNonDense (int i, int j)
    {
      return (convolveOneRowDense (i,j));
    }

    template<> pcl::RGB
    Convolution<pcl::RGB, pcl::RGB>::convolveOneColNonDense (int i, int j)
    {
      return (convolveOneColDense (i,j));
    }

    template<> void
    Convolution<pcl::RGB, pcl::RGB>::makeInfinite (pcl::RGB& p)
    {
      p.r = 0; p.g = 0; p.b = 0;
    }    
  }
}

template <typename PointIn, typename PointOut> void
pcl::filters::Convolution<PointIn, PointOut>::convolve_rows (PointCloudOut& output)
{
  using namespace pcl::common;

  int width = input_->width;
  int height = input_->height;
  int last = input_->width - half_width_;
  if (input_->is_dense)
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int j = 0; j < height; ++j)
    {
      for (int i = 0; i < half_width_; ++i)
        makeInfinite (output (i,j));

      for (int i = half_width_; i < last; ++i)
        output (i,j) = convolveOneRowDense (i,j);

      for (int i = last; i < width; ++i)
        makeInfinite (output (i,j));
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int j = 0; j < height; ++j)
    {
      for (int i = 0; i < half_width_; ++i)
        makeInfinite (output (i,j));

      for (int i = half_width_; i < last; ++i)
        output (i,j) = convolveOneRowNonDense (i,j);

      for (int i = last; i < width; ++i)
        makeInfinite (output (i,j));
    }
  }
}

template <typename PointIn, typename PointOut> void
pcl::filters::Convolution<PointIn, PointOut>::convolve_rows_duplicate (PointCloudOut& output)
{
  using namespace pcl::common;

  int width = input_->width;
  int height = input_->height;
  int last = input_->width - half_width_;
  int w = last - 1;
  if (input_->is_dense)
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
        output (i,j) = convolveOneRowDense (i,j);

      for (int i = last; i < width; ++i)
        output (i,j) = output (w, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_, j);
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
        output (i,j) = convolveOneRowNonDense (i,j);

      for (int i = last; i < width; ++i)
        output (i,j) = output (w, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_, j);
    }
  }
}

template <typename PointIn, typename PointOut> void
pcl::filters::Convolution<PointIn, PointOut>::convolve_rows_mirror (PointCloudOut& output)
{
  using namespace pcl::common;

  int width = input_->width;
  int height = input_->height;
  int last = input_->width - half_width_;
  int w = last - 1;
  if (input_->is_dense)
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
        output (i,j) = convolveOneRowDense (i,j);

      for (int i = last, l = 0; i < width; ++i, ++l)
        output (i,j) = output (w-l, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_+1-i, j);
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
        output (i,j) = convolveOneRowNonDense (i,j);

      for (int i = last, l = 0; i < width; ++i, ++l)
        output (i,j) = output (w-l, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_+1-i, j);
    }
  }
}

template <typename PointIn, typename PointOut> void
pcl::filters::Convolution<PointIn, PointOut>::convolve_cols (PointCloudOut& output)
{
  using namespace pcl::common;

  int width = input_->width;
  int height = input_->height;
  int last = input_->height - half_width_;
  if (input_->is_dense)
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int i = 0; i < width; ++i)
    {
      for (int j = 0; j < half_width_; ++j)
        makeInfinite (output (i,j));

      for (int j = half_width_; j < last; ++j)
        output (i,j) = convolveOneColDense (i,j);

      for (int j = last; j < height; ++j)
        makeInfinite (output (i,j));
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int i = 0; i < width; ++i)
    {
      for (int j = 0; j < half_width_; ++j)
        makeInfinite (output (i,j));

      for (int j = half_width_; j < last; ++j)
        output (i,j) = convolveOneColNonDense (i,j);

      for (int j = last; j < height; ++j)
        makeInfinite (output (i,j));
    }
  }
}

template <typename PointIn, typename PointOut> void
pcl::filters::Convolution<PointIn, PointOut>::convolve_cols_duplicate (PointCloudOut& output)
{
  using namespace pcl::common;

  int width = input_->width;
  int height = input_->height;
  int last = input_->height - half_width_;
  int h = last -1;
  if (input_->is_dense)
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
        output (i,j) = convolveOneColDense (i,j);

      for (int j = last; j < height; ++j)
        output (i,j) = output (i,h);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_);
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
        output (i,j) = convolveOneColNonDense (i,j);

      for (int j = last; j < height; ++j)
        output (i,j) = output (i,h);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i,half_width_);
    }
  }
}

template <typename PointIn, typename PointOut> void
pcl::filters::Convolution<PointIn, PointOut>::convolve_cols_mirror (PointCloudOut& output)
{
  using namespace pcl::common;

  int width = input_->width;
  int height = input_->height;
  int last = input_->height - half_width_;
  int h = last -1;
  if (input_->is_dense)
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
        output (i,j) = convolveOneColDense (i,j);

      for (int j = last, l = 0; j < height; ++j, ++l)
        output (i,j) = output (i,h-l);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_+1-j);
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads (threads_)
#endif
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
        output (i,j) = convolveOneColNonDense (i,j);

      for (int j = last, l = 0; j < height; ++j, ++l)
        output (i,j) = output (i,h-l);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i,half_width_+1-j);
    }
  }
}

#endif //PCL_FILTERS_CONVOLUTION_IMPL_HPP
