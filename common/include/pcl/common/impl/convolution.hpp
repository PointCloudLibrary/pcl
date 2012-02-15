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
 * $Id$
 *
 */

#ifndef PCL_COMMON_CONVOLUTION_IMPL_HPP
#define PCL_COMMON_CONVOLUTION_IMPL_HPP

template <typename PointIn, typename PointOut> void
pcl::common::AbstractConvolution<PointIn, PointOut>::initCompute (PointCloudOut& output)
{
  if (borders_policy_ != IGNORE && borders_policy_ != MIRROR && borders_policy_ != DUPLICATE)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::Convolution::initCompute] unknown borders policy.");

  if(kernel_.size () % 2 == 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::Convolution::initCompute] convolving element width must be odd.");
  
  if (distance_threshold_ != std::numeric_limits<float>::infinity ())
    distance_threshold_*= (kernel_.size () % 2) * distance_threshold_;
}

template <typename PointT> void
pcl::common::Convolution<PointT>::initCompute (PointCloud<PointT>& output)
{
  try
  {
    pcl::common::AbstractConvolution<PointT, PointT>::initCompute (output);
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
  catch (InitFailedException& e)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::Convolution::initCompute] " << e.what ());
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::initCompute (PointCloudOut& output)
{
  try
  {
    pcl::common::AbstractConvolution<PointIn, PointOut>::initCompute (output);
    if (output.height != input_->height || output.width != input_->width)
    {
      output.resize (input_->width * input_->height);
      output.width = input_->width;
      output.height = input_->height;
    }
    output.is_dense = input_->is_dense;
  }
  catch (InitFailedException& e)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::ConvolutionWithTransform::initCompute] " << e.what ());
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_rows ( PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->width - half_width_;
  Eigen::ArrayXf kernel = kernel_;
  
  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = 0; i < half_width_; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      
      for (int i = half_width_; i < last; ++i)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (l,j) * kernel [k];
      }
      
      for (int i = last; i < width; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = 0; i < half_width_; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      
      for (int i = half_width_; i < last; ++i)
      {
        int counter = 0;
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (l,j)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
          {
            output (i,j)+= (*input_) (l,j) * kernel [k];
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();          
      }      
      for (int i = last; i < width; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_rows_duplicate ( PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->width - half_width_;
  int w = last - 1;
  Eigen::ArrayXf kernel = kernel_;
  
  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (l,j) * kernel [k];
      }
      
      for (int i = last; i < width; ++i)
        output (i,j) = output (w, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_, j);      
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        int counter = 0;
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (l,j)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
          {
            output (i,j)+= (*input_) (l,j) * kernel [k];
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();          
      }
      
      for (int i = last; i < width; ++i)
        output (i,j) = output (w, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_, j);
    }
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_rows_mirror (PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->width - half_width_;
  int w = last - 1;
  Eigen::ArrayXf kernel = kernel_;
  
  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (l,j) * kernel [k];
      }
      
      for (int i = last, l = 0; i < width; ++i, ++l)
        output (i,j) = output (w-l, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_+1-i, j);      
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        int counter = 0;
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (l,j)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
          {
            output (i,j)+= (*input_) (l,j) * kernel [k];
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();          
      }
      
      for (int i = last, l = 0; i < width; ++i, ++l)
        output (i,j) = output (w-l, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_+1-i, j);
    }
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_cols (PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->height - half_width_;
  Eigen::ArrayXf kernel = kernel_;
  
  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = 0; j < half_width_; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (i,l) * kernel [k];
      }
      
      for (int j = last; j < height; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = 0; j < half_width_; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
     
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = PointT ();
        int counter = 0;
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (i,l)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
          {
            output (i,j)+= (*input_) (i,l) * kernel [k];
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      }
      
      for (int j = last; j < height; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_cols_duplicate (PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->height - half_width_;
  int h = last -1;
  Eigen::ArrayXf kernel = kernel_;
  
  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (i,l) * kernel [k];
      }
      
      for (int j = last; j < height; ++j)
        output (i,j) = output (i,h);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_);
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = PointT ();
        int counter = 0;
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (i,l)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
          {
            output (i,j)+= (*input_) (i,l) * kernel [k];
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      }
      
      for (int j = last; j < height; ++j)
        output (i,j) = output (i,h);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i,half_width_);
    }
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_cols_mirror (PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->height - half_width_;
  int h = last -1;
  Eigen::ArrayXf kernel = kernel_;
  
  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (i,l) * kernel [k];
      }
      
      for (int j = last, l = 0; j < height; ++j, ++l)
        output (i,j) = output (i,h-l);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_+1-j);
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = PointT ();
        int counter = 0;
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (i,l)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
          {
            output (i,j)+= (*input_) (i,l) * kernel [k];
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      }
      
      for (int j = last, l = 0; j < height; ++j, ++l)
        output (i,j) = output (i,h-l);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i,half_width_+1-j);
    }
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_rows (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->width - half_width_;
  Eigen::ArrayXf kernel = kernel_;

  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = 0; i < half_width_; ++i)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      
      for (int i = half_width_; i < last; ++i)
      {
        output (i,j) = transform_ (0);
        for (int k = kernel_width_ - 1, l = i - half_width_; k > -1; --k, ++l)
          output (i,j)+= transform_ ((*input_) (l,j) * kernel [k]);
      }
      
      for (int i = last; i < width; ++i)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = 0; i < half_width_; ++i)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      
      for (int i = half_width_; i < last; ++i)
      {
        int counter = 0;
        output (i,j) = transform_ (0);
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (l,j)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
          {
            output (i,j)+= transform_ ((*input_) (l,j) * kernel [k]);
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      }

      for (int i = last; i < width; ++i)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
    }
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_rows_duplicate (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->width - half_width_;
  int w = last -1;
  Eigen::ArrayXf kernel = kernel_;

  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        output (i,j) = transform_ (0);
        for (int k = kernel_width_ - 1, l = i - half_width_; k > -1; --k, ++l)
          output (i,j)+= transform_ ((*input_) (l,j) * kernel [k]);
      }
      
      for (int i = last; i < width; ++i)
        output (i,j) = output (w,j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_,j);
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        int counter = 0;
        output (i,j) = transform_ (0);
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (l,j)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
          {
            output (i,j)+= transform_ ((*input_) (l,j) * kernel [k]);
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      }

      for (int i = last; i < width; ++i)
        output (i,j) = output (w,j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_,j);
    }
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_rows_mirror (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->width - half_width_;
  int w = last - 1;
  Eigen::ArrayXf kernel = kernel_;

  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        output (i,j) = transform_ (0);
        for (int k = kernel_width_ - 1, l = i - half_width_; k > -1; --k, ++l)
          output (i,j)+= transform_ ((*input_) (l,j) * kernel [k]);
      }
      
      for (int i = last, l = 0; i < width; ++i, ++l)
        output (i,j) = output (w-l, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_+1-i, j);
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int j = 0; j < height; ++j)
    {
      for (int i = half_width_; i < last; ++i)
      {
        int counter = 0;
        output (i,j) = transform_ (0);
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (l,j)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
          {
            output (i,j)+= transform_ ((*input_) (l,j) * kernel [k]);
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      }

      for (int i = last, l = 0; i < width; ++i, ++l)
        output (i,j) = output (w-l, j);

      for (int i = 0; i < half_width_; ++i)
        output (i,j) = output (half_width_+1-i, j);
    }
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_cols (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->height - half_width_;
  Eigen::ArrayXf kernel = kernel_;

  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = 0; j < half_width_; ++j)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = transform_ (0);
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
          output (i,j)+= transform_ ((*input_) (i,l) * kernel [k]);
      }
      
      for (int j = last; j < height; ++j)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = 0; j < half_width_; ++j)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
     
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = transform_ (0);
        int counter = 0;
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (i,l)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
          {
            output (i,j)+= transform_ ((*input_) (i,l) * kernel [k]);
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      }
      
      for (int j = last; j < height; ++j)
        output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
    }
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_cols_duplicate (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->height - half_width_;
  int h = last - 1;
  Eigen::ArrayXf kernel = kernel_;

  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = transform_ (0);
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
          output (i,j)+= transform_ ((*input_) (i,l) * kernel [k]);
      }

      for (int j = last; j < height; ++j)
        output (i,j) = output (i,h);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_);
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = transform_ (0);
        int counter = 0;
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (i,l)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
          {
            output (i,j)+= transform_ ((*input_) (i,l) * kernel [k]);
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      }

      for (int j = last; j < height; ++j)
        output (i,j) = output (i,h);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_);
    }
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_cols_mirror (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
	int width = input_->width;
	int height = input_->height;
  int last = input_->height - half_width_;
  Eigen::ArrayXf kernel = kernel_;
  int h = last - 1;
  if (input_->is_dense)
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = transform_ (0);
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
          output (i,j)+= transform_ ((*input_) (i,l) * kernel [k]);
      }
      
      for (int j = last, l = 0; j < height; ++j, ++l)
        output (i,j) = output (i,h-l);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_+1-j);
    }
  }
  else
  {
#pragma omp parallel for shared (output, kernel, kernel_width_, last) num_threads (threads_)
    for(int i = 0; i < width; ++i)
    {
      for (int j = half_width_; j < last; ++j)
      {
        output (i,j) = transform_ (0);
        int counter = 0;
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        {
          if (!isFinite ((*input_) (i,l)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
          {
            output (i,j)+= transform_ ((*input_) (i,l) * kernel [k]);
            ++counter;
          }
        }
        if (counter == 0)
          output (i,j) = transform_ (std::numeric_limits<float>::quiet_NaN ());
      }

      for (int j = last, l = 0; j < height; ++j, ++l)
        output (i,j) = output (i,h-l);

      for (int j = 0; j < half_width_; ++j)
        output (i,j) = output (i, half_width_+1-j);
    }
  }
}
#endif
