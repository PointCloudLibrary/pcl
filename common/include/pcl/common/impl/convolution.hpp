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
pcl::common::Convolution<PointIn, PointOut>::initCompute ()
{
  if(kernel_width_ % 2 == 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::Convolution::initCompute] convolving element width must be odd.");

  half_width_ = kernel_width_ / 2;

  switch (borders_policy_)
  {
    case IGNORE : break;
    case MIRROR :
    {
      spring_.setInputCloud (input_);
      spring_.setAmount (half_width_);
      spring_.setExpandPolicy (borders_policy_);
      spring_.setDirection (convolve_direction_);
      spring_.expand ();
    } break;
    case DUPLICATE :
    {
      spring_.setInputCloud (input_);
      spring_.setAmount (half_width_);
      spring_.setExpandPolicy (borders_policy_);
      spring_.setDirection (convolve_direction_);
      spring_.expand ();
    } break;
    default: 
      PCL_THROW_EXCEPTION (InitFailedException,
                           "[pcl::common::Convolution::initCompute] unknown border policy");
      break;
  }

  if (&(*input_) != &(*output_))
  {
    if (output_->height < input_->height || output_->width < input_->width)
    {
      output_->resize (input_->width * input_->height);
      output_->width = input_->width;
      output_->height = input_->height;
    }
  }
  else
    input_.reset (new PointCloudOut (*input_));
}

template <typename PointIn, typename PointOut> inline void
pcl::common::Convolution<PointIn, PointOut>::convolve_rows (const Eigen::ArrayXf& kernel, 
                                                              PointCloudOutPtr& output)
{
  int i, h(input_->height), w(input_->width), last(w - half_width_);
  for(int j = 0; j < h; j++)
  {
    for (i = 0; i < half_width_; i++)
      (*output_) (i,j) = operators_ ();
    
    for ( ; i < last; i++)  {
      (*output_) (i,j) = operators_ ();
      for (int k = kernel_width_ - 1, l = i - half_width_; k >= 0 ; k--, l++)
        operators_.plus_assign ((*output_) (i,j), 
                                operators_.dot (operators_ ((*input_) (l,j)), kernel [k]));
    }
    
    for ( ; i < w; i++)
      (*output_) (i,j) = operators_ ();
  }
}

template <typename PointIn, typename PointOut> inline void
pcl::common::Convolution<PointIn, PointOut>::convolve_cols (const Eigen::ArrayXf& kernel, 
                                                              PointCloudOutPtr& output)
{
  int j, h(input_->height), w(input_->width), last(h - half_width_);
  for(int i = 0; i < w; i++)
  {
    for (j = 0; j < half_width_; j++)
      (*output_) (i,j) = operators_ ();
    
    for ( ; j < last; j++)  {
      (*output_) (i,j) = operators_ ();
      for (int k = kernel_width_, l = j - half_width_; k >= 0; k--, l++)
      {
        operators_.plus_assign ((*output_) (i,j), 
                                operators_.dot (operators_ ((*input_) (i,l)), kernel [k]));
      }
    }
    
    for ( ; j < h; j++)
      (*output_) (i,j) = operators_ ();
  }
}

#endif
