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
  if(kernel_.size () % 2 == 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::Convolution::initCompute] convolving element width must be odd.");

  switch (borders_policy_)
  {
    case IGNORE : break;
    case MIRROR : 
    {
      switch (convolve_direction_)
      {
        case HORIZONTAL : 
        {
          try 
          {
            mirrorRows (*input_, *input_, kernel_.size () / 2); 
          }
          catch (PCLException &e)
          {
            PCL_THROW_EXCEPTION (InitFailedException,
                                 "[pcl::common::Convolution::initCompute] expand failed");
          }
        } break;
        case VERTICAL : 
        {
          try 
          {
            mirrorColumns (*input_, *input_, kernel_.size () / 2); 
          }
          catch (PCLException &e)
          {
            PCL_THROW_EXCEPTION (InitFailedException,
                                 "[pcl::common::Convolution::initCompute] expand failed");
          }
        } break;
        default : 
        PCL_THROW_EXCEPTION (InitFailedException, 
                             "[pcl::common::Convolution::initCompute] invalid convolving direction!"); 
        break;
      }
    } break;
    case DUPLICATE :
    {
      switch (convolve_direction_)
      {
        case HORIZONTAL : 
        {
          try
          {
            duplicateRows (*input_, *input_, kernel_.size () / 2); 
          }
          catch (PCLException &e)
          {
            PCL_THROW_EXCEPTION (InitFailedException,
                                 "[pcl::common::Convolution::initCompute] expand failed");
          }
        } break;
        case VERTICAL : 
        {
          try 
          { 
            duplicateColumns (*input_, *input_, kernel_.size () / 2); 
          } 
          catch (PCLException &e)
          {
            PCL_THROW_EXCEPTION (InitFailedException,
                                 "[pcl::common::Convolution::initCompute] expand failed");
          }
        } break;
        default : 
          PCL_THROW_EXCEPTION (InitFailedException, 
                               "[pcl::common::Convolution::initCompute] invalid convolving direction!"); 
          break;
      }
    } break;
    default: 
      PCL_THROW_EXCEPTION (InitFailedException,
                           "[pcl::common::Convolution::initCompute] unknown border policy");
      break;
  }

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
  
  if (distance_threshold_ != std::numeric_limits<float>::infinity ())
    distance_threshold_*= (kernel_.size () % 2) * distance_threshold_;
}

template <typename PointIn, typename PointOut> void
pcl::common::AbstractConvolution<PointIn, PointOut>::deinitCompute (PointCloudOut& output)
{
  if (borders_policy_ == MIRROR || borders_policy_ == DUPLICATE)
  {
    if (convolve_direction_ == HORIZONTAL)
    { 
      try 
      {
        deleteRows (*input_, *input_, kernel_.size () / 2);
        deleteRows (output, output, kernel_.size () / 2);
      }
      catch (InitFailedException &e)
      {
        PCL_THROW_EXCEPTION (InitFailedException,
                             "[pcl::common::Convolution::deinitCompute] failed " << e.what ());
      }
    }
    else if (convolve_direction_ == VERTICAL)
    {
      try 
      {
        deleteCols (*input_, *input_, kernel_.size () / 2);
        deleteCols (output, output, kernel_.size () / 2);
      }
      catch (InitFailedException &e)
      {
        PCL_THROW_EXCEPTION (InitFailedException,
                             "[pcl::common::Convolution::deinitCompute] failed " << e.what ());
      }
    }
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_rows ( PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
  int i, h(input_->height), w(input_->width), last(w - half_width_);

  if (input_->is_dense)
  {
    for(int j = 0; j < h; ++j)
    {
      for (i = 0; i < half_width_; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      
      for ( ; i < last; ++i)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (l,j) * kernel_ [k];
      }
      
      for ( ; i < w; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
  else
  {
    for(int j = 0; j < h; ++j)
    {
      for (i = 0; i < half_width_; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      
      for ( ; i < last; ++i)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = i - half_width_; k > -1; --k, ++l)
        {
          // if (!isFinite ((*input_) (l,j)))
          // {
          //  output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
          //  break;
          // }
          if (!isFinite ((*input_) (l,j)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (l,j)) < distance_threshold_)
            output (i,j)+= (*input_) (l,j) * kernel_ [k];
        }
      }
      
      for ( ; i < w; ++i)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
}

template <typename PointT> void
pcl::common::Convolution<PointT>::convolve_cols (PointCloud<PointT>& output)
{
  using namespace pcl::common;
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
  int j, h (input_->height), w (input_->width), last (h - half_width_);

  if (input_->is_dense)
  {
    for(int i = 0; i < w; ++i)
    {
      for (j = 0; j < half_width_; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
      
      for ( ; j < last; ++j)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
          output (i,j)+= (*input_) (i,l) * kernel_ [k];
      }
      
      for ( ; j < h; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
  else
  {
    for(int i = 0; i < w; ++i)
    {
      for (j = 0; j < half_width_; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
     
      for ( ; j < last; ++j)
      {
        output (i,j) = PointT ();
        for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        {
          // if (!isFinite ((*input_) (i,l)))
          // {
          //  output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
          //  break;
          // }
          if (!isFinite ((*input_) (i,l)))
            continue;
          if (pcl::squaredEuclideanDistance ((*input_) (i,j), (*input_) (i,l)) < distance_threshold_)
            output (i,j)+= (*input_) (i,l) * kernel_ [k];
        }
      }
      
      for ( ; j < h; ++j)
        output (i,j).x = output (i,j).y = output (i,j).z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_rows (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
  int i, h(input_->height), w(input_->width), last(w - half_width_);
  for(int j = 0; j < h; ++j)
  {
    for (i = 0; i < half_width_; ++i)
      output (i,j) = transform_ ();
    
    for ( ; i < last; ++i)
    {
      output (i,j) = transform_ ();
      for (int k = kernel_width_ - 1, l = i - half_width_; k > -1; --k, ++l)
        output (i,j)+= transform_ ((*input_) (l,j) * kernel_ [k]);
    }
    
    for ( ; i < w; ++i)
      output (i,j) = transform_ ();
  }
}

template <typename PointInToPointOut> void
pcl::common::ConvolutionWithTransform<PointInToPointOut>::convolve_cols (PointCloudOut& output)
{
  int half_width_ = kernel_.size () / 2;
  int kernel_width_ = kernel_.size () - 1;
  int j, h(input_->height), w(input_->width), last(h - half_width_);
  for(int i = 0; i < w; ++i)
  {
    for (j = 0; j < half_width_; ++j)
      output (i,j) = transform_ ();
    
    for ( ; j < last; ++j)
    {
      output (i,j) = transform_ ();
      for (int k = kernel_width_, l = j - half_width_; k > -1; --k, ++l)
        output (i,j)+= transform_ ((*input_) (i,l) * kernel_ [k]);
    }
    
    for ( ; j < h; ++j)
      output (i,j) = transform_ ();
  }
}

#endif
