/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id$
 *
 */


#ifndef PCL_FILTERS_IMPL_PYRAMID_HPP
#define PCL_FILTERS_IMPL_PYRAMID_HPP

#include <pcl/common/distances.h>
#include <pcl/filters/pyramid.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>

namespace pcl
{
namespace filters
{
template <typename PointT> bool
Pyramid<PointT>::initCompute ()
{
  if (!input_->isOrganized ())
  {
    PCL_ERROR ("[pcl::fileters::%s::initCompute] Number of levels should be at least 2!\n", getClassName ().c_str ());
    return (false);
  }

  if (levels_ < 2)
  {
    PCL_ERROR ("[pcl::fileters::%s::initCompute] Number of levels should be at least 2!\n", getClassName ().c_str ());
    return (false);
  }

  // std::size_t ratio (std::pow (2, levels_));
  // std::size_t last_width = input_->width / ratio;
  // std::size_t last_height = input_->height / ratio;

  if (levels_ > 4)
  {
    PCL_ERROR ("[pcl::fileters::%s::initCompute] Number of levels should not exceed 4!\n", getClassName ().c_str ());
    return (false);
  }

  if (large_)
  {
    Eigen::VectorXf k (5);
    k << 1.f/16.f, 1.f/4.f, 3.f/8.f, 1.f/4.f, 1.f/16.f;
    kernel_ = k * k.transpose ();
    if (threshold_ != std::numeric_limits<float>::infinity ())
      threshold_ *= 2 * threshold_;

  }
  else
  {
    Eigen::VectorXf k (3);
    k << 1.f/4.f, 1.f/2.f, 1.f/4.f;
    kernel_ = k * k.transpose ();
    if (threshold_ != std::numeric_limits<float>::infinity ())
      threshold_ *= threshold_;
  }

  return (true);
}

template <typename PointT> void
Pyramid<PointT>::compute (std::vector<PointCloudPtr>& output)
{
  std::cout << "compute" << std::endl;
  if (!initCompute ())
  {
    PCL_ERROR ("[pcl::%s::compute] initCompute failed!\n", getClassName ().c_str ());
    return;
  }

  int kernel_rows = static_cast<int> (kernel_.rows ());
  int kernel_cols = static_cast<int> (kernel_.cols ());
  int kernel_center_x = kernel_cols / 2;
  int kernel_center_y = kernel_rows / 2;

  output.resize (levels_ + 1);
  output[0].reset (new pcl::PointCloud<PointT>);
  *(output[0]) = *input_;

  if (input_->is_dense)
  {
    for (int l = 1; l <= levels_; ++l)
    {
      output[l].reset (new pcl::PointCloud<PointT> (output[l-1]->width/2, output[l-1]->height/2));
      const PointCloud<PointT> &previous = *output[l-1];
      PointCloud<PointT> &next = *output[l];
#pragma omp parallel for \
  default(none)          \
  shared(next)           \
  num_threads(threads_)
      for(int i=0; i < next.height; ++i)
      {
        for(int j=0; j < next.width; ++j)
        {
          for(int m=0; m < kernel_rows; ++m)
          {
            int mm = kernel_rows - 1 - m;
            for(int n=0; n < kernel_cols; ++n)
            {
              int nn = kernel_cols - 1 - n;

              int ii = 2*i + (m - kernel_center_y);
              int jj = 2*j + (n - kernel_center_x);

              if (ii < 0) ii = 0;
              if (ii >= previous.height) ii = previous.height - 1;
              if (jj < 0) jj = 0;
              if (jj >= previous.width) jj = previous.width - 1;
              next.at (j,i) += previous.at (jj,ii) * kernel_ (mm,nn);
            }
          }
        }
      }
    }
  }
  else
  {
    for (int l = 1; l <= levels_; ++l)
    {
      output[l].reset (new pcl::PointCloud<PointT> (output[l-1]->width/2, output[l-1]->height/2));
      const PointCloud<PointT> &previous = *output[l-1];
      PointCloud<PointT> &next = *output[l];
#pragma omp parallel for \
  default(none)          \
  shared(next)           \
  num_threads(threads_)
      for(int i=0; i < next.height; ++i)
      {
        for(int j=0; j < next.width; ++j)
        {
          float weight = 0;
          for(int m=0; m < kernel_rows; ++m)
          {
            int mm = kernel_rows - 1 - m;
            for(int n=0; n < kernel_cols; ++n)
            {
              int nn = kernel_cols - 1 - n;
              int ii = 2*i + (m - kernel_center_y);
              int jj = 2*j + (n - kernel_center_x);
              if (ii < 0) ii = 0;
              if (ii >= previous.height) ii = previous.height - 1;
              if (jj < 0) jj = 0;
              if (jj >= previous.width) jj = previous.width - 1;
              if (!isFinite (previous.at (jj,ii)))
                continue;
              if (pcl::squaredEuclideanDistance (previous.at (2*j,2*i), previous.at (jj,ii)) < threshold_)
              {
                next.at (j,i) += previous.at (jj,ii).x * kernel_ (mm,nn);
                weight+= kernel_ (mm,nn);
              }
            }
          }
          if (weight == 0)
            nullify (next.at (j,i));
          else
          {
            weight = 1.f/weight;
            next.at (j,i)*= weight;
          }
        }
      }
    }
  }
}

template <> void
Pyramid<pcl::PointXYZRGB>::compute (std::vector<Pyramid<pcl::PointXYZRGB>::PointCloudPtr> &output);

template <> void
Pyramid<pcl::PointXYZRGBA>::compute (std::vector<Pyramid<pcl::PointXYZRGBA>::PointCloudPtr> &output);

template<> void
Pyramid<pcl::RGB>::nullify (pcl::RGB& p)
{
  p.r = 0; p.g = 0; p.b = 0;
}

template <> void
Pyramid<pcl::RGB>::compute (std::vector<Pyramid<pcl::RGB>::PointCloudPtr> &output);

} // namespace filters
} // namespace pcl

#endif
