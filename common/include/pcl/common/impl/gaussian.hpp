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

#pragma once

#include <pcl/common/gaussian.h>

namespace pcl
{

template <typename PointT> void
GaussianKernel::convolveRows(const pcl::PointCloud<PointT> &input,
                             std::function <float (const PointT& p)> field_accessor,
                             const Eigen::VectorXf& kernel,
                             pcl::PointCloud<float> &output) const
{
  assert(kernel.size () % 2 == 1);
  int kernel_width = kernel.size () -1;
  int radius = kernel.size () / 2.0;
  if(output.height < input.height || output.width < input.width)
  {
    output.width = input.width;
    output.height = input.height;
    output.resize (input.height * input.width);
  }

  int i;
  for(int j = 0; j < input.height; j++)
  {
    for (i = 0 ; i < radius ; i++)
      output (i,j) = 0;

    for ( ; i < input.width - radius ; i++)  {
      output (i,j) = 0;
      for (int k = kernel_width, l = i - radius; k >= 0 ; k--, l++)
        output (i,j) += field_accessor (input (l,j)) * kernel[k];
    }

    for ( ; i < input.width ; i++)
      output (i,j) = 0;
  }
}

template <typename PointT> void
GaussianKernel::convolveCols(const pcl::PointCloud<PointT> &input,
                             std::function <float (const PointT& p)> field_accessor,
                             const Eigen::VectorXf& kernel,
                             pcl::PointCloud<float> &output) const
{
  assert(kernel.size () % 2 == 1);
  int kernel_width = kernel.size () -1;
  int radius = kernel.size () / 2.0;
  if(output.height < input.height || output.width < input.width)
  {
    output.width = input.width;
    output.height = input.height;
    output.resize (input.height * input.width);
  }

  int j;
  for(int i = 0; i < input.width; i++)
  {
    for (j = 0 ; j < radius ; j++)
      output (i,j) = 0;

    for ( ; j < input.height - radius ; j++)  {
      output (i,j) = 0;
      for (int k = kernel_width, l = j - radius ; k >= 0 ; k--, l++)
      {
        output (i,j) += field_accessor (input (i,l)) * kernel[k];
      }
    }

    for ( ; j < input.height ; j++)
      output (i,j) = 0;
  }
}

} // namespace pcl

