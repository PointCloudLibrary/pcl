/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: extract_indices.cpp 1385 2011-06-19 19:15:56Z rusu $
 *
 */

#include <pcl/filters/impl/random_sample.hpp>


///////////////////////////////////////////////////////////////////////////////
bool
pcl::RandomSample<sensor_msgs::PointCloud2>::initCompute ()
{
  rng_.seed (seed_);
  return (FilterIndices<sensor_msgs::PointCloud2>::initCompute ());
}

///////////////////////////////////////////////////////////////////////////////
void
pcl::RandomSample<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &output)
{
  unsigned N = input_->width * input_->height;
  // If sample size is 0 or if the sample size is greater then input cloud size
  //   then return entire copy of cloud
  if (sample_ >= N)
  {
    output = *input_;
  }
  else
  {
    // Resize output cloud to sample size
    output.data.resize (sample_ * input_->point_step);

    // Copy the common fields
    output.fields = input_->fields;
    output.is_bigendian = input_->is_bigendian;
    output.row_step = input_->row_step;
    output.point_step = input_->point_step;
    output.height = 1;

    // Algorithm A
    float one_over_N = 0.f;
    float top = 0.f;
    size_t index = 0;
    std::vector<bool> added;
    if (extract_removed_indices_)
      added.resize (indices_->size (), false);
    size_t i = 0;

    for (size_t n = sample_; n > 1; n--)
    {
      top = N - n; // N are the remaining number of elements, n the remaining number of wanted samples
      one_over_N = 1.f / static_cast<float> (N); //we need to re-calculate N^{-1}

      float V = unifRand ();
      size_t S = 0;
      float quot = top * one_over_N;

      while( quot > V )
      {
        S ++;
        N --;
        quot = quot * (top * one_over_N);
      }

      N--; // this together with N-- above is the same than N - S - 1 (paper Vit84)
      index += S;

      memcpy (&output.data[i * output.point_step], &input_->data[index * output.point_step], output.point_step);

      i ++;
      index ++;
    }
  }

  output.width = sample_;
  output.row_step = output.point_step * output.width;
}

///////////////////////////////////////////////////////////////////////////////
void
pcl::RandomSample<sensor_msgs::PointCloud2>::applyFilter (std::vector<int> &indices)
{
  unsigned N = input_->width * input_->height;
  // If sample size is 0 or if the sample size is greater then input cloud size
  //   then return all indices
  if (sample_ >= N)
  {
    indices = *indices_;
  }
  else
  {
    // Resize output indices to sample size
    indices.resize (sample_);

    // Algorithm A
    float one_over_N = 0.f;
    float top = 0.f;
    size_t index = 0;
    std::vector<bool> added;
    if (extract_removed_indices_)
      added.resize (indices_->size (), false);
    size_t i = 0;

    for (size_t n = sample_; n > 1; n--)
    {
      top = N - n; // N are the remaining number of elements, n the remaining number of wanted samples
      one_over_N = 1.f / static_cast<float> (N); //we need to re-calculate N^{-1}

      float V = unifRand ();
      size_t S = 0;
      float quot = top * one_over_N;

      while( quot > V )
      {
        S ++;
        N --;
        quot = quot * (top * one_over_N);
      }

      N--; // this together with N-- above is the same than N - S - 1 (paper Vit84)
      index += S;

      if (extract_removed_indices_)
        added[index] = true;
      indices[i] = (*indices_)[index];

      i ++;
      index ++;
    }
  }
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

  PCL_INSTANTIATE(RandomSample, PCL_POINT_TYPES)

    #endif    // PCL_NO_PRECOMPILE

