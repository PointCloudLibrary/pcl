/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: extract_indices.cpp 1385 2011-06-19 19:15:56Z rusu $
 *
 */

#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/random_sample.h"
#include "pcl/filters/impl/random_sample.hpp"

///////////////////////////////////////////////////////////////////////////////
void
pcl::RandomSample<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &output)
{
  // If sample size is 0 or if the sample size is greater then input cloud size
  //   then return entire copy of cloud
  if (sample_ >= input_->width * input_->height)
  {
    output = *input_;
  }
  else
  {
    // Resize output cloud to sample size
    output.data.resize (sample_ * input_->point_step);

    // Copy the common fields
    output.is_bigendian = input_->is_bigendian;
    output.point_step = input_->point_step;
    output.height = 1;

    // Fill output cloud with first sample points from input cloud
    for (size_t i = 0; i < sample_; ++i)
      output.data[i * output.point_step] = input_->data[i * input_->point_step];

    // Set random seed so derived indices are the same each time the filter runs
    std::srand (seed_);

    // Iterate over the rest of the points of the input cloud picking a random
    //   index from the output indices to replace
    for (size_t i = sample_; i < (*indices_).size (); ++i)
    {
      size_t index = std::rand () % (i + 1);
      if (index < sample_)
        output.data[index * output.point_step] = input_->data[i * input_->point_step];
    }
    output.width = sample_;
    output.row_step = output.point_step * output.width;
  }
}

///////////////////////////////////////////////////////////////////////////////
void
pcl::RandomSample<sensor_msgs::PointCloud2>::applyFilter (std::vector<int> &indices)
{
  // If sample size is 0 or if the sample size is greater then input cloud size
  //   then return all indices
  if (sample_ >= input_->width * input_->height)
  {
    indices = *indices_;
  }
  else
  {
    // Resize output indices to sample size
    indices.resize (sample_);

    // Fill output indices with first sample indices from input cloud
    for (size_t i = 0; i < indices.size (); ++i)
      indices[i] = (*indices_)[i];

    // Set random seed so derived indices are the same each time the filter runs
    std::srand (seed_);

    // Iterate over the rest of the indices of the input cloud picking a random
    //   index from the output indices to replace
    for (size_t i = indices.size (); i < (*indices_).size (); ++i)
    {
      size_t index = std::rand () % (i + 1);
      if (index < indices.size ())
        indices[index] = (*indices_)[i];
    }
  }
}

PCL_INSTANTIATE(RandomSample, PCL_POINT_TYPES);
