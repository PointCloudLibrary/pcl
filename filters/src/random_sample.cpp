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
void
pcl::RandomSample<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
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

    // Set random seed so derived indices are the same each time the filter runs
    std::srand (seed_);

    unsigned top = N - sample_;
    unsigned i = 0;
    unsigned index = 0;

    // Algorithm A
    for (std::size_t n = sample_; n >= 2; n--)
    {
      float V = unifRand ();
      unsigned S = 0;
      float quot = float (top) / float (N);
      while (quot > V)
      {
        S++;
        top--;
        N--;
        quot *= float (top) / float (N);
      }
      index += S;
      memcpy (&output.data[i++ * output.point_step], &input_->data[index++ * output.point_step], output.point_step);
      N--;
    }

    index += N * static_cast<unsigned> (unifRand ());
    memcpy (&output.data[i++ * output.point_step], &input_->data[index++ * output.point_step], output.point_step);

    output.width = sample_;
    output.row_step = output.point_step * output.width;
  }
}

///////////////////////////////////////////////////////////////////////////////
void
pcl::RandomSample<pcl::PCLPointCloud2>::applyFilter (std::vector<int> &indices)
{
  // Note: this function does not have to access input_ at all
  std::size_t N = indices_->size ();
  std::size_t sample_size = negative_ ? N - sample_ : sample_;
  // If sample size is 0 or if the sample size is greater then input cloud size
  //   then return all indices
  if (sample_size >= N)
  {
    indices = *indices_;
    removed_indices_->clear ();
  }
  else
  {
    // Resize output indices to sample size
    indices.resize (sample_size);
    if (extract_removed_indices_)
      removed_indices_->resize (N - sample_size);

    // Set random seed so derived indices are the same each time the filter runs
    std::srand (seed_);

    // Algorithm S
    std::size_t i = 0;
    std::size_t index = 0;
    std::vector<bool> added;
    if (extract_removed_indices_)
      added.resize (indices_->size (), false);
    std::size_t n = sample_size;
    while (n > 0)
    {
      // Step 1: [Generate U.] Generate a random variate U that is uniformly distributed between 0 and 1.
      const float U = unifRand ();
      // Step 2: [Test.] If N * U > n, go to Step 4.
      if ((N * U) <= n)
      {
        // Step 3: [Select.] Select the next record in the file for the sample, and set n : = n - 1.
        if (extract_removed_indices_)
          added[index] = true;
        indices[i++] = (*indices_)[index];
        --n;
      }
      // Step 4: [Don't select.] Skip over the next record (do not include it in the sample).
      // Set N : = N - 1.
      --N;
      ++index;
      // If n > 0, then return to Step 1; otherwise, the sample is complete and the algorithm terminates.
    }

    // Now populate removed_indices_ appropriately
    if (extract_removed_indices_)
    {
      std::size_t ri = 0;
      for (std::size_t i = 0; i < added.size (); i++)
      {
        if (!added[i])
        {
          (*removed_indices_)[ri++] = (*indices_)[i];
        }
      }
    }
  }
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

PCL_INSTANTIATE(RandomSample, PCL_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE

