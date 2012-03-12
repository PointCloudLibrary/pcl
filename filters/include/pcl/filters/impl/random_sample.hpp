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
 * $Id: extract_indices.hpp 1897 2011-07-26 20:35:49Z rusu $
 *
 */

#ifndef PCL_FILTERS_IMPL_RANDOM_SAMPLE_H_
#define PCL_FILTERS_IMPL_RANDOM_SAMPLE_H_

#include <pcl/filters/random_sample.h>


///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::RandomSample<PointT>::applyFilter (PointCloud &output)
{
  unsigned N = static_cast<unsigned> (input_->size ());
  float one_over_N = 1.0f / float (N);

  // If sample size is 0 or if the sample size is greater then input cloud size
  //   then return entire copy of cloud
  if (sample_ >= N)
  {
    output = *input_;
  }
  else
  {
    // Resize output cloud to sample size
    output.points.resize (sample_);
    output.width = sample_;
    output.height = 1;

    // Set random seed so derived indices are the same each time the filter runs
    std::srand (seed_);

    unsigned top = N - sample_;
    unsigned i = 0;
    unsigned index = 0;

    // Algorithm A
    for (size_t n = sample_; n >= 2; n--)
    {
      unsigned int V = unifRand ();
      unsigned S = 0;
      float quot = float (top) * one_over_N;
      while (quot > V)
      {
        S++;
        top--;
        N--;
        quot = quot * float (top) * one_over_N;
      }
      index += S;
      output.points[i++] = input_->points[index++];
      N--;
    }

    index += N * static_cast<unsigned> (unifRand ());
    output.points[i++] = input_->points[index++];
  }
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
pcl::RandomSample<PointT>::applyFilter (std::vector<int> &indices)
{
  unsigned N = static_cast<unsigned> (input_->size ());
  float one_over_N = 1.0f / float (N);

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

    // Set random seed so derived indices are the same each time the filter runs
    std::srand (seed_);

    // Algorithm A
    unsigned top = N - sample_;
    unsigned i = 0;
    unsigned index = 0;

    for (size_t n = sample_; n >= 2; n--)
    {
      unsigned int V = unifRand ();
      unsigned S = 0;
      float quot = float (top) * one_over_N;
      while (quot > V)
      {
        S++;
        top--;
        N--;
        quot = quot * float (top) * one_over_N;
      }
      index += S;
      indices[i++] = (*indices_)[index++];
      N--;
    }

    index += N * static_cast<unsigned> (unifRand ());
    indices[i++] = (*indices_)[index++];
  }
}

#define PCL_INSTANTIATE_RandomSample(T) template class PCL_EXPORTS pcl::RandomSample<T>;

#endif    // PCL_FILTERS_IMPL_RANDOM_SAMPLE_H_
