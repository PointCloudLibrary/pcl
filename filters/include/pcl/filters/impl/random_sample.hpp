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
 * $Id: extract_indices.hpp 1897 2011-07-26 20:35:49Z rusu $
 *
 */

#ifndef PCL_FILTERS_IMPL_RANDOM_SAMPLE_H_
#define PCL_FILTERS_IMPL_RANDOM_SAMPLE_H_

#include <pcl/filters/random_sample.h>
#include <pcl/common/io.h>
#include <pcl/point_traits.h>


///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::RandomSample<PointT>::applyFilter (PointCloud &output)
{
  std::vector<int> indices;
  if (keep_organized_)
  {
    bool temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    applyFilter (indices);
    extract_removed_indices_ = temp;
    copyPointCloud (*input_, output);
    // Get X, Y, Z fields
    std::vector<pcl::PCLPointField> fields;
    pcl::getFields (*input_, fields);
    std::vector<size_t> offsets;
    for (size_t i = 0; i < fields.size (); ++i)
    {
      if (fields[i].name == "x" ||
          fields[i].name == "y" ||
          fields[i].name == "z")
        offsets.push_back (fields[i].offset);
    }
    // For every "removed" point, set the x,y,z fields to user_filter_value_
    const static float user_filter_value = user_filter_value_;
    for (size_t rii = 0; rii < removed_indices_->size (); ++rii)
    {
      uint8_t* pt_data = reinterpret_cast<uint8_t*> (&output[(*removed_indices_)[rii]]);
      for (size_t i = 0; i < offsets.size (); ++i)
      {
        memcpy (pt_data + offsets[i], &user_filter_value, sizeof (float));
      }
      if (!pcl_isfinite (user_filter_value_))
        output.is_dense = false;
    }
  }
  else
  {
    output.is_dense = true;
    applyFilter (indices);
    copyPointCloud (*input_, indices, output);
  }
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
pcl::RandomSample<PointT>::applyFilter (std::vector<int> &indices)
{
  unsigned N = static_cast<unsigned> (indices_->size ());
  
  unsigned int sample_size = negative_ ? N - sample_ : sample_;
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
    indices.resize (static_cast<size_t> (sample_size));
    if (extract_removed_indices_)
      removed_indices_->resize (static_cast<size_t> (N - sample_size));

    // Set random seed so derived indices are the same each time the filter runs
    std::srand (seed_);

    // Algorithm A
    unsigned top = N - sample_size;
    unsigned i = 0;
    unsigned index = 0;
    std::vector<bool> added;
    if (extract_removed_indices_)
      added.resize (indices_->size (), false);
    for (size_t n = sample_size; n >= 2; n--)
    {
      float V = unifRand ();
      unsigned S = 0;
      float quot = static_cast<float> (top) / static_cast<float> (N);
      while (quot > V)
      {
        S++;
        top--;
        N--;
        quot = quot * static_cast<float> (top) / static_cast<float> (N);
      }
      index += S;
      if (extract_removed_indices_)
        added[index] = true;
      indices[i++] = (*indices_)[index++];
      N--;
    }

    index += N * static_cast<unsigned> (unifRand ());
    if (extract_removed_indices_)
      added[index] = true;
    indices[i++] = (*indices_)[index++];

    // Now populate removed_indices_ appropriately
    if (extract_removed_indices_)
    {
      unsigned ri = 0;
      for (size_t i = 0; i < added.size (); i++)
      {
        if (!added[i])
        {
          (*removed_indices_)[ri++] = (*indices_)[i];
        }
      }
    }
  }
}

#define PCL_INSTANTIATE_RandomSample(T) template class PCL_EXPORTS pcl::RandomSample<T>;

#endif    // PCL_FILTERS_IMPL_RANDOM_SAMPLE_H_
