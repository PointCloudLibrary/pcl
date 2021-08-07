/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_FILTERS_IMPL_NORMAL_SPACE_SAMPLE_H_
#define PCL_FILTERS_IMPL_NORMAL_SPACE_SAMPLE_H_

#include <pcl/filters/normal_space.h>

#include <vector>
#include <list>

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> bool
pcl::NormalSpaceSampling<PointT, NormalT>::initCompute ()
{
  if (!FilterIndices<PointT>::initCompute ())
    return false;

  // If sample size is 0 or if the sample size is greater then input cloud size then return entire copy of cloud
  if (sample_ >= input_->size ())
  {
    PCL_ERROR ("[NormalSpaceSampling::initCompute] Requested more samples than the input cloud size: %d vs %lu\n",
               sample_, input_->size ());
    return false;
  }

  rng_.seed (seed_);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> bool 
pcl::NormalSpaceSampling<PointT, NormalT>::isEntireBinSampled (boost::dynamic_bitset<> &array,
                                                               unsigned int start_index,
                                                               unsigned int length)
{
  bool status = true;
  for (unsigned int i = start_index; i < start_index + length; i++)
  {
    status &= array.test (i);
  }
  return status;
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> unsigned int 
pcl::NormalSpaceSampling<PointT, NormalT>::findBin (const float *normal)
{
  const unsigned ix = static_cast<unsigned> (std::round (0.5f * (binsx_ - 1.f) * (normal[0] + 1.f)));
  const unsigned iy = static_cast<unsigned> (std::round (0.5f * (binsy_ - 1.f) * (normal[1] + 1.f)));
  const unsigned iz = static_cast<unsigned> (std::round (0.5f * (binsz_ - 1.f) * (normal[2] + 1.f)));
  return ix * (binsy_*binsz_) + iy * binsz_ + iz;
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
pcl::NormalSpaceSampling<PointT, NormalT>::applyFilter (Indices &indices)
{
  if (!initCompute ())
  {
    indices = *indices_;
    return;
  }

  unsigned int max_values = (std::min) (sample_, static_cast<unsigned int> (input_normals_->size ()));
  // Resize output indices to sample size
  indices.resize (max_values);
  removed_indices_->resize (max_values);
  
  // Allocate memory for the histogram of normals. Normals will then be sampled from each bin.
  unsigned int n_bins = binsx_ * binsy_ * binsz_;
  // list<int> holds the indices of points in that bin. Using list to avoid repeated resizing of vectors.
  // Helps when the point cloud is large.
  std::vector<std::list <int> > normals_hg;
  normals_hg.reserve (n_bins);
  for (unsigned int i = 0; i < n_bins; i++)
    normals_hg.emplace_back();

  for (const auto index : *indices_)
  {
    unsigned int bin_number = findBin ((*input_normals_)[index].normal);
    normals_hg[bin_number].push_back (index);
  }


  // Setting up random access for the list created above. Maintaining the iterators to individual elements of the list
  // in a vector. Using vector now as the size of the list is known.
  std::vector<std::vector<std::list<int>::iterator> > random_access (normals_hg.size ());
  for (std::size_t i = 0; i < normals_hg.size (); i++)
  {
    random_access.emplace_back();
    random_access[i].resize (normals_hg[i].size ());

    std::size_t j = 0;
    for (std::list<int>::iterator itr = normals_hg[i].begin (); itr != normals_hg[i].end (); ++itr, ++j)
      random_access[i][j] = itr;
  }
  std::vector<std::size_t> start_index (normals_hg.size ());
  start_index[0] = 0;
  std::size_t prev_index = 0;
  for (std::size_t i = 1; i < normals_hg.size (); i++)
  {
    start_index[i] = prev_index + normals_hg[i-1].size ();
    prev_index = start_index[i];
  }

  // Maintaining flags to check if a point is sampled
  boost::dynamic_bitset<> is_sampled_flag (input_normals_->size ());
  // Maintaining flags to check if all points in the bin are sampled
  boost::dynamic_bitset<> bin_empty_flag (normals_hg.size ());
  unsigned int i = 0;
  while (i < sample_)
  {
    // Iterating through every bin and picking one point at random, until the required number of points are sampled.
    for (std::size_t j = 0; j < normals_hg.size (); j++)
    {
      unsigned int M = static_cast<unsigned int> (normals_hg[j].size ());
      if (M == 0 || bin_empty_flag.test (j)) // bin_empty_flag(i) is set if all points in that bin are sampled..
        continue;

      unsigned int pos = 0;
      unsigned int random_index = 0;
      std::uniform_int_distribution<unsigned> rng_uniform_distribution (0u, M - 1u);

      // Picking up a sample at random from jth bin
      do
      {
        random_index = rng_uniform_distribution (rng_);
        pos = start_index[j] + random_index;
      } while (is_sampled_flag.test (pos));

      is_sampled_flag.flip (start_index[j] + random_index);

      // Checking if all points in bin j are sampled.
      if (isEntireBinSampled (is_sampled_flag, start_index[j], static_cast<unsigned int> (normals_hg[j].size ()))) 
        bin_empty_flag.flip (j);

      unsigned int index = *(random_access[j][random_index]);
      indices[i] = index;
      i++;
      if (i == sample_)
        break;
    }
  }
  
  // If we need to return the indices that we haven't sampled
  if (extract_removed_indices_)
  {
    Indices indices_temp = indices;
    std::sort (indices_temp.begin (), indices_temp.end ());

    Indices all_indices_temp = *indices_;
    std::sort (all_indices_temp.begin (), all_indices_temp.end ());
    set_difference (all_indices_temp.begin (), all_indices_temp.end (), 
                    indices_temp.begin (), indices_temp.end (), 
                    inserter (*removed_indices_, removed_indices_->begin ()));
  }
}

#define PCL_INSTANTIATE_NormalSpaceSampling(T,NT) template class PCL_EXPORTS pcl::NormalSpaceSampling<T,NT>;

#endif    // PCL_FILTERS_IMPL_NORMAL_SPACE_SAMPLE_H_
