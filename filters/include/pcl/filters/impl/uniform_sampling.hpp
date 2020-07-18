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
 * $Id$
 *
 */

#ifndef PCL_FILTERS_UNIFORM_SAMPLING_IMPL_H_
#define PCL_FILTERS_UNIFORM_SAMPLING_IMPL_H_

#include <pcl/common/common.h>
#include <pcl/filters/uniform_sampling.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::UniformSampling<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::detectKeypoints] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  output.height       = 1;                    // downsampling breaks the organized structure
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  pcl::getMinMax3D<PointT>(*input_, min_p, max_p);

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (std::floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (std::floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (std::floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (std::floor (max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int> (std::floor (min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int> (std::floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  // Clear the leaves
  leaves_.clear ();

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

  removed_indices_->clear();
  // First pass: build a set of leaves with the point index closest to the leaf center
  for (std::size_t cp = 0; cp < indices_->size (); ++cp)
  {
    if (!input_->is_dense)
    {
      // Check if the point is invalid
      if (!std::isfinite ((*input_)[(*indices_)[cp]].x) || 
          !std::isfinite ((*input_)[(*indices_)[cp]].y) || 
          !std::isfinite ((*input_)[(*indices_)[cp]].z))
      {
        if (extract_removed_indices_)
          removed_indices_->push_back ((*indices_)[cp]);
        continue;
      }
    }

    Eigen::Vector4i ijk = Eigen::Vector4i::Zero ();
    ijk[0] = static_cast<int> (std::floor ((*input_)[(*indices_)[cp]].x * inverse_leaf_size_[0]));
    ijk[1] = static_cast<int> (std::floor ((*input_)[(*indices_)[cp]].y * inverse_leaf_size_[1]));
    ijk[2] = static_cast<int> (std::floor ((*input_)[(*indices_)[cp]].z * inverse_leaf_size_[2]));

    // Compute the leaf index
    int idx = (ijk - min_b_).dot (divb_mul_);
    Leaf& leaf = leaves_[idx];
    // First time we initialize the index
    if (leaf.idx == -1)
    {
      leaf.idx = (*indices_)[cp];
      continue;
    }

    // Check to see if this point is closer to the leaf center than the previous one we saved
    float diff_cur   = ((*input_)[(*indices_)[cp]].getVector4fMap () - ijk.cast<float> ()).squaredNorm ();
    float diff_prev  = ((*input_)[leaf.idx].getVector4fMap ()        - ijk.cast<float> ()).squaredNorm ();

    // If current point is closer, copy its index instead
    if (diff_cur < diff_prev)
    {
      if (extract_removed_indices_)
        removed_indices_->push_back (leaf.idx);
      leaf.idx = (*indices_)[cp];
    }
    else
    {
      if (extract_removed_indices_)
        removed_indices_->push_back ((*indices_)[cp]);
    }
  }

  // Second pass: go over all leaves and copy data
  output.resize (leaves_.size ());
  int cp = 0;

  for (const auto& leaf : leaves_)
    output[cp++] = (*input_)[leaf.second.idx];
  output.width = output.size ();
}

#define PCL_INSTANTIATE_UniformSampling(T) template class PCL_EXPORTS pcl::UniformSampling<T>;

#endif    // PCL_FILTERS_UNIFORM_SAMPLING_IMPL_H_

