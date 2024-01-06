/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_IMPL_SHADOW_POINTS_FILTER_H_
#define PCL_FILTERS_IMPL_SHADOW_POINTS_FILTER_H_

#include <pcl/filters/shadowpoints.h>

#include <vector>

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
pcl::ShadowPoints<PointT, NormalT>::applyFilter (PointCloud &output)
{
  assert (input_normals_ != nullptr);
  output.resize (input_->size ());
  if (extract_removed_indices_)
    removed_indices_->resize (input_->size ());

  std::size_t cp = 0;
  std::size_t ri = 0;
  for (std::size_t i = 0; i < input_->size (); i++)
  {
    const NormalT &normal = (*input_normals_)[i];
    const PointT &pt = (*input_)[i];
    const float val = std::abs (normal.normal_x * pt.x + normal.normal_y * pt.y + normal.normal_z * pt.z);

    if ( (val >= threshold_) ^ negative_)
      output[cp++] = pt;
    else 
    {
      if (extract_removed_indices_)
        (*removed_indices_)[ri++] = i;
      if (keep_organized_)
      {
        PointT &pt_new = output[cp++] = pt;
        pt_new.x = pt_new.y = pt_new.z = user_filter_value_;
      }

    }  
  }
  output.resize (cp);
  removed_indices_->resize (ri);
  output.height = 1;
  output.width = output.size ();
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
pcl::ShadowPoints<PointT, NormalT>::applyFilter (Indices &indices)
{
  assert (input_normals_ != nullptr);
  indices.resize (input_->size ());
  if (extract_removed_indices_)
    removed_indices_->resize (indices_->size ());

  unsigned int k = 0;
  unsigned int z = 0;
  for (const auto& idx : (*indices_))
  {
    const NormalT &normal = (*input_normals_)[idx];
    const PointT &pt = (*input_)[idx];
    
    float val = std::abs (normal.normal_x * pt.x + normal.normal_y * pt.y + normal.normal_z * pt.z);

    if ( (val >= threshold_) ^ negative_)
      indices[k++] = idx;
    else if (extract_removed_indices_)
      (*removed_indices_)[z++] = idx;
  }
  indices.resize (k);
  removed_indices_->resize (z);
}

#define PCL_INSTANTIATE_ShadowPoints(T,NT) template class PCL_EXPORTS pcl::ShadowPoints<T,NT>;

#endif    // PCL_FILTERS_IMPL_NORMAL_SPACE_SAMPLE_H_
