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
  assert (input_normals_ != NULL);
  output.points.resize (input_->points.size ());
  if (extract_removed_indices_)
    removed_indices_->resize (input_->points.size ());

  unsigned int cp = 0;
  unsigned int ri = 0;
  for (unsigned int i = 0; i < input_->points.size (); i++)
  {
    const NormalT &normal = input_normals_->points[i];
    const PointT &pt = input_->points[i];
    float val = fabsf (normal.normal_x * pt.x + normal.normal_y * pt.y + normal.normal_z * pt.z);

    if ( (val >= threshold_) ^ negative_)
      output.points[cp++] = pt;
    else 
    {
      if (extract_removed_indices_)
        (*removed_indices_)[ri++] = i;
      if (keep_organized_)
      {
        PointT &pt_new = output.points[cp++] = pt;
        pt_new.x = pt_new.y = pt_new.z = user_filter_value_;
      }

    }  
  }
  output.points.resize (cp);
  removed_indices_->resize (ri);
  output.width = 1;
  output.height = static_cast<uint32_t> (output.points.size ());
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
pcl::ShadowPoints<PointT, NormalT>::applyFilter (std::vector<int> &indices)
{
  assert (input_normals_ != NULL);
  indices.resize (input_->points.size ());
  if (extract_removed_indices_)
    removed_indices_->resize (indices_->size ());

  unsigned int k = 0;
  unsigned int z = 0;
  for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
  {
    const NormalT &normal = input_normals_->points[*idx];
    const PointT &pt = input_->points[*idx];
    
    float val = fabsf (normal.normal_x * pt.x + normal.normal_y * pt.y + normal.normal_z * pt.z);

    if ( (val >= threshold_) ^ negative_)
      indices[k++] = *idx;
    else if (extract_removed_indices_)
      (*removed_indices_)[z++] = *idx;
  }
  indices.resize (k);
  removed_indices_->resize (z);
}

#define PCL_INSTANTIATE_ShadowPoints(T,NT) template class PCL_EXPORTS pcl::ShadowPoints<T,NT>;

#endif    // PCL_FILTERS_IMPL_NORMAL_SPACE_SAMPLE_H_
