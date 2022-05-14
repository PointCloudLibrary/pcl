/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_IMPL_PASSTHROUGH_HPP_
#define PCL_FILTERS_IMPL_PASSTHROUGH_HPP_

#include <pcl/filters/passthrough.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PassThrough<PointT>::applyFilterIndices (Indices &indices)
{
  // The arrays to be used
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  // Has a field name been specified?
  if (filter_field_name_.empty ())
  {
    // Only filter for non-finite entries then
    for (const auto ii : *indices_)  // ii = input index
    {
      // Non-finite entries are always passed to removed indices
      if (!std::isfinite ((*input_)[ii].x) ||
          !std::isfinite ((*input_)[ii].y) ||
          !std::isfinite ((*input_)[ii].z))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = ii;
        continue;
      }
      indices[oii++] = ii;
    }
  }
  else
  {
    // Attempt to get the field name's index
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex<PointT> (filter_field_name_, fields);
    if (distance_idx == -1)
    {
      PCL_WARN ("[pcl::%s::applyFilter] Unable to find field name in point type.\n", getClassName ().c_str ());
      indices.clear ();
      removed_indices_->clear ();
      return;
    }

    // Filter for non-finite entries and the specified field limits
    for (const auto ii : *indices_)  // ii = input index
    {
      // Non-finite entries are always passed to removed indices
      if (!std::isfinite ((*input_)[ii].x) ||
          !std::isfinite ((*input_)[ii].y) ||
          !std::isfinite ((*input_)[ii].z))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = ii;
        continue;
      }

      // Get the field's value
      const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&(*input_)[ii]);
      float field_value = 0;
      memcpy (&field_value, pt_data + fields[distance_idx].offset, sizeof (float));

      // Remove NAN/INF/-INF values. We expect passthrough to output clean valid data.
      if (!std::isfinite (field_value))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = ii;
        continue;
      }

      // Outside of the field limits are passed to removed indices
      if (!negative_ && (field_value < filter_limit_min_ || field_value > filter_limit_max_))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = ii;
        continue;
      }

      // Inside of the field limits are passed to removed indices if negative was set
      if (negative_ && field_value >= filter_limit_min_ && field_value <= filter_limit_max_)
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = ii;
        continue;
      }

      // Otherwise it was a normal point for output (inlier)
      indices[oii++] = ii;
    }
  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_PassThrough(T) template class PCL_EXPORTS pcl::PassThrough<T>;

#endif  // PCL_FILTERS_IMPL_PASSTHROUGH_HPP_

