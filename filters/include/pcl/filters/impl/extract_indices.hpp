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

#ifndef PCL_FILTERS_IMPL_EXTRACT_INDICES_HPP_
#define PCL_FILTERS_IMPL_EXTRACT_INDICES_HPP_

#include <pcl/filters/extract_indices.h>
#include <numeric> // for std::iota

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ExtractIndices<PointT>::filterDirectly (PointCloudPtr &cloud)
{
  Indices indices;
  bool temp = extract_removed_indices_;
  extract_removed_indices_ = true;
  this->setInputCloud (cloud);
  applyFilterIndices (indices);
  extract_removed_indices_ = temp;

  std::vector<pcl::PCLPointField> fields;
  pcl::for_each_type<FieldList> (pcl::detail::FieldAdder<PointT> (fields));
  for (const auto& rii : (*removed_indices_)) // rii = removed indices iterator
  {
    uindex_t pt_index = (uindex_t) rii;
    if (pt_index >= input_->size ())
    {
      PCL_ERROR ("[pcl::%s::filterDirectly] The index exceeds the size of the input. Do nothing.\n",
                 getClassName ().c_str ());
      *cloud = *input_;
      return;
    }
    std::uint8_t* pt_data = reinterpret_cast<std::uint8_t*> (&(*cloud)[pt_index]);
    for (const auto &field : fields)
      memcpy (pt_data + field.offset, &user_filter_value_, sizeof (float));
  }
  if (!std::isfinite (user_filter_value_))
    cloud->is_dense = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ExtractIndices<PointT>::applyFilter (PointCloud &output)
{
  Indices indices;
  if (keep_organized_)
  {
    bool temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    applyFilterIndices (indices);
    extract_removed_indices_ = temp;

    output = *input_;
    std::vector<pcl::PCLPointField> fields;
    pcl::for_each_type<FieldList> (pcl::detail::FieldAdder<PointT> (fields));
    for (const auto ri : *removed_indices_)  // ri = removed index
    {
      std::size_t pt_index = (std::size_t)ri;
      if (pt_index >= input_->size ())
      {
        PCL_ERROR ("[pcl::%s::applyFilter] The index exceeds the size of the input. Do nothing.\n",
                   getClassName ().c_str ());
        output = *input_;
        return;
      }
      std::uint8_t* pt_data = reinterpret_cast<std::uint8_t*> (&output[pt_index]);
      for (const auto &field : fields)
        memcpy (pt_data + field.offset, &user_filter_value_, sizeof (float));
    }
    if (!std::isfinite (user_filter_value_))
      output.is_dense = false;
  }
  else
  {
    applyFilterIndices (indices);
    copyPointCloud (*input_, indices, output);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ExtractIndices<PointT>::applyFilterIndices (Indices &indices)
{
  if (indices_->size () > input_->size ())
  {
    PCL_ERROR ("[pcl::%s::applyFilter] The indices size exceeds the size of the input.\n", getClassName ().c_str ());
    indices.clear ();
    removed_indices_->clear ();
    return;
  }

  if (!negative_)  // Normal functionality
  {
    indices = *indices_;

    if (extract_removed_indices_)
    {
      // Set up the full indices set
      Indices full_indices (input_->size ());
      std::iota (full_indices.begin (), full_indices.end (), static_cast<index_t> (0));

      // Set up the sorted input indices
      Indices sorted_input_indices = *indices_;
      std::sort (sorted_input_indices.begin (), sorted_input_indices.end ());

      // Store the difference in removed_indices
      removed_indices_->clear ();
      set_difference (full_indices.begin (), full_indices.end (), sorted_input_indices.begin (), sorted_input_indices.end (), inserter (*removed_indices_, removed_indices_->begin ()));
    }
  }
  else  // Inverted functionality
  {
    // Set up the full indices set
    Indices full_indices (input_->size ());
    std::iota (full_indices.begin (), full_indices.end (), static_cast<index_t> (0));

    // Set up the sorted input indices
    Indices sorted_input_indices = *indices_;
    std::sort (sorted_input_indices.begin (), sorted_input_indices.end ());

    // Store the difference in indices
    indices.clear ();
    set_difference (full_indices.begin (), full_indices.end (), sorted_input_indices.begin (), sorted_input_indices.end (), inserter (indices, indices.begin ()));

    if (extract_removed_indices_)
      removed_indices_ = indices_;
  }
}

#define PCL_INSTANTIATE_ExtractIndices(T) template class PCL_EXPORTS pcl::ExtractIndices<T>;

#endif  // PCL_FILTERS_IMPL_EXTRACT_INDICES_HPP_

