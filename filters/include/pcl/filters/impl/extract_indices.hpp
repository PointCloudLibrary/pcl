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

#ifndef PCL_FILTERS_IMPL_EXTRACT_INDICES_HPP_
#define PCL_FILTERS_IMPL_EXTRACT_INDICES_HPP_

#include <pcl/filters/extract_indices.h>
#include <pcl/common/io.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ExtractIndices<PointT>::applyFilter (PointCloud &output)
{
  // If the subset is the empty set
  if (indices_->empty () || input_->points.empty ())
  {
    // Full set copy
    if (negative_)
      output = *input_;
    // Empty set copy
    else if (keep_organized_)
    {
      output = *input_;
      std::vector<sensor_msgs::PointField> fields;
      pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::detail::FieldAdder<PointT> (fields));
      for (int p_it = 0; p_it < static_cast<int> (output.points.size ()); ++p_it)
      {
        uint8_t* pt_data = reinterpret_cast<uint8_t*> (&output.points[p_it]);
        for (int f_it = 0; f_it < static_cast<int> (fields.size ()); ++f_it)
          memcpy (pt_data + fields[f_it].offset, &user_filter_value_, sizeof (float));
      }
      if (pcl_isfinite (user_filter_value_))
        output.is_dense = true;
      else
        output.is_dense = false;
    }
    else
    {
      output.width = output.height = 0;
      output.points.clear ();
    }
    return;
  }

  // If the subset is the full set
  if (indices_->size () == (input_->width * input_->height) || indices_->size () == input_->points.size ())
  {
    // Empty set copy
    if (negative_)
      if (keep_organized_)
      {
        output = *input_;
        std::vector<sensor_msgs::PointField> fields;
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::detail::FieldAdder<PointT> (fields));
        for (int p_it = 0; p_it < static_cast<int> (output.points.size ()); ++p_it)
        {
          uint8_t* pt_data = reinterpret_cast<uint8_t*> (&output.points[p_it]);
          for (int f_it = 0; f_it < static_cast<int> (fields.size ()); ++f_it)
            memcpy (pt_data + fields[f_it].offset, &user_filter_value_, sizeof (float));
        }
        if (pcl_isfinite (user_filter_value_))
          output.is_dense = true;
        else
          output.is_dense = false;
      }
      else
      {
        output.width = output.height = 0;
        output.points.clear ();
      }
    // Full set copy
    else
      output = *input_;
    return;
  }

  // If the subset is a proper subset
  if (negative_)
  {
    if (keep_organized_)
    {
      // Case: negative = true, keep_organized_ = true
      output = *input_;
      std::vector<sensor_msgs::PointField> fields;
      pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::detail::FieldAdder<PointT> (fields));
      for (int i_it = 0; i_it < static_cast<int> (indices_->size ()); ++i_it)
      {
        uint8_t* pt_data = reinterpret_cast<uint8_t*> (&output.points[(*indices_)[i_it]]);
        for (int f_it = 0; f_it < static_cast<int> (fields.size ()); ++f_it)
          memcpy (pt_data + fields[f_it].offset, &user_filter_value_, sizeof (float));
      }
      if (!pcl_isfinite (user_filter_value_))
        output.is_dense = false;
    }
    else
    {
      // Case: negative = true, keep_organized_ = false
      std::vector<int> indices_fullset (input_->points.size ());
      std::vector<int> indices_subset = *indices_;
      std::vector<int> indices_difference;

      // Set up the full indices set
      for (int p_it = 0; p_it < static_cast<int> (indices_fullset.size ()); ++p_it)
        indices_fullset[p_it] = p_it;
      // Set up the subset input indices
      std::sort (indices_subset.begin (), indices_subset.end ());
      // Get the difference
      set_difference (indices_fullset.begin (), indices_fullset.end (), indices_subset.begin (), indices_subset.end (), inserter (indices_difference, indices_difference.begin ()));

      copyPointCloud (*input_, indices_difference, output);
    }
  }
  else if (keep_organized_)
  {
    // Case: negative = false, keep_organized_ = true
    std::vector<int> indices_fullset (input_->points.size ());
    std::vector<int> indices_subset = *indices_;
    std::vector<int> indices_difference;

    // Set up the full indices set
    for (int p_it = 0; p_it < static_cast<int> (indices_fullset.size ()); ++p_it)
      indices_fullset[p_it] = p_it;
    // Set up the subset input indices
    std::sort (indices_subset.begin (), indices_subset.end ());
    // Get the difference
    set_difference (indices_fullset.begin (), indices_fullset.end (), indices_subset.begin (), indices_subset.end (), inserter (indices_difference, indices_difference.begin ()));

    output = *input_;
    std::vector<sensor_msgs::PointField> fields;
    pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::detail::FieldAdder<PointT> (fields));
    for (int i_it = 0; i_it < static_cast<int> (indices_difference.size ()); ++i_it)
    {
      uint8_t* pt_data = reinterpret_cast<uint8_t*> (&output.points[indices_difference[i_it]]);
      for (int f_it = 0; f_it < static_cast<int> (fields.size ()); ++f_it)
        memcpy (pt_data + fields[f_it].offset, &user_filter_value_, sizeof (float));
    }
    if (!pcl_isfinite (user_filter_value_))
      output.is_dense = false;
  }
  else
  {
    // Case: negative = false, keep_organized_ = false
    copyPointCloud (*input_, *indices_, output);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ExtractIndices<PointT>::applyFilter (std::vector<int> &indices)
{
  if (negative_)
  {
    // If the subset is the full set
    if (indices_->size () == (input_->width * input_->height) || indices_->size () == input_->points.size ())
    {
      // Empty set copy
      indices.clear ();
      return;
    }

    // Set up the full indices set
    std::vector<int> indices_fullset (input_->points.size ());
    for (int p_it = 0; p_it < static_cast<int> (indices_fullset.size ()); ++p_it)
      indices_fullset[p_it] = p_it;

    // If the subset is the empty set
    if (indices_->empty () || input_->points.empty ())
    {
      // Full set copy
      indices = indices_fullset;
      return;
    }

    // If the subset is a proper subset
    // Set up the subset input indices
    std::vector<int> indices_subset = *indices_;
    std::sort (indices_subset.begin (), indices_subset.end ());

    // Get the difference
    set_difference (indices_fullset.begin (), indices_fullset.end (), indices_subset.begin (), indices_subset.end (), inserter (indices, indices.begin ()));
  }
  else
    indices = *indices_;
}

#define PCL_INSTANTIATE_ExtractIndices(T) template class PCL_EXPORTS pcl::ExtractIndices<T>;

#endif  //#ifndef PCL_FILTERS_IMPL_EXTRACT_INDICES_HPP_

