/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/filters/impl/extract_indices.hpp>

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ExtractIndices<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
  if (keep_organized_)
  {
    output = *input_;
    if (negative_)
    {
      // Prepare the output and copy the data
      for (size_t i = 0; i < indices_->size (); ++i)
        for (size_t j = 0; j < output.fields.size(); ++j)
          memcpy (&output.data[(*indices_)[i] * output.point_step + output.fields[j].offset],
                  &user_filter_value_, sizeof(float));
    }
    else
    {
      // Prepare a vector holding all indices
      std::vector<int> all_indices (input_->width * input_->height);
      for (int i = 0; i < static_cast<int>(all_indices.size ()); ++i)
        all_indices[i] = i;

      std::vector<int> indices = *indices_;
      std::sort (indices.begin (), indices.end ());

      // Get the diference
      std::vector<int> remaining_indices;
      set_difference (all_indices.begin (), all_indices.end (), indices.begin (), indices.end (),
                      inserter (remaining_indices, remaining_indices.begin ()));

      // Prepare the output and copy the data
      for (size_t i = 0; i < remaining_indices.size (); ++i)
        for (size_t j = 0; j < output.fields.size(); ++j)
          memcpy (&output.data[remaining_indices[i] * output.point_step + output.fields[j].offset],
                  &user_filter_value_, sizeof(float));
    }
    if (!pcl_isfinite (user_filter_value_))
      output.is_dense = false;
    return;
  }
  if (indices_->empty () || (input_->width * input_->height == 0))
  {
    output.width = output.height = 0;
    output.data.clear ();
    // If negative, copy all the data
    if (negative_)
      output = *input_;
    return;
  }
  if (indices_->size () == (input_->width * input_->height))
  {
    // If negative, then return an empty cloud
    if (negative_)
    {
      output.width = output.height = 0;
      output.data.clear ();
    }
    // else, we need to return all points
    else
      output = *input_;
    return;
  }

  // Copy the common fields (header and fields should have already been copied)
  output.is_bigendian = input_->is_bigendian;
  output.point_step   = input_->point_step;
  output.height       = 1;
  // TODO: check the output cloud and assign is_dense based on whether the points are valid or not
  output.is_dense     = false;

  if (negative_)
  {
    // Prepare a vector holding all indices
    std::vector<int> all_indices (input_->width * input_->height);
    for (int i = 0; i < static_cast<int>(all_indices.size ()); ++i)
      all_indices[i] = i;

    std::vector<int> indices = *indices_;
    std::sort (indices.begin (), indices.end ());

    // Get the diference
    std::vector<int> remaining_indices;
    set_difference (all_indices.begin (), all_indices.end (), indices.begin (), indices.end (),
                    inserter (remaining_indices, remaining_indices.begin ()));

    // Prepare the output and copy the data
    output.width = static_cast<uint32_t> (remaining_indices.size ());
    output.data.resize (remaining_indices.size () * output.point_step);
    for (size_t i = 0; i < remaining_indices.size (); ++i)
      memcpy (&output.data[i * output.point_step], &input_->data[remaining_indices[i] * output.point_step], output.point_step);
  }
  else
  {
    // Prepare the output and copy the data
    output.width = static_cast<uint32_t> (indices_->size ());
    output.data.resize (indices_->size () * output.point_step);
    for (size_t i = 0; i < indices_->size (); ++i)
      memcpy (&output.data[i * output.point_step], &input_->data[(*indices_)[i] * output.point_step], output.point_step);
  }
  output.row_step = output.point_step * output.width;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ExtractIndices<pcl::PCLPointCloud2>::applyFilter (std::vector<int> &indices)
{
  if (negative_)
  {
    // If the subset is the full set
    if (indices_->size () == (input_->width * input_->height))
    {
      // Empty set copy
      indices.clear ();
      return;
    }

    // Set up the full indices set
    std::vector<int> indices_fullset (input_->width * input_->height);
    for (int p_it = 0; p_it < static_cast<int> (indices_fullset.size ()); ++p_it)
      indices_fullset[p_it] = p_it;

    // If the subset is the empty set
    if (indices_->empty () || (input_->width * input_->height == 0))
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

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE(ExtractIndices, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::Normal)(pcl::PointXYZRGBNormal))
#else
  PCL_INSTANTIATE(ExtractIndices, PCL_POINT_TYPES)
#endif

#endif    // PCL_NO_PRECOMPILE

