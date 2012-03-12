/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_IMPL_PASSTHROUGH_H_
#define PCL_FILTERS_IMPL_PASSTHROUGH_H_

#include <pcl/filters/passthrough.h>
#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PassThrough<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Check if we're going to keep the organized structure of the cloud or not
  if (keep_organized_)
  {
    if (filter_field_name_.empty ())
    {
      // Silly - if no filtering is actually done, and we want to keep the data organized, 
      // just copy everything. Any optimizations that can be done here???
      output = *input_;
      return;
    }

    output.width  = input_->width;
    output.height = input_->height;
    // Check what the user value is: if !finite, set is_dense to false, true otherwise
    if (!pcl_isfinite (user_filter_value_))
      output.is_dense = false;
    else
      output.is_dense = input_->is_dense;
  }
  else
  {
    output.height = 1;                    // filtering breaks the organized structure
    // Because we're doing explit checks for isfinite, is_dense = true
    output.is_dense = true;
  }
  output.points.resize (input_->points.size ());
  removed_indices_->resize (input_->points.size ());

  size_t nr_p = 0;
  int nr_removed_p = 0;

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<sensor_msgs::PointField> fields;
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
    if (distance_idx == -1)
    {
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);
      output.width = output.height = 0;
      output.points.clear ();
      return;
    }

    if (keep_organized_)
    {
      for (int cp = 0; cp < static_cast<int>(input_->points.size ()); ++cp)
      {
        if (pcl_isnan (input_->points[cp].x) ||
            pcl_isnan (input_->points[cp].y) ||
            pcl_isnan (input_->points[cp].z))
        {
          output.points[cp].x = output.points[cp].y = output.points[cp].z = user_filter_value_;
          continue;
        }

        // Copy the point
        pcl::for_each_type<FieldList> (pcl::NdConcatenateFunctor <PointT, PointT> (input_->points[cp], output.points[cp]));

        // Filter it. Get the distance value
        const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&input_->points[cp]);
        float distance_value = 0;
        memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

        if (filter_limit_negative_)
        {
          // Use a threshold for cutting out points which inside the interval
          if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
          {
            output.points[cp].x = output.points[cp].y = output.points[cp].z = user_filter_value_;
            continue;
          }
          else 
          {
            if (extract_removed_indices_)
            {
              (*removed_indices_)[nr_removed_p] = cp;
              nr_removed_p++;
            }
          }
        }
        else
        {
          // Use a threshold for cutting out points which are too close/far away
          if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
          {
            output.points[cp].x = output.points[cp].y = output.points[cp].z = user_filter_value_;
            continue;
          }
          else
          {
            if (extract_removed_indices_)
            {
              (*removed_indices_)[nr_removed_p] = cp;
              nr_removed_p++;
            }
          }
        }
      }
      nr_p = input_->points.size ();
    }
    // Remove filtered points
    else
    {
      // Go over all points
      for (int cp = 0; cp < static_cast<int>(input_->points.size ()); ++cp)
      {
        // Check if the point is invalid
        if (!pcl_isfinite (input_->points[cp].x) || !pcl_isfinite (input_->points[cp].y) || !pcl_isfinite (input_->points[cp].z))
        {
          if (extract_removed_indices_)
          {
            (*removed_indices_)[nr_removed_p] = cp;
            nr_removed_p++;
          }
          continue;
        }

        // Get the distance value
        const uint8_t* pt_data = reinterpret_cast<const uint8_t*>(&input_->points[cp]);
        float distance_value = 0;
        memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

        if (filter_limit_negative_)
        {
          // Use a threshold for cutting out points which inside the interval
          if (distance_value < filter_limit_max_ && distance_value > filter_limit_min_)
          {
            if (extract_removed_indices_)
            {
              (*removed_indices_)[nr_removed_p] = cp;
              nr_removed_p++;
            }
            continue;
          }
        }
        else
        {
          // Use a threshold for cutting out points which are too close/far away
          if (distance_value > filter_limit_max_ || distance_value < filter_limit_min_)
          {
            if (extract_removed_indices_)
            {
              (*removed_indices_)[nr_removed_p] = cp;
              nr_removed_p++;
            }
            continue;
          }
        }

        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointT, PointT> (input_->points[cp], output.points[nr_p]));
        nr_p++;
      }
      output.width = static_cast<uint32_t>(nr_p);
    } // !keep_organized_
  }
  // No distance filtering, process all data. No need to check for is_organized here as we did it above
  else
  {
    // First pass: go over all points and insert them into the right leaf
    for (int cp = 0; cp < static_cast<int>(input_->points.size ()); ++cp)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[cp].x) || !pcl_isfinite (input_->points[cp].y) || !pcl_isfinite (input_->points[cp].z))
      {
        if (extract_removed_indices_)
        {
          (*removed_indices_)[nr_removed_p] = cp;
          nr_removed_p++;
        }
        continue;
      }

      pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointT, PointT> (input_->points[cp], output.points[nr_p]));
      nr_p++;
    }
    output.width = static_cast<uint32_t>(nr_p);
  }

  output.points.resize (output.width * output.height);
  removed_indices_->resize(nr_removed_p);
}

#define PCL_INSTANTIATE_PassThrough(T) template class PCL_EXPORTS pcl::PassThrough<T>;

#endif    // PCL_FILTERS_IMPL_PASSTHROUGH_H_

