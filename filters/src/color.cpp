/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Suat Gedikli (gedikli@willowgarage.com)
 *
 */

#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/color.h"
#include "pcl/filters/impl/color.hpp"

//////////////////////////////////////////////////////////////////////////
void
pcl::ColorFilter<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &output)
{
  if (!input_)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset not given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  int nr_points = input_->width * input_->height;

  // Check if we're going to keep the organized structure of the cloud or not
  if (keep_organized_)
  {
    output.width = input_->width;
    output.height = input_->height;
    // Check what the user value is: if !finite, set is_dense to false, true otherwise
    output.is_dense = false;
  }
  else
  {
    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1; // filtering breaks the organized structure
    // Because we're doing explit checks for isfinite, is_dense = true
    output.is_dense = true;
  }
  output.row_step = input_->row_step;
  output.point_step = input_->point_step;
  output.is_bigendian = input_->is_bigendian;
  output.data.resize (input_->data.size ());

  removed_indices_->resize (input_->data.size ());

  int nr_p = 0;
  int nr_removed_p = 0;
  // Create the first xyz_offset
  Eigen::Array4i xyz_offset (input_->fields[x_idx_].offset, input_->fields[y_idx_].offset,
                             input_->fields[z_idx_].offset, 0);

  Eigen::Vector4f pt = Eigen::Vector4f::Zero ();
  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    int distance_idx = pcl::getFieldIndex (*input_, "rgb");
    if (distance_idx == -1)
    {
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);
      output.width = output.height = 0;
      output.data.clear ();
      return;
    }

    // @todo fixme
    if (input_->fields[distance_idx].datatype != sensor_msgs::PointField::FLOAT32)
    {
      PCL_ERROR ("[pcl::%s::downsample] Distance filtering requested, but distances are not float/double in the dataset! Only FLOAT32/FLOAT64 distances are supported right now.\n", getClassName ().c_str ());
      output.width = output.height = 0;
      output.data.clear ();
      return;
    }

    float badpt = std::numeric_limits<float>::quiet_NaN ();
    // Check whether we need to store filtered valued in place
    if (keep_organized_)
    {
      float distance_value = 0;
      // Go over all points
      for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step)
      {
        // Copy all the fields
        memcpy (&output.data[cp * output.point_step], &input_->data[cp * output.point_step], output.point_step);

        // Get the distance value
        memcpy (&distance_value, &input_->data[cp * input_->point_step + input_->fields[distance_idx].offset],
                sizeof(float));

        // ToDo
        memcpy (&output.data[xyz_offset[0]], &badpt, sizeof(float));
        memcpy (&output.data[xyz_offset[1]], &badpt, sizeof(float));
        memcpy (&output.data[xyz_offset[2]], &badpt, sizeof(float));
      }
    }
    // Remove filtered points
    else
    {
      // Go over all points
      float distance_value = 0;
      for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step)
      {
        // Get the distance value
        memcpy (&distance_value, &input_->data[cp * input_->point_step + input_->fields[distance_idx].offset],
                sizeof(float));

        // Unoptimized memcpys: assume fields x, y, z are in random order
        memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof(float));
        memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof(float));
        memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof(float));

        // Check if the point is invalid
        if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2]))
        {
          if (extract_removed_indices_)
          {
            (*removed_indices_)[nr_removed_p] = cp;
            nr_removed_p++;
          }
          continue;
        }

        // Copy all the fields
        memcpy (&output.data[nr_p * output.point_step], &input_->data[cp * output.point_step], output.point_step);
        nr_p++;
      }
      output.width = nr_p;
    } // !keep_organized_
  }
  // No distance filtering, process all data. No need to check for is_organized here as we did it above
  else
  {
    for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step)
    {
      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof(float));
      memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof(float));
      memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof(float));

      // Check if the point is invalid
      if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2]))
      {
        if (extract_removed_indices_)
        {
          (*removed_indices_)[nr_removed_p] = cp;
          nr_removed_p++;
        }
        continue;
      }

      // Copy all the fields
      memcpy (&output.data[nr_p * output.point_step], &input_->data[cp * output.point_step], output.point_step);
      nr_p++;
    }
    output.width = nr_p;
  }

  output.row_step = output.point_step * output.width;
  output.data.resize (output.width * output.height * output.point_step);

  removed_indices_->resize (nr_removed_p);
}
// Instantiations of specific point types
PCL_INSTANTIATE(ColorFilter, (pcl::PointXYZRGB));

