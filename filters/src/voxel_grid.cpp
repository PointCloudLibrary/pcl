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
 * $Id: voxel_grid.cpp 35830 2011-02-08 06:18:23Z rusu $
 *
 */

#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/io/io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/impl/voxel_grid.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::getMinMax3D (const sensor_msgs::PointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  // @todo fix this
  if (cloud->fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      cloud->fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      cloud->fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("[pcl::getMinMax3D] XYZ dimensions are not float type!");
    return;
  }

  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  int nr_points = cloud->width * cloud->height;

  Eigen::Array4f pt = Eigen::Array4f::Zero ();
  Eigen::Array4i xyz_offset (cloud->fields[x_idx].offset, cloud->fields[y_idx].offset, cloud->fields[z_idx].offset, 0);

  for (int cp = 0; cp < nr_points; ++cp)
  {
    // Unoptimized memcpys: assume fields x, y, z are in random order
    memcpy (&pt[0], &cloud->data[xyz_offset[0]], sizeof (float));
    memcpy (&pt[1], &cloud->data[xyz_offset[1]], sizeof (float));
    memcpy (&pt[2], &cloud->data[xyz_offset[2]], sizeof (float));
    // Check if the point is invalid
    if (!pcl_isfinite (pt[0]) || 
        !pcl_isfinite (pt[1]) || 
        !pcl_isfinite (pt[2]))
    {
      xyz_offset += cloud->point_step;
      continue;
    }
    xyz_offset += cloud->point_step;
    min_p = (min_p.min) (pt);
    max_p = (max_p.max) (pt);
  }
  min_pt = min_p;
  max_pt = max_p;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::getMinMax3D (const sensor_msgs::PointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                  const std::string &distance_field_name, float min_distance, float max_distance,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative)
{
  // @todo fix this
  if (cloud->fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      cloud->fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      cloud->fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("[pcl::getMinMax3D] XYZ dimensions are not float type!");
    return;
  }

  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // Get the distance field index
  int distance_idx = pcl::getFieldIndex (*cloud, distance_field_name);

  // @todo fix this
  if (cloud->fields[distance_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("[pcl::getMinMax3D] Filtering dimensions is not float type!");
    return;
  }

  int nr_points = cloud->width * cloud->height;

  Eigen::Array4f pt = Eigen::Array4f::Zero ();
  Eigen::Array4i xyz_offset (cloud->fields[x_idx].offset,
                             cloud->fields[y_idx].offset,
                             cloud->fields[z_idx].offset,
                             0);
  float distance_value = 0;
  for (int cp = 0; cp < nr_points; ++cp)
  {
    int point_offset = cp * cloud->point_step;

    // Get the distance value
    memcpy (&distance_value, &cloud->data[point_offset + cloud->fields[distance_idx].offset], sizeof (float));

    if (limit_negative)
    {
      // Use a threshold for cutting out points which inside the interval
      if ((distance_value < max_distance) && (distance_value > min_distance))
      {
        xyz_offset += cloud->point_step;
        continue;
      }
    }
    else
    {
      // Use a threshold for cutting out points which are too close/far away
      if ((distance_value > max_distance) || (distance_value < min_distance))
      {
        xyz_offset += cloud->point_step;
        continue;
      }
    }

    // Unoptimized memcpys: assume fields x, y, z are in random order
    memcpy (&pt[0], &cloud->data[xyz_offset[0]], sizeof (float));
    memcpy (&pt[1], &cloud->data[xyz_offset[1]], sizeof (float));
    memcpy (&pt[2], &cloud->data[xyz_offset[2]], sizeof (float));
    // Check if the point is invalid
    if (!pcl_isfinite (pt[0]) || 
        !pcl_isfinite (pt[1]) || 
        !pcl_isfinite (pt[2]))
    {
      xyz_offset += cloud->point_step;
      continue;
    }
    xyz_offset += cloud->point_step;
    min_p = (min_p.min) (pt);
    max_p = (max_p.max) (pt);
  }
  min_pt = min_p;
  max_pt = max_p;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::VoxelGrid<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &output)
{
  // If fields x/y/z are not present, we cannot downsample
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
  {
    ROS_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }
  // Avoid division errors
  if (leaf_size_[3] == 0)
    leaf_size_[3] = 1;

  int nr_points  = input_->width * input_->height;

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height         = 1;                    // downsampling breaks the organized structure
  if (downsample_all_data_)
  {
    output.fields       = input_->fields;
    output.point_step   = input_->point_step;
  }
  else
  {
    output.fields.resize (4);

    output.fields[0] = input_->fields[x_idx_];
    output.fields[0].offset = 0;

    output.fields[1] = input_->fields[y_idx_];
    output.fields[1].offset = 4;

    output.fields[2] = input_->fields[z_idx_];
    output.fields[2].offset = 8;

    output.fields[3].name = "rgba";
    output.fields[3].offset = 12;
    output.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    output.point_step = 16;
  }
  output.is_bigendian = input_->is_bigendian;
  output.row_step     = input_->row_step;
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D (input_, x_idx_, y_idx_, z_idx_, filter_field_name_, filter_limit_min_, filter_limit_max_, min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D (input_, x_idx_, y_idx_, z_idx_, min_p, max_p);

  // Use multiplications instead of divisions
  Eigen::Array4f inverse_leaf_size = Eigen::Array4f::Ones () / leaf_size_.array ();

  // @todo fix floor
  //min_b = (min_p.cwise () * leaf_size).cast<int> ();
  //max_b = (max_p.cwise () * leaf_size).cast<int> ();
  // Compute the minimum and maximum bounding box values
  min_b_[0] = (int)(floor (min_p[0] * inverse_leaf_size[0]));
  max_b_[0] = (int)(floor (max_p[0] * inverse_leaf_size[0]));
  min_b_[1] = (int)(floor (min_p[1] * inverse_leaf_size[1]));
  max_b_[1] = (int)(floor (max_p[1] * inverse_leaf_size[1]));
  min_b_[2] = (int)(floor (min_p[2] * inverse_leaf_size[2]));
  max_b_[2] = (int)(floor (max_p[2] * inverse_leaf_size[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  // Clear the leaves
  leaves_.clear();

  // Create the first xyz_offset, and set up the division multiplier
  Eigen::Array4i xyz_offset (input_->fields[x_idx_].offset,
                             input_->fields[y_idx_].offset,
                             input_->fields[z_idx_].offset,
                             0);
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);
  Eigen::Vector4f pt  = Eigen::Vector4f::Zero ();
  Eigen::Vector4i ijk = Eigen::Vector4i::Zero ();

  int centroid_size = 4;
  if (downsample_all_data_)
    centroid_size = input_->fields.size ();

  int rgba_index = -1;

  // ---[ RGB special case
  // if the data contains "rgba" or "rgb", add an extra field for r/g/b in centroid
  for (int d = 0; d < centroid_size; ++d)
  {
    if (input_->fields[d].name == std::string("rgba") || input_->fields[d].name == std::string("rgb"))
    {
      rgba_index = d;
      centroid_size += 3;
      break;
    }
  }

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_);

    // @todo fixme
    if (input_->fields[distance_idx].datatype != sensor_msgs::PointField::FLOAT32)
    {
      ROS_ERROR ("[pcl::%s::applyFilter] Distance filtering requested, but distances are not float/double in the dataset! Only FLOAT32/FLOAT64 distances are supported right now.", getClassName ().c_str ());
      output.width = output.height = 0;
      output.data.clear ();
      return;
    }

    // First pass: go over all points and insert them into the right leaf
    float distance_value = 0;
    for (int cp = 0; cp < nr_points; ++cp)
    {
      int point_offset = cp * input_->point_step;
      // Get the distance value
      memcpy (&distance_value, &input_->data[point_offset + input_->fields[distance_idx].offset], sizeof (float));

      if (filter_limit_negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if (distance_value < filter_limit_max_ && distance_value > filter_limit_min_)
        {
          xyz_offset += input_->point_step;
          continue;
        }
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if (distance_value > filter_limit_max_ || distance_value < filter_limit_min_)
        {
          xyz_offset += input_->point_step;
          continue;
        }
      }

      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof (float));
      memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof (float));
      memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof (float));

      // Check if the point is invalid
      if (!pcl_isfinite (pt[0]) || 
          !pcl_isfinite (pt[1]) || 
          !pcl_isfinite (pt[2]))
      {
        xyz_offset += input_->point_step;
        continue;
      }

      // @todo fix floor
      //ijk = (pt.cwise () / leaf_size_).cast<int> ();
      ijk[0] = (int)(floor (pt[0] * inverse_leaf_size[0]));
      ijk[1] = (int)(floor (pt[1] * inverse_leaf_size[1]));
      ijk[2] = (int)(floor (pt[2] * inverse_leaf_size[2]));

      // Compute the centroid leaf index
      int idx = (ijk - min_b_).dot (divb_mul_);
      Leaf& leaf = leaves_[idx];

      if (leaf.nr_points == 0)
      {
        leaf.centroid.resize (centroid_size);
        leaf.centroid.setZero ();
      }

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
        leaf.centroid.head<4> () += pt;
      }
      else
      {
        Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
        // ---[ RGB special case
        // fill in extra r/g/b centroid field
        if (rgba_index >= 0)
        {
          int rgb;
          memcpy (&rgb, &input_->data[point_offset + input_->fields[rgba_index].offset], sizeof (int));
          centroid[centroid_size-3] = (rgb>>16) & 0x0000ff;
          centroid[centroid_size-2] = (rgb>>8)  & 0x0000ff;
          centroid[centroid_size-1] = (rgb)     & 0x0000ff;
        }
        // Copy all the fields
        for (unsigned int d = 0; d < input_->fields.size (); ++d)
          memcpy (&centroid[d], &input_->data[point_offset + input_->fields[d].offset], field_sizes_[d]);
        leaf.centroid += centroid;
      }
      ++leaf.nr_points;

      xyz_offset += input_->point_step;
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the right leaf
    for (int cp = 0; cp < nr_points; ++cp)
    {
      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof (float));
      memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof (float));
      memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof (float));

      // Check if the point is invalid
      if (!pcl_isfinite (pt[0]) || 
          !pcl_isfinite (pt[1]) || 
          !pcl_isfinite (pt[2]))
      {
        xyz_offset += input_->point_step;
        continue;
      }

      // @todo fix floor
      //ijk = (pt.cwise () / leaf_size_).cast<int> ();
      ijk[0] = (int)(floor (pt[0] * inverse_leaf_size[0]));
      ijk[1] = (int)(floor (pt[1] * inverse_leaf_size[1]));
      ijk[2] = (int)(floor (pt[2] * inverse_leaf_size[2]));

      // Compute the centroid leaf index
      int idx = (ijk - min_b_).dot (divb_mul_);


      Leaf& leaf = leaves_[idx];

      if (leaf.nr_points == 0)
      {
        leaf.centroid.resize (centroid_size);
        leaf.centroid.setZero ();
      }

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
        leaf.centroid.head<4> () += pt;
      }
      else
      {
        Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
        int point_offset = cp * input_->point_step;
        // ---[ RGB special case
        // fill extra r/g/b centroid field
        if (rgba_index >= 0)
        {
          int rgb;
          memcpy (&rgb, &input_->data[point_offset + input_->fields[rgba_index].offset], sizeof (int));
          centroid[centroid_size-3] = (rgb>>16) & 0x0000ff;
          centroid[centroid_size-2] = (rgb>>8)  & 0x0000ff;
          centroid[centroid_size-1] = (rgb)     & 0x0000ff;
        }
        // Copy all the fields
        for (unsigned int d = 0; d < input_->fields.size(); ++d)
          memcpy (&centroid[d], &input_->data[point_offset + input_->fields[d].offset], field_sizes_[d]);
        leaf.centroid += centroid;
      }
      ++leaf.nr_points;

      xyz_offset += input_->point_step;
    }
  }

  // Second pass: go over all leaves and compute centroids
  int nr_p = 0;
  // If we downsample each field, the {x,y,z}_idx_ offsets should correspond in input_ and output
  if (downsample_all_data_)
    xyz_offset = Eigen::Array4i (output.fields[x_idx_].offset,
                                 output.fields[y_idx_].offset,
                                 output.fields[z_idx_].offset,
                                 0);
  else
    // If not, we must have created a new xyzw cloud
    xyz_offset = Eigen::Array4i (0, 4, 8, 12);

  Eigen::VectorXf centroid;
  output.width = leaves_.size ();
  output.row_step = output.point_step * output.width;
  output.data.resize (output.width * output.point_step);

  leaf_layout_.clear ();
  if (save_leaf_layout_)
    leaf_layout_.resize (div_b_[0]*div_b_[1]*div_b_[2], -1);

  int cp = 0;
  for (std::map<size_t, Leaf>::const_iterator it = leaves_.begin (); it != leaves_.end (); ++it)
  {
    // Save leaf layout information for fast access to cells relative to current position
    if (save_leaf_layout_)
      leaf_layout_[it->first] = cp++;

    // Normalize the centroid
    const Leaf& leaf = it->second;
    float norm_pts = 1.0f / leaf.nr_points;
    centroid = leaf.centroid * norm_pts;

    // Do we need to process all the fields?
    if (!downsample_all_data_)
    {
      // Copy the data
      memcpy (&output.data[xyz_offset[0]], &centroid[0], sizeof (float));
      memcpy (&output.data[xyz_offset[1]], &centroid[1], sizeof (float));
      memcpy (&output.data[xyz_offset[2]], &centroid[2], sizeof (float));
      xyz_offset += output.point_step;
    }
    else
    {
      int point_offset = nr_p * output.point_step;
      // Copy all the fields
      for (size_t d = 0; d < output.fields.size (); ++d)
        memcpy (&output.data[point_offset + output.fields[d].offset], &centroid[d], field_sizes_[d]);

      // ---[ RGB special case
      // full extra r/g/b centroid field
      if (rgba_index >= 0) 
      {
        float r = centroid[centroid_size-3], g = centroid[centroid_size-2], b = centroid[centroid_size-1];
        int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
        memcpy (&output.data[point_offset + output.fields[rgba_index].offset], &rgb, sizeof(float));
      }
    }
    ++nr_p;
  }
}

// Instantiations of specific point types
PCL_INSTANTIATE(getMinMax3D, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(VoxelGrid, PCL_XYZ_POINT_TYPES);

