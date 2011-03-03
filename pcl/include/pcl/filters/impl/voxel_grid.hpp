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
 * $Id: voxel_grid.hpp 35830 2011-02-08 06:18:23Z rusu $
 *
 */

#ifndef PCL_FILTERS_IMPL_VOXEL_GRID_H_
#define PCL_FILTERS_IMPL_VOXEL_GRID_H_

#include "pcl/common/common.h"
#include "pcl/filters/voxel_grid.h"

template <typename PointT> void
pcl::getMinMax3D (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                  const std::string &distance_field_name, float min_distance, float max_distance,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative)
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // Get the fields list and the distance field index
  std::vector<sensor_msgs::PointField> fields;
  int distance_idx = pcl::getFieldIndex (*cloud, distance_field_name, fields);

  float distance_value;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    // Get the distance value
    uint8_t* pt_data = (uint8_t*)&cloud->points[i];
    memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

    if (limit_negative)
    {
      // Use a threshold for cutting out points which inside the interval
      if ((distance_value < max_distance) && (distance_value > min_distance))
        continue;
    }
    else
    {
      // Use a threshold for cutting out points which are too close/far away
      if ((distance_value > max_distance) || (distance_value < min_distance))
        continue;
    }

    // Check if the point is invalid
    if (!pcl_isfinite (cloud->points[i].x) || 
        !pcl_isfinite (cloud->points[i].y) || 
        !pcl_isfinite (cloud->points[i].z))
      continue;
    // Create the point structure and get the min/max
    pcl::Array4fMapConst pt = cloud->points[i].getArray4fMap ();
    min_p = min_p.min (pt);
    max_p = max_p.max (pt);
  }
  min_pt = min_p;
  max_pt = max_p;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelGrid<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    ROS_WARN ("[pcl::%s::applyFilter] No input dataset given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  // Avoid division errors
  if (leaf_size_[3] == 0)
    leaf_size_[3] = 1;

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height       = 1;                    // downsampling breaks the organized structure
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D<PointT>(input_, filter_field_name_, filter_limit_min_, filter_limit_max_, min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D<PointT>(*input_, min_p, max_p);

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

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);
  Eigen::Vector4i ijk = Eigen::Vector4i::Zero ();

  int centroid_size = 4;
  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  std::vector<sensor_msgs::PointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
  if (rgba_index == -1)
  {
    rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
  }
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 3;
  }

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<sensor_msgs::PointField> fields;
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
    if (distance_idx == -1)
      ROS_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.", getClassName ().c_str (), distance_idx);

    // First pass: go over all points and insert them into the right leaf
    for (size_t cp = 0; cp < input_->points.size (); ++cp)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[cp].x) || 
          !pcl_isfinite (input_->points[cp].y) || 
          !pcl_isfinite (input_->points[cp].z))
        continue;

      // Get the distance value
      uint8_t* pt_data = (uint8_t*)&input_->points[cp];
      float distance_value = 0;
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (filter_limit_negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
          continue;
      }

      Eigen::Vector4f pt (input_->points[cp].x, input_->points[cp].y, input_->points[cp].z, 0);

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
        leaf.centroid.template head<4> () += pt;
      }
      else
      {
        // Copy all the fields
        Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          // fill r/g/b data
          int rgb;
          memcpy (&rgb, ((char *)&(input_->points[cp])) + rgba_index, sizeof (int));
          centroid[centroid_size-3] = (rgb>>16) & 0x0000ff;
          centroid[centroid_size-2] = (rgb>>8)  & 0x0000ff;
          centroid[centroid_size-1] = (rgb)     & 0x0000ff;
        }
        pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <PointT> (input_->points[cp], centroid));
        leaf.centroid += centroid;
      }
      ++leaf.nr_points;
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the right leaf
    for (size_t cp = 0; cp < input_->points.size (); ++cp)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[cp].x) || 
          !pcl_isfinite (input_->points[cp].y) || 
          !pcl_isfinite (input_->points[cp].z))
        continue;

      Eigen::Vector4f pt (input_->points[cp].x, input_->points[cp].y, input_->points[cp].z, 0);

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
        leaf.centroid.template head<4> () += pt;
      }
      else
      {
        // Copy all the fields
        Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          // fill r/g/b data
          int rgb;
          memcpy (&rgb, ((char *)&(input_->points[cp])) + rgba_index, sizeof (int));
          centroid[centroid_size-3] = (rgb>>16) & 0x0000ff;
          centroid[centroid_size-2] = (rgb>>8)  & 0x0000ff;
          centroid[centroid_size-1] = (rgb)     & 0x0000ff;
        }
        pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <PointT> (input_->points[cp], centroid));
        leaf.centroid += centroid;
      }
      ++leaf.nr_points;
    }
  }

  // Second pass: go over all leaves and compute centroids
  output.points.clear ();
  output.points.reserve (leaves_.size ());
  int cp = 0;
  leaf_layout_.clear ();
  if (save_leaf_layout_)
    leaf_layout_.resize (div_b_[0]*div_b_[1]*div_b_[2], -1);
  for (typename std::map<size_t, Leaf>::const_iterator it = leaves_.begin (); it != leaves_.end (); ++it)
  {
    // Save leaf layout information for fast access to cells relative to current position
    if (save_leaf_layout_)
      leaf_layout_[it->first] = cp++;

    // Normalize the centroid
    const Leaf& leaf = it->second;
    output.points.push_back (PointT ());
    PointT& point = output.points.back ();

    // Normalize the centroid
    float norm_pts = 1.0f / leaf.nr_points;
    Eigen::VectorXf centroid = leaf.centroid * norm_pts;

    // Do we need to process all the fields?
    if (!downsample_all_data_)
    {
      point.x = centroid[0];
      point.y = centroid[1];
      point.z = centroid[2];
    }
    else
    {
      pcl::for_each_type <FieldList> (pcl::NdCopyEigenPointFunctor <PointT> (centroid, point));
      // ---[ RGB special case
      if (rgba_index >= 0)
      {
        // pack r/g/b into rgb
        float r = centroid[centroid_size-3], g = centroid[centroid_size-2], b = centroid[centroid_size-1];
        int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
        memcpy (((char *)&point) + rgba_index, &rgb, sizeof (float));
      }
    }
  }
  output.width = output.points.size ();
}

#define PCL_INSTANTIATE_VoxelGrid(T) template class pcl::VoxelGrid<T>;
#define PCL_INSTANTIATE_getMinMax3D(T) template void pcl::getMinMax3D<T> (const pcl::PointCloud<T>::ConstPtr &, const std::string &, float, float, Eigen::Vector4f &, Eigen::Vector4f &, bool);

#endif    // PCL_FILTERS_IMPL_VOXEL_GRID_H_

