/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <limits>
#include <pcl/common/io.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <boost/sort/spreadsort/integer_sort.hpp>
#include <array>

using Array4size_t = Eigen::Array<std::size_t, 4, 1>;

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::getMinMax3D (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  // @todo fix this
  if (cloud->fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
      cloud->fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
      cloud->fields[z_idx].datatype != pcl::PCLPointField::FLOAT32)
  {
    PCL_ERROR ("[pcl::getMinMax3D] XYZ dimensions are not float type!\n");
    return;
  }

  Eigen::Array4f min_p, max_p;
  min_p.setConstant (std::numeric_limits<float>::max());
  max_p.setConstant (std::numeric_limits<float>::lowest());

  std::size_t nr_points = cloud->width * cloud->height;

  Eigen::Array4f pt = Eigen::Array4f::Zero ();
  Array4size_t xyz_offset (cloud->fields[x_idx].offset, cloud->fields[y_idx].offset, cloud->fields[z_idx].offset, 0);

  for (std::size_t cp = 0; cp < nr_points; ++cp)
  {
    // Unoptimized memcpys: assume fields x, y, z are in random order
    memcpy (&pt[0], &cloud->data[xyz_offset[0]], sizeof (float));
    memcpy (&pt[1], &cloud->data[xyz_offset[1]], sizeof (float));
    memcpy (&pt[2], &cloud->data[xyz_offset[2]], sizeof (float));
    // Check if the point is invalid
    if (!std::isfinite (pt[0]) || 
        !std::isfinite (pt[1]) || 
        !std::isfinite (pt[2]))
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

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::getMinMax3D (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                  const std::string &distance_field_name, float min_distance, float max_distance,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative)
{
  // @todo fix this
  if (cloud->fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
      cloud->fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
      cloud->fields[z_idx].datatype != pcl::PCLPointField::FLOAT32)
  {
    PCL_ERROR ("[pcl::getMinMax3D] XYZ dimensions are not float type!\n");
    return;
  }

  Eigen::Array4f min_p, max_p;
  min_p.setConstant (std::numeric_limits<float>::max());
  max_p.setConstant (std::numeric_limits<float>::lowest());

  // Get the distance field index
  int distance_idx = pcl::getFieldIndex (*cloud, distance_field_name);

  // @todo fix this
  if (cloud->fields[distance_idx].datatype != pcl::PCLPointField::FLOAT32)
  {
    PCL_ERROR ("[pcl::getMinMax3D] Filtering dimensions is not float type!\n");
    return;
  }

  std::size_t nr_points = cloud->width * cloud->height;

  Eigen::Array4f pt = Eigen::Array4f::Zero ();
  Array4size_t xyz_offset (cloud->fields[x_idx].offset,
                           cloud->fields[y_idx].offset,
                           cloud->fields[z_idx].offset,
                           0);
  float distance_value = 0;
  for (std::size_t cp = 0; cp < nr_points; ++cp)
  {
    std::size_t point_offset = cp * cloud->point_step;

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
    if (!std::isfinite (pt[0]) || 
        !std::isfinite (pt[1]) || 
        !std::isfinite (pt[2]))
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

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::VoxelGrid<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
  // If fields x/y/z are not present, we cannot downsample
  if (x_idx_ == UNAVAILABLE || y_idx_ == UNAVAILABLE || z_idx_ == UNAVAILABLE)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }
  std::size_t nr_points  = input_->width * input_->height;

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

    output.point_step = 12;
  }
  output.is_bigendian = input_->is_bigendian;
  output.row_step     = input_->row_step;
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D (input_, x_idx_, y_idx_, z_idx_, filter_field_name_, 
                 static_cast<float> (filter_limit_min_), 
                 static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D (input_, x_idx_, y_idx_, z_idx_, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if( (dx*dy*dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) )
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.\n", getClassName().c_str());
    //output.width = output.height = 0;
    //output.data.clear();
    //return;
  }

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (std::floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (std::floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (std::floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (std::floor (max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int> (std::floor (min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int> (std::floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  std::vector<cloud_point_index_idx> index_vector;
  index_vector.reserve (nr_points);

  // Create the first xyz_offset, and set up the division multiplier
  Array4size_t xyz_offset (input_->fields[x_idx_].offset,
                           input_->fields[y_idx_].offset,
                           input_->fields[z_idx_].offset,
                           0);
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);
  Eigen::Vector4f pt  = Eigen::Vector4f::Zero ();

  int centroid_size = 4;
  int rgba_index = -1;

  if (downsample_all_data_)
  {
    centroid_size = static_cast<int> (input_->fields.size ());
    
    // ---[ RGB special case 
    // if the data contains "rgba" or "rgb", add an extra field for r/g/b in centroid
    for (int d = 0; d < centroid_size; ++d)
    {
      if (input_->fields[d].name == std::string ("rgba") || input_->fields[d].name == std::string ("rgb"))
      {
        rgba_index = d;
        centroid_size += 4;
        break;
      }
    }
  }
  
  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_);

    // @todo fixme
    if (input_->fields[distance_idx].datatype != pcl::PCLPointField::FLOAT32)
    {
      PCL_ERROR ("[pcl::%s::applyFilter] Distance filtering requested, but distances are not float/double in the dataset! Only FLOAT32/FLOAT64 distances are supported right now.\n", getClassName ().c_str ());
      output.width = output.height = 0;
      output.data.clear ();
      return;
    }

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    float distance_value = 0;
    for (std::size_t cp = 0; cp < nr_points; ++cp)
    {
      std::size_t point_offset = cp * input_->point_step;
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
      if (!std::isfinite (pt[0]) || 
          !std::isfinite (pt[1]) || 
          !std::isfinite (pt[2]))
      {
        xyz_offset += input_->point_step;
        continue;
      }

      int ijk0 = static_cast<int> (std::floor (pt[0] * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (std::floor (pt[1] * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (std::floor (pt[2] * inverse_leaf_size_[2]) - min_b_[2]);
      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.emplace_back(idx, static_cast<unsigned int> (cp));

      xyz_offset += input_->point_step;
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the right leaf
    for (std::size_t cp = 0; cp < nr_points; ++cp)
    {
      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof (float));
      memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof (float));
      memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof (float));

      // Check if the point is invalid
      if (!std::isfinite (pt[0]) || 
          !std::isfinite (pt[1]) || 
          !std::isfinite (pt[2]))
      {
        xyz_offset += input_->point_step;
        continue;
      }

      int ijk0 = static_cast<int> (std::floor (pt[0] * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (std::floor (pt[1] * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (std::floor (pt[2] * inverse_leaf_size_[2]) - min_b_[2]);
      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.emplace_back(idx, static_cast<unsigned int> (cp));
      xyz_offset += input_->point_step;
    }
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  auto rightshift_func = [](const cloud_point_index_idx &x, const unsigned offset) { return x.idx >> offset; };
  boost::sort::spreadsort::integer_sort(index_vector.begin(), index_vector.end(), rightshift_func);

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  std::size_t total = 0;
  std::size_t index = 0;
  // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
  // index_vector belonging to the voxel which corresponds to the i-th output point,
  // and of the first point not belonging to.
  std::vector<std::pair<index_t, index_t> > first_and_last_indices_vector;
  // Worst case size
  first_and_last_indices_vector.reserve (index_vector.size ());
  while (index < index_vector.size ()) 
  {
    std::size_t i = index + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
      ++i;
    if ((i - index) >= min_points_per_voxel_)
    {
      ++total;
      first_and_last_indices_vector.emplace_back(index, i);
    }
    index = i;
  }

  // Fourth pass: compute centroids, insert them into their final position
  output.width = std::uint32_t (total);
  output.row_step = output.point_step * output.width;
  output.data.resize (output.width * output.point_step);

  if (save_leaf_layout_) 
  {
    try
    {
      // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
      std::uint32_t new_layout_size = div_b_[0]*div_b_[1]*div_b_[2];
      //This is the number of elements that need to be re-initialized to -1
      std::uint32_t reinit_size = std::min (static_cast<unsigned int> (new_layout_size), static_cast<unsigned int> (leaf_layout_.size()));
      for (std::uint32_t i = 0; i < reinit_size; i++)
      {
        leaf_layout_[i] = -1;
      }        
      leaf_layout_.resize (new_layout_size, -1);           
    }
    catch (std::bad_alloc&)
    {
      throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout", 
        "voxel_grid.cpp", "applyFilter");	
    }
    catch (std::length_error&)
    {
      throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout", 
        "voxel_grid.cpp", "applyFilter");	
    }
  }
  
  // If we downsample each field, the {x,y,z}_idx_ offsets should correspond in input_ and output
  if (downsample_all_data_)
    xyz_offset = Array4size_t (output.fields[x_idx_].offset,
                               output.fields[y_idx_].offset,
                               output.fields[z_idx_].offset,
                               0);
  else
    // If not, we must have created a new xyzw cloud
    xyz_offset = Array4size_t (0, 4, 8, 12);

  index=0;
  Eigen::VectorXf temporary = Eigen::VectorXf::Zero (centroid_size);
  const std::array<index_t, 3> field_indices {x_idx_, y_idx_, z_idx_};

  for (const auto &cp : first_and_last_indices_vector)
  {
    // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
    index_t first_index = cp.first;
    index_t last_index = cp.second;

    // index is centroid final position in resulting PointCloud
    if (save_leaf_layout_)
      leaf_layout_[index_vector[first_index].idx] = index;

    //Limit downsampling to coords
    if (!downsample_all_data_)
    {
      Eigen::Vector4f centroid (Eigen::Vector4f::Zero ());

      for (index_t li = first_index; li < last_index; ++li)
      {
        std::size_t point_offset = index_vector[li].cloud_point_index * input_->point_step;
        for (uint8_t ind=0; ind<3; ++ind)
        {
          memcpy (&pt[ind], &input_->data[point_offset+input_->fields[field_indices[ind]].offset], sizeof (float));
        }
        centroid += pt;
      }

      centroid /= static_cast<float> (last_index - first_index);
      for (uint8_t ind=0; ind<3; ++ind)
      {
        memcpy (&output.data[xyz_offset[ind]], &centroid[ind], sizeof (float));
      }
      xyz_offset += output.point_step;
    }
    else
    {

      Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
      // fill in the accumulator with leaf points
      for (index_t li = first_index; li < last_index; ++li) {
        std::size_t point_offset = index_vector[li].cloud_point_index * input_->point_step;
        // ---[ RGB special case
        // fill extra r/g/b centroid field
        if (rgba_index >= 0)
        {
            pcl::RGB rgb;
            memcpy (&rgb, &input_->data[point_offset + input_->fields[rgba_index].offset], sizeof (RGB));
            temporary[centroid_size-4] = rgb.r;
            temporary[centroid_size-3] = rgb.g;
            temporary[centroid_size-2] = rgb.b;
            temporary[centroid_size-1] = rgb.a;
        }
        // Copy all the fields
        for (std::size_t d = 0; d < input_->fields.size (); ++d)
          memcpy (&temporary[d], &input_->data[point_offset + input_->fields[d].offset], field_sizes_[d]);
        centroid += temporary;
      }
      centroid /= static_cast<float> (last_index - first_index);
      
      std::size_t point_offset = index * output.point_step;
      // Copy all the fields
      for (std::size_t d = 0; d < output.fields.size (); ++d)
        memcpy (&output.data[point_offset + output.fields[d].offset], &centroid[d], field_sizes_[d]);

      // ---[ RGB special case
      // full extra r/g/b centroid field
      if (rgba_index >= 0) 
      {
        float r = centroid[centroid_size-4], g = centroid[centroid_size-3], b = centroid[centroid_size-2], a = centroid[centroid_size-1];
        int rgb = (static_cast<int> (a) << 24) | (static_cast<int> (r) << 16) | (static_cast<int> (g) << 8) | static_cast<int> (b);
        memcpy (&output.data[point_offset + output.fields[rgba_index].offset], &rgb, sizeof (float));
      }
    }
     
    ++index;
  }
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(getMinMax3D, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE(VoxelGrid, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE
