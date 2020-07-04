/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#include <map>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid_label.h>
#include <pcl/filters/impl/voxel_grid.hpp>

//////////////////////////////////////////////////////////////////////////////
void
pcl::VoxelGridLabel::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height       = 1;                    // downsampling breaks the organized structure
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D<pcl::PointXYZRGBL>(input_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D<pcl::PointXYZRGBL>(*input_, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if( (dx*dy*dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) )
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    output.clear();
    return;
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

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

  int centroid_size = 4;
  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  std::vector<pcl::PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex<PointXYZRGBL> ("rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex<PointXYZRGBL> ("rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 3;
  }

  // ---[ Label special case
  int label_index = -1;
  label_index = pcl::getFieldIndex<PointXYZRGBL> ("label", fields);

  std::vector<cloud_point_index_idx> index_vector;
  index_vector.reserve(input_->points.size());

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex<PointXYZRGBL> (filter_field_name_, fields);
    if (distance_idx == -1)
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (unsigned int cp = 0; cp < static_cast<unsigned int> (input_->points.size ()); ++cp)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!std::isfinite ((*input_)[cp].x) || 
            !std::isfinite ((*input_)[cp].y) || 
            !std::isfinite ((*input_)[cp].z))
          continue;

      // Get the distance value
      const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&(*input_)[cp]);
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
      
      int ijk0 = static_cast<int> (std::floor ((*input_)[cp].x * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (std::floor ((*input_)[cp].y * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (std::floor ((*input_)[cp].z * inverse_leaf_size_[2]) - min_b_[2]);

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.emplace_back(static_cast<unsigned int> (idx), cp);
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (unsigned int cp = 0; cp < static_cast<unsigned int> (input_->points.size ()); ++cp)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!std::isfinite ((*input_)[cp].x) || 
            !std::isfinite ((*input_)[cp].y) || 
            !std::isfinite ((*input_)[cp].z))
          continue;

      int ijk0 = static_cast<int> (std::floor ((*input_)[cp].x * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (std::floor ((*input_)[cp].y * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (std::floor ((*input_)[cp].z * inverse_leaf_size_[2]) - min_b_[2]);

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.emplace_back(static_cast<unsigned int> (idx), cp);
    }
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  unsigned int total = 0;
  unsigned int index = 0;
  while (index < index_vector.size ()) 
  {
    unsigned int i = index + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
      ++i;
    ++total;
    index = i;
  }

  // Fourth pass: compute centroids, insert them into their final position
  output.points.resize (total);
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
        "voxel_grid.hpp", "applyFilter");	
    }
    catch (std::length_error&)
    {
      throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout", 
        "voxel_grid.hpp", "applyFilter");	
    }
  }
  
  index = 0;
  Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
  Eigen::VectorXf temporary = Eigen::VectorXf::Zero (centroid_size);

  for (unsigned int cp = 0; cp < index_vector.size ();)
  {
    std::map<int, int> labels;
    
    // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
    if (!downsample_all_data_) 
    {
      centroid[0] = (*input_)[index_vector[cp].cloud_point_index].x;
      centroid[1] = (*input_)[index_vector[cp].cloud_point_index].y;
      centroid[2] = (*input_)[index_vector[cp].cloud_point_index].z;
    }
    else 
    {
      // ---[ RGB special case
      if (rgba_index >= 0)
      {
        // Fill r/g/b data, assuming that the order is BGRA
        pcl::RGB rgb;
        memcpy (&rgb, reinterpret_cast<const char*> (&(*input_)[index_vector[cp].cloud_point_index]) + rgba_index, sizeof (RGB));
        centroid[centroid_size-3] = rgb.r;
        centroid[centroid_size-2] = rgb.g;
        centroid[centroid_size-1] = rgb.b;
      }

      // ---[ Label special case
      if (label_index >= 0)
      {
        // store the label in a map data structure
        std::uint32_t label = (*input_)[index_vector[cp].cloud_point_index].label;
        std::map<int, int>::iterator it = labels.find (label);
        if (it == labels.end ())
          labels.insert (labels.begin (), std::pair<int, int> (label, 1));
        else
          it->second = it->second++;
      }

      pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <pcl::PointXYZRGBL> ((*input_)[index_vector[cp].cloud_point_index], centroid));
    }

    unsigned int i = cp + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[cp].idx) 
    {
      if (!downsample_all_data_) 
      {
        centroid[0] += (*input_)[index_vector[i].cloud_point_index].x;
        centroid[1] += (*input_)[index_vector[i].cloud_point_index].y;
        centroid[2] += (*input_)[index_vector[i].cloud_point_index].z;
      }
      else 
      {
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          // Fill r/g/b data, assuming that the order is BGRA
          pcl::RGB rgb;
          memcpy (&rgb, reinterpret_cast<const char*> (&(*input_)[index_vector[i].cloud_point_index]) + rgba_index, sizeof (RGB));
          temporary[centroid_size-3] = rgb.r;
          temporary[centroid_size-2] = rgb.g;
          temporary[centroid_size-1] = rgb.b;
        }
        pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <pcl::PointXYZRGBL> ((*input_)[index_vector[i].cloud_point_index], temporary));
        centroid += temporary;
      }
      ++i;
    }

    // index is centroid final position in resulting PointCloud
    if (save_leaf_layout_)
      leaf_layout_[index_vector[cp].idx] = index;

    centroid /= static_cast<float> (i - cp);

    // store centroid
    // Do we need to process all the fields?
    if (!downsample_all_data_) 
    {
      output[index].x = centroid[0];
      output[index].y = centroid[1];
      output[index].z = centroid[2];
    }
    else 
    {
      pcl::for_each_type<FieldList> (pcl::NdCopyEigenPointFunctor <pcl::PointXYZRGBL> (centroid, output[index]));
      // ---[ RGB special case
      if (rgba_index >= 0) 
      {
        // pack r/g/b into rgb
        float r = centroid[centroid_size-3], g = centroid[centroid_size-2], b = centroid[centroid_size-1];
        int rgb = (static_cast<int> (r) << 16) | (static_cast<int> (g) << 8) | static_cast<int> (b);
        memcpy (reinterpret_cast<char*> (&output[index]) + rgba_index, &rgb, sizeof (float));
      }

      if (label_index >= 0)
      {
        // find the label with the highest occurrence
        int n_occurrence = labels.begin()->second;
        int best_label = labels.begin()->first;
        for (const auto &label : labels)
        {
          if (n_occurrence < label.second)
          {
            n_occurrence = label.second;
            best_label = label.first;
          }
        }
        output[index].label = best_label;
      }
    }
    cp = i;
    ++index;
  }
  output.width = static_cast<std::uint32_t> (output.points.size ());
}

