/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
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

#pragma once

#include <pcl/common/geometry.h>
#include <pcl/filters/farthest_point_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <limits>
#include <random>

template<typename PointT> void
pcl::FarthestPointSampling<PointT>::applyFilter (Indices &indices)
{
  const std::size_t size = input_->size();
  //if requested number of point is equal to the point cloud size, copy original cloud
  if (sample_size_ == size)
  {
    indices = *indices_;
    removed_indices_->clear ();
    return;
  }
  //check if requested number of points is greater than the point cloud size
  if (sample_size_ > size)
  {
    PCL_THROW_EXCEPTION (BadArgumentException,
                        "Requested number of points is greater than point cloud size!");
  }

  std::vector<float> distances_to_selected_points (size, std::numeric_limits<float>::max ());
  
  //set random seed
  std::mt19937 random_gen(seed_);
  std::uniform_int_distribution<index_t> dis(0, size -1);

  //pick the first point at random
  index_t max_index = dis(random_gen);
  distances_to_selected_points[max_index] = -1.0;
  indices.push_back(max_index);
  
  for (std::size_t j = 1; j < sample_size_; ++j)
  {
    index_t next_max_index = 0;
    
    //recompute distances
    for (index_t i = 0; i < size; ++i)
    {
      if (distances_to_selected_points[i] != -1.0)
        distances_to_selected_points[i] = std::min(distances_to_selected_points[i], geometry::distance((*input_)[i], (*input_)[max_index]));
      if (distances_to_selected_points[i] > distances_to_selected_points[next_max_index])
        next_max_index = i;
    }

    //select farthest point based on previously calculated distances
    //since distance is set to -1 for all selected elements,previously selected 
    //elements are guaranteed to not be selected
    max_index = next_max_index;
    distances_to_selected_points[max_index] = -1.0;
    indices.push_back(max_index);
    //set distance to -1 to ignore during max element search
  }

  if (extract_removed_indices_)
  {
    for (std::size_t k = 0; k < distances_to_selected_points.size(); ++k)
    {
      if (distances_to_selected_points[k] != -1.0)
        (*removed_indices_).push_back(k);
    }
  }
}

#define PCL_INSTANTIATE_FarthestPointSampling(T) template class PCL_EXPORTS pcl::FarthestPointSampling<T>;
