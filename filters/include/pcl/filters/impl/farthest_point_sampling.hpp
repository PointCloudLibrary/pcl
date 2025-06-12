/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *
 *  All rights reserved.
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
  const std::size_t size = indices_->size();
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

  //lambda to map filter indices back to pointcloud indices for increased readability
  auto toCloudIndex = [this](const auto idx){return (*indices_)[idx];};

  //pick the first point at random
  index_t max_index = dis(random_gen);
  distances_to_selected_points[max_index] = -1.0;
  indices.push_back(toCloudIndex(max_index));
  
  for (std::size_t j = 1; j < sample_size_; ++j)
  {
    const PointT& max_index_point = (*input_)[toCloudIndex(max_index)];

    #pragma omp parallel \
    default(none) \
    shared(distances_to_selected_points, max_index, max_index_point, size, toCloudIndex) \
    num_threads(num_threads_)
    {
      std::ptrdiff_t local_max_index = max_index;

      #pragma omp for
      for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(size); ++i)
      {
        if (distances_to_selected_points[i] == -1.0)
          continue;
        distances_to_selected_points[i] = std::min(distances_to_selected_points[i], geometry::distance((*input_)[toCloudIndex(i)], max_index_point));
        if (distances_to_selected_points[i] > distances_to_selected_points[local_max_index]) {
            local_max_index = i;
        }
      }

      #pragma omp critical
      {
        if (distances_to_selected_points[local_max_index] > distances_to_selected_points[max_index]) 
          max_index = local_max_index;
      }
    } //pragma omp parallel

    //select farthest point based on previously calculated distances
    //since distance is set to -1 for all selected elements,previously selected 
    //elements are guaranteed to not be selected
    distances_to_selected_points[max_index] = -1.0;
    indices.push_back(toCloudIndex(max_index));
    //set distance to -1 to ignore during max element search
  }

  if (extract_removed_indices_)
  {
    for (std::size_t k = 0; k < distances_to_selected_points.size(); ++k)
    {
      if (distances_to_selected_points[k] != -1.0)
        (*removed_indices_).push_back(toCloudIndex(k));
    }
  }
}

#define PCL_INSTANTIATE_FarthestPointSampling(T) template class PCL_EXPORTS pcl::FarthestPointSampling<T>;
