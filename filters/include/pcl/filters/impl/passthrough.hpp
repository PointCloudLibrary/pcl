/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/passthrough.h>

template <typename PointT> bool
pcl::PassThrough<PointT>::condition(const PointCloud& cloud, index_t idx)
{
    if (isXYZFinite(cloud[idx])) {
    // If a field name been specified, then filter non-finite entries
    if (filter_field_name_.empty())
      return !negative_;

    // Get the field's value
    const auto* pt_data = reinterpret_cast<const std::uint8_t*>(&cloud[idx]);
    float field_value = 0;
    memcpy(&field_value, pt_data + fields_[distance_idx_].offset, sizeof(float));
    if (std::isfinite(field_value) && field_value >= filter_limit_min_ &&
        field_value <= filter_limit_max_) {
      return true;
    }
  }
  return false;
}

template <typename PointT> void
pcl::PassThrough<PointT>::applyFilter (Indices &indices)
{
  // Attempt to get the field name's index
  if (!filter_field_name_.empty() && distance_idx_ == -1)
  {
  PCL_WARN ("[pcl::%s::applyFilter] Unable to find field name in point type.\n", getClassName ().c_str ());
  indices.clear ();
  removed_indices_->clear ();
  return;
  }

  const auto lambda = [this](const PointCloud& cloud, index_t idx) {
    return condition(cloud, idx);
  };

  pcl::FunctorFilter<PointT, decltype(lambda)> filter{lambda, extract_removed_indices_};
  filter.setInputCloud(input_);
  filter.setIndices(indices_);
  filter.setNegative(negative_);
  filter.applyFilter(indices);
  removed_indices_ =  std::const_pointer_cast<Indices>(filter.getRemovedIndices());
}

#define PCL_INSTANTIATE_PassThrough(T) template class PCL_EXPORTS pcl::PassThrough<T>;
