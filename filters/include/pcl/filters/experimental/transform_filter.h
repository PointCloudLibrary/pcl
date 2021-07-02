/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/common/point_tests.h>
#include <pcl/filters/filter.h>

namespace pcl {
namespace experimental {

/**
 * \brief The base of GridFilterBase which has the common member functions and
 * attribute for the grid filters.
 * \ingroup filters
 */
template <typename GridStruct, typename PointT>
class TransformFilter : public Filter<PointT>, public GridStruct {
protected:
  using Filter<PointT>::filter_name_;
  using Filter<PointT>::getClassName;
  using Filter<PointT>::input_;

  using PointCloud = typename Filter<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  /** \brief Empty constructor. */
  TransformFilter()
  {
    if (GridStruct::filter_name_.empty()) {
      PCL_WARN("[pcl::%s] Filter name is empty\n", getClassName().c_str());
    }
    filter_name_ = GridStruct::filter_name_;
  }

  /** \brief Destructor. */
  ~TransformFilter() {}

protected:
  /** \brief Filter a Point Cloud based on the templated grid structure
   * \param[out] output the resultant point cloud message
   */
  void
  applyFilter(PointCloud& output) override
  {
    // Has the input dataset been set already?
    if (!input_) {
      PCL_WARN("[pcl::%s::applyFilter] No input dataset given!\n",
               getClassName().c_str());
      return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1;      // downsampling breaks the organized structure
    output.is_dense = true; // we filter out invalid points

    if (!GridStruct::setUp()) {
      output = *input_;
      return;
    }

    for (const auto& pt : *input_) {
      if (input_->is_dense || isXYZFinite(pt)) {
        GridStruct::addPointToGrid(pt);
      }
    }

    output.reserve(GridStruct::grid_.size());
    for (auto it = GridStruct::grid_.begin(); it != GridStruct::grid_.end(); ++it) {
      const auto& res = GridStruct::filterGrid(it);
      if (res) {
        output.push_back(res.value());
      }
    }
  }
};

} // namespace experimental
} // namespace pcl