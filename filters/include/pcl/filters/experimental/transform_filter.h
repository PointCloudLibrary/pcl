/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/experimental/grid_filter_base.h>

namespace pcl {
namespace experimental {

#define GET_POINT_TYPE(GridStructT) typename GridStructT::PointCloud::PointType

/**
 * \brief Filter point clouds based on a templated grid structure
 * \details Used as the base class for grid based filters, e.g. VoxelGrid,
 * ApproximateVoxelGrid. For grid based filters with different behavior, one can
 * implement a custom grid structure and pass to this class. The templated grid
 * structure should have two main components: member functions for filtering
 * opterations, Grid member attribute for storing the information of the smaller spaces
 * divided by the grid (e.g. voxels in VoxelGrid).
 *
 * Requirements of the grid structure:
 *  1. Three member functions (setUp, addPointToGrid, filterGrid) the grid structure are
 * called in applyFilter and thus required to declare.
 *  2. A Grid member attribute is required, it can be any type with built-in iterator,
 * e.g. STL container or custom grid with iterator
 * \ingroup filters
 */
template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class TransformFilter : public GridFilterBase<PointT>, public GridStruct {
protected:
  using GridFilterBase<PointT>::filter_name_;
  using GridFilterBase<PointT>::input_;
  using GridFilterBase<PointT>::getClassName;

  using PointCloud = typename GridFilterBase<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  /** \brief Empty constructor. */
  TransformFilter() : GridStruct()
  {
    if (GridStruct::filter_name_.empty()) {
      PCL_WARN("[pcl::%s] Filter name is empty\n", getClassName().c_str());
    }
    filter_name_ = GridStruct::filter_name_;
  }

  /** \brief Destructor. */
  ~TransformFilter() {}

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

    if (!GridStruct::setUp(this)) {
      output = *input_;
      return;
    }

    for (const auto& pt : *input_) {
      GridStruct::addPointToGrid(pt);
    }

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