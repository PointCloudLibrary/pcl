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

#define GET_POINT_TYPE(GridStructT) typename GridStructT::PointCloud::PointType

/**
 * \brief The base of GridFilterBase which has the common member functions and
 * attribute for the grid filters.
 * \ingroup filters
 */
template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class TransformFilter : public Filter<PointT> {
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
    if (grid_struct_.filter_name_.empty())
      PCL_WARN("[pcl::%s] Filter name is empty\n", getClassName().c_str());

    filter_name_ = grid_struct_.filter_name_;
  }

  /** \brief Destructor. */
  ~TransformFilter() {}

protected:
  const GridStruct&
  getGridStruct() const
  {
    return grid_struct_;
  }

  GridStruct&
  getGridStruct()
  {
    return const_cast<GridStruct&>(
        const_cast<const TransformFilter*>(this)->getGridStruct());
  }

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

    output.height = 1;      // downsampling breaks the organized structure
    output.is_dense = true; // we filter out invalid points

    if (!grid_struct_.setUp(this)) {
      output = *input_;
      return;
    }

    for (const auto& pt : *input_) {
      if (input_->is_dense || isXYZFinite(pt)) {
        grid_struct_.addPointToGrid(pt);
      }
    }

    // allocate enough space for points
    output.reserve(grid_struct_.size());
    for (auto it = grid_struct_.begin(); it != grid_struct_.end(); ++it) {
      const optional<PointT> res = grid_struct_.filterGrid(it);
      if (res)
        output.push_back(res.value());
      }
    }
  }

  GridStruct grid_struct_;
};

} // namespace experimental
} // namespace pcl
