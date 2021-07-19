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

/** \brief @b TransformFilter represents the base class for filters that performs some
 * grid based tranformation on a point cloud, contrast to FilterIndices that performs
 * binary point removel. The tranformation can defined by a GridStruct object passed to
 * the class.
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
      PCL_WARN("[pcl::TransformFilter] Filter name is empty\n");

    filter_name_ = grid_struct_.filter_name_;
  }

  /** \brief Destructor. */
  ~TransformFilter() {}

protected:
  /** \brief Return const reference of the GridStruct object */
  const GridStruct&
  getGridStruct() const
  {
    return grid_struct_;
  }

  /** \brief Return reference of the GridStruct object */
  GridStruct&
  getGridStruct()
  {
    return grid_struct_;
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

    // iterate and add points to the grid
    if (input_->is_dense) {
      for (const auto& pt : *input_)
        grid_struct_.addPointToGrid(pt);
    }
    else {
      // filter out points with NAN
      for (const auto& pt : *input_) {
        if (isXYZFinite(pt))
          grid_struct_.addPointToGrid(pt);
      }
    }

    // allocate enough space for points
    output.reserve(grid_struct_.size());

    // iterate over the grid cells and compute the output point
    for (auto it = grid_struct_.begin(); it != grid_struct_.end(); ++it) {
      const optional<PointT> res = grid_struct_.filterGrid(it);
      if (res)
        output.push_back(res.value());
      }
    }
  }

  /** \brief The object defines the filtering operations and grid cells
   * \note Require the following member functions:
   *
   *   Non-functional:
   *     1. std::size_t size()
   *     2. begin() and end(), it can return any type which can be incremented and
   * compared, e.g. iterator of map containers or index of array containers
   *
   *   Functional:
   *     3. bool setUp(TransformFilter<GridStruct>*)
   *     4. void addPointToGrid(PointT&)
   *     5. pcl::experimental::optional<PointT> filterGrid(...), which accepts output
   * from begin() and end()
   */
  GridStruct grid_struct_;
};

} // namespace experimental
} // namespace pcl
