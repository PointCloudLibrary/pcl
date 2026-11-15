/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2021-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/common/point_tests.h>
#include <pcl/console/print.h>

#include <boost/optional.hpp> // std::optional for C++17

#include <type_traits>

namespace pcl {
namespace experimental {

template <typename PointT>
using optional = boost::optional<PointT>;

template <template <typename> class FilterBase, typename GridStruct, typename PointT>
class CartesianFilter;
namespace detail {
template <template <typename> class FilterBase, typename GridStruct, typename PointT>
struct IsGridStruct {
private:
  template <typename T>
  static constexpr auto
  hasSetUp(T*) -> typename std::is_same<
      decltype(std::declval<T>().setUp(
          std::declval<CartesianFilter<FilterBase, T, PointT>&>())),
      bool>::type;
  template <typename>
  static constexpr std::false_type
  hasSetUp(...);

  template <typename T>
  static constexpr auto
  hasAddPoint(T*) -> typename std::is_same<
      decltype(std::declval<T>().addPointToGrid(std::declval<PointT&>())),
      void>::type;
  template <typename>
  static constexpr std::false_type
  hasAddPoint(...);

  template <typename T>
  static constexpr auto
  hasFilterBegin(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(begin(std::declval<T&>()))),
      optional<PointT>>::type;
  template <typename T>
  static constexpr auto
  hasFilterBegin(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(std::declval<T>().begin())),
      optional<PointT>>::type;
  template <typename>
  static constexpr std::false_type
  hasFilterBegin(...);

  template <typename T>
  static constexpr auto
  hasFilterEnd(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(end(std::declval<T>()))),
      optional<PointT>>::type;
  template <typename T>
  static constexpr auto
  hasFilterEnd(T*) -> typename std::is_same<
      decltype(std::declval<T>().filterGrid(std::declval<T>().end())),
      optional<PointT>>::type;
  template <typename>
  static constexpr std::false_type
  hasFilterEnd(...);

  template <typename T>
  static constexpr auto
  hasSize(T*) ->
      typename std::is_same<decltype(std::declval<T>().size()), std::size_t>::type;
  template <typename>
  static constexpr std::false_type
  hasSize(...);

  using has_set_up = decltype(hasSetUp<GridStruct>(0));
  using has_add_point = decltype(hasAddPoint<GridStruct>(0));
  using has_filter_begin = decltype(hasFilterBegin<GridStruct>(0));
  using has_filter_end = decltype(hasFilterEnd<GridStruct>(0));
  using has_size = decltype(hasSize<GridStruct>(0));

  static constexpr bool is_set_up_valid = has_set_up::value;
  static constexpr bool is_add_point_valid = has_add_point::value;
  static constexpr bool is_filter_valid =
      has_filter_begin::value && has_filter_end::value;
  static constexpr bool is_size_valid = has_size::value;

public:
  static constexpr bool is_valid =
      is_set_up_valid && is_add_point_valid && is_filter_valid && is_size_valid;
};
} // namespace detail

#define GET_POINT_TYPE(GridStructT) typename GridStructT::PointCloud::PointType

/** \brief @b TransformFilter represents the base class for filters that performs some
 * grid based tranformation on a point cloud, contrast to FilterIndices that performs
 * binary point removel. The tranformation can defined by a GridStruct object passed to
 * the class.
 * \ingroup filters
 */
template <template <typename> class FilterBase,
          typename GridStruct,
          typename PointT = GET_POINT_TYPE(GridStruct)>
class TransformFilter : public FilterBase<PointT> {
  static_assert(detail::IsGridStruct<FilterBase, GridStruct, PointT>::is_valid,
                "GridStruct requirement is not satisfied");

protected:
  using FilterBase<PointT>::filter_name_;
  using FilterBase<PointT>::getClassName;
  using FilterBase<PointT>::input_;

  using PointCloud = typename FilterBase<PointT>::PointCloud;
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

  /** \brief Constructor.
   * \param[in] filter_name filter name
   */
  TransformFilter(const std::string& filter_name) { filter_name_ = filter_name; }

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

    output.clear();
    output.height = 1;      // downsampling breaks the organized structure
    output.is_dense = true; // we filter out invalid points

    if (!grid_struct_.setUp(
            static_cast<CartesianFilter<FilterBase, GridStruct, PointT>&>(*this))) {
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
    {
      using std::begin; // invoke ADL, just like ranged-for loop
      using std::end;
      auto it = begin(grid_struct_);
      auto end_it = end(grid_struct_);
      for (; it != end_it; ++it) {
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
   *     3. bool setUp(CartesianFilter<FilterBase, GridStruct>&)
   *     4. void addPointToGrid(PointT&)
   *     5. pcl::experimental::optional<PointT> filterGrid(...), which accepts output
   * from begin() and end()
   */
  GridStruct grid_struct_;
};

} // namespace experimental
} // namespace pcl
