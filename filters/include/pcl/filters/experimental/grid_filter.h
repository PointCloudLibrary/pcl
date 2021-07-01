/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/filter.h>

#include <limits>

namespace pcl {
namespace experimental {

// template <typename GridStructT>
// using get_point_type = typename GridStructT::PointCloud::PointType;

#define GET_POINT_TYPE(GridStructT) typename GridStructT::PointCloud::PointType

template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class GridFilter : public Filter<PointT>, public GridStruct {
protected:
  using Filter<PointT>::filter_name_;
  using Filter<PointT>::input_;
  using Filter<PointT>::getClassName;

  using PointCloud = typename Filter<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  /** \brief Empty constructor. */
  GridFilter()
  : GridStruct()
  , downsample_all_data_(true)
  , save_leaf_layout_(false)
  , filter_field_name_("")
  , filter_limit_min_(std::numeric_limits<double>::min())
  , filter_limit_max_(std::numeric_limits<double>::max())
  , filter_limit_negative_(false)
  , min_points_per_voxel_(0)
  {
    filter_name_ = GridStruct::filter_name_;
  }

  /** \brief Destructor. */
  ~GridFilter() {}

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
   * \param[in] downsample the new value (true/false)
   */
  inline void
  setDownsampleAllData(const bool downsample)
  {
    downsample_all_data_ = downsample;
  }

  /** \brief Get the state of the internal downsampling parameter (true if all fields
   * need to be downsampled, false if just XYZ).
   */
  inline bool
  getDownsampleAllData() const
  {
    return downsample_all_data_;
  }

  /** \brief Set the minimum number of points required for a voxel to be used.
   * \param[in] min_points_per_voxel the minimum number of points for required for a
   * voxel to be used
   */
  inline void
  setMinimumPointsNumberPerVoxel(const std::size_t min_points_per_voxel)
  {
    min_points_per_voxel_ = min_points_per_voxel;
  }

  /** \brief Return the minimum number of points required for a voxel to be used.
   */
  inline std::size_t
  getMinimumPointsNumberPerVoxel() const
  {
    return min_points_per_voxel_;
  }

  /** \brief Set to true if leaf layout information needs to be saved for later access.
   * \param[in] save_leaf_layout the new value (true/false)
   */
  inline void
  setSaveLeafLayout(const bool save_leaf_layout)
  {
    save_leaf_layout_ = save_leaf_layout;
  }

  /** \brief Returns true if leaf layout information will to be saved for later access.
   */
  inline bool
  getSaveLeafLayout() const
  {
    return save_leaf_layout_;
  }

  /** \brief Provide the name of the field to be used for filtering data. In conjunction
   * with \a setFilterLimits, points having values outside this interval will be
   * discarded.
   * \param[in] field_name the name of the field that contains values used for filtering
   */
  inline void
  setFilterFieldName(const std::string& field_name)
  {
    filter_field_name_ = field_name;
  }

  /** \brief Get the name of the field used for filtering. */
  inline std::string const
  getFilterFieldName() const
  {
    return filter_field_name_;
  }

  /** \brief Set the field filter limits. All points having field values outside this
   * interval will be discarded.
   * \param[in] limit_min the minimum allowed field value
   * \param[in] limit_max the maximum allowed field value
   */
  inline void
  setFilterLimits(const double& limit_min, const double& limit_max)
  {
    filter_limit_min_ = limit_min;
    filter_limit_max_ = limit_max;
  }

  /** \brief Get the field filter limits (min/max) set by the user. The default values
   * are numeric_limits<double>::min(), numeric_limits<double>::max().
   * \param[out] limit_min the minimum allowed field value
   * \param[out] limit_max the maximum allowed field value
   */
  inline void
  getFilterLimits(double& limit_min, double& limit_max) const
  {
    limit_min = filter_limit_min_;
    limit_max = filter_limit_max_;
  }

  /** \brief Set to true if we want to return the data outside the interval specified by
   * setFilterLimits (min, max).
   * Default: false.
   * \param[in] limit_negative return data inside the interval (false) or outside (true)
   */
  inline void
  setFilterLimitsNegative(const bool limit_negative)
  {
    filter_limit_negative_ = limit_negative;
  }

  /** \brief Get whether the data outside the interval (min/max) is to be returned
   * (true) or inside (false).
   * \param[out] limit_negative true if data \b outside the interval [min; max] is to be
   * returned, false otherwise
   */
  inline void
  getFilterLimitsNegative(bool& limit_negative) const
  {
    limit_negative = filter_limit_negative_;
  }

  /** \brief Get whether the data outside the interval (min/max) is to be returned
   * (true) or inside (false).
   * \return true if data \b outside the interval [min; max] is to be returned, false
   * otherwise
   */
  inline bool
  getFilterLimitsNegative() const
  {
    return filter_limit_negative_;
  }

protected:
  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
  bool downsample_all_data_;

  /** \brief Set to true if leaf layout information needs to be saved in \a
   * leaf_layout_. */
  bool save_leaf_layout_;

  /** \brief The desired user filter field name. */
  std::string filter_field_name_;

  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a
   * filter_limit_max_).
   * Default: false. */
  bool filter_limit_negative_;

  /** \brief Minimum number of points per voxel for the centroid to be computed */
  std::size_t min_points_per_voxel_;

  /** \brief Downsample a Point Cloud using a voxelized grid approach
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
      auto res = GridStruct::filterGrid(it);
      if (res) {
        output.push_back(res.value());
      }
    }
  }
};

} // namespace experimental
} // namespace pcl