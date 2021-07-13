/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/experimental/transform_filter.h>

#include <limits>

namespace pcl {
namespace experimental {

/**
 * \brief CartesianFilter represents the base class for grid filters.
 * \ingroup filters
 */
template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class CartesianFilter : public TransformFilter<GridStruct, PointT> {
protected:
  using PointCloud = typename TransformFilter<GridStruct, PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  /** \brief Empty constructor. */
  CartesianFilter() {}

  /** \brief Destructor. */
  ~CartesianFilter() {}

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

  /** \brief Hash a point to the index of a grid cell.
   * \param[in] pt the point to hash
   * \param[in] inverse_leaf_size the array of 1/leaf_size
   * \param[in] min_b the minimum bin coordinate
   * \param[in] divb_mul the division multiplier
   */
  inline std::size_t
  hashPoint(const PointT& pt,
            const Eigen::Array4f& inverse_leaf_size,
            const Eigen::Vector4i& min_b,
            const Eigen::Vector4i& divb_mul) const
  {
    const std::size_t ijk0 = std::floor(pt.x * inverse_leaf_size[0]) - min_b[0];
    const std::size_t ijk1 = std::floor(pt.y * inverse_leaf_size[1]) - min_b[1];
    const std::size_t ijk2 = std::floor(pt.z * inverse_leaf_size[2]) - min_b[2];
    return ijk0 * divb_mul[0] + ijk1 * divb_mul[1] + ijk2 * divb_mul[2];
  }

  /** \brief Check if the leaf size is too small, given the range of the point cloud
   * \param[in] min_b the minimum bin coordinate
   * \param[in] min_b the minimum bin coordinate
   * \param[in] inverse_leaf_size the array of 1/leaf_size
   * \note Return false if hash range is larger than the range of size_t as the unsigned
   * int "wrap around" effect will occur during hashing a point
   */
  std::size_t
  checkHashRange(const Eigen::Vector4f& min_p,
                 const Eigen::Vector4f& max_p,
                 const Eigen::Array4f& inverse_leaf_size) const
  {
    // Pass inverse_leaf_size[2] = 0 for checking 2D grids
    const std::size_t dx = std::floor((max_p[0] - min_p[0]) * inverse_leaf_size[0]) + 1;
    const std::size_t dy = std::floor((max_p[1] - min_p[1]) * inverse_leaf_size[1]) + 1;
    const std::size_t dz = std::floor((max_p[2] - min_p[2]) * inverse_leaf_size[2]) + 1;

    // Check hashing range
    const size_t max_size = std::numeric_limits<std::size_t>::max();
    if (dy > max_size / dx || dx * dy > max_size / dz)
      return 0;
    else
      return dx * dy * dz;
  }

protected:
  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
  bool downsample_all_data_ = true;

  /** \brief Minimum number of points per voxel for the centroid to be computed */
  std::size_t min_points_per_voxel_ = 0;
};

} // namespace experimental
} // namespace pcl
