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

/** \brief Hash a point to the index of a 2D grid cell.
 * \param[in] pt the point to hash
 * \param[in] inverse_leaf_size the array of 1/leaf_size
 * \param[in] min_b the minimum bin coordinate
 * \param[in] divb_mul_y division multiplier of the second dimension
 */
template <typename PointT>
inline std::size_t
hashPoint(const PointT& pt,
          const Eigen::Array4f& inverse_leaf_size,
          const Eigen::Vector4i& min_b,
          const std::size_t divb_mul_y)
{
  const std::size_t ijk0 = std::floor(pt.x * inverse_leaf_size[0]) - min_b[0];
  const std::size_t ijk1 = std::floor(pt.y * inverse_leaf_size[1]) - min_b[1];
  return ijk0 + ijk1 * divb_mul_y;
}

/** \brief Hash a point to the index of a 3D grid cell.
 * \param[in] pt the point to hash
 * \param[in] inverse_leaf_size the array of 1/leaf_size
 * \param[in] min_b the minimum bin coordinate
 * \param[in] divb_mul_y division multiplier of the second dimension
 * \param[in] divb_mul_z division multiplier of the third dimension
 */
template <typename PointT>
inline std::size_t
hashPoint(const PointT& pt,
          const Eigen::Array4f& inverse_leaf_size,
          const Eigen::Vector4i& min_b,
          const std::size_t divb_mul_y,
          const std::size_t divb_mul_z)
{
  const std::size_t ijk0 = std::floor(pt.x * inverse_leaf_size[0]) - min_b[0];
  const std::size_t ijk1 = std::floor(pt.y * inverse_leaf_size[1]) - min_b[1];
  const std::size_t ijk2 = std::floor(pt.z * inverse_leaf_size[2]) - min_b[2];
  return ijk0 + ijk1 * divb_mul_y + ijk2 * divb_mul_z;
}

/** \brief Check if the leaf size is too small, given the range of the point cloud
 * \param[in] min_b the minimum bin coordinate
 * \param[in] min_b the minimum bin coordinate
 * \param[in] inverse_leaf_size 1/leaf_size of dimension x
 * \param[in] inverse_leaf_size 1/leaf_size of dimension y
 * \note Return false if hash range is larger than the range of size_t, as the
 * unsigned int "wrap around" effect will occur during hashing a point.
 */
std::size_t
checkHashRange(const Eigen::Vector4f& min_p,
               const Eigen::Vector4f& max_p,
               const float inverse_leaf_size_x,
               const float inverse_leaf_size_y)
{
  const float max_float = std::numeric_limits<float>::max();
  // Overflow when
  // a > 0 && b < 0 && a - b > max
  // -> - b > max - a
  if ((max_p[0] > 0. && min_p[0] < 0. && -min_p[0] > max_float - max_p[0]) ||
      (max_p[1] > 0. && min_p[1] < 0. && -min_p[1] > max_float - max_p[1]))
    return 0;

  const float range_x = max_p[0] - min_p[0];
  const float range_y = max_p[1] - min_p[1];

  const std::size_t max_size = std::numeric_limits<std::size_t>::max();
  // Check the number of bins of each dimension
  if (range_x > (max_size - 1) / inverse_leaf_size_x ||
      range_y > (max_size - 1) / inverse_leaf_size_y)
    return 0;

  const std::size_t dx =
      static_cast<std::size_t>(std::floor(range_x * inverse_leaf_size_x)) + 1;
  const std::size_t dy =
      static_cast<std::size_t>(std::floor(range_y * inverse_leaf_size_y)) + 1;

  // Check hashing range
  if (dy > max_size / dx)
    return 0;
  else
    return dx * dy;
}

/** \brief Check if the leaf size is too small, given the range of the point cloud
 * \param[in] min_b the minimum bin coordinate
 * \param[in] min_b the minimum bin coordinate
 * \param[in] inverse_leaf_size 1/leaf_size of dimension x
 * \param[in] inverse_leaf_size 1/leaf_size of dimension y
 * \param[in] inverse_leaf_size 1/leaf_size of dimension z
 * \note Return false if hash range is larger than the range of size_t, as the
 * unsigned int "wrap around" effect will occur during hashing a point.
 */
std::size_t
checkHashRange(const Eigen::Vector4f& min_p,
               const Eigen::Vector4f& max_p,
               const float inverse_leaf_size_x,
               const float inverse_leaf_size_y,
               const float inverse_leaf_size_z)
{
  const float max_float = std::numeric_limits<float>::max();
  // Overflow when
  // a > 0 && b < 0 && a - b > max
  // -> - b > max - a
  if ((max_p[0] > 0. && min_p[0] < 0. && -min_p[0] > max_float - max_p[0]) ||
      (max_p[1] > 0. && min_p[1] < 0. && -min_p[1] > max_float - max_p[1]) ||
      (max_p[2] > 0. && min_p[2] < 0. && -min_p[2] > max_float - max_p[2]))
    return 0;

  const float range_x = max_p[0] - min_p[0];
  const float range_y = max_p[1] - min_p[1];
  const float range_z = max_p[2] - min_p[2];

  const std::size_t max_size = std::numeric_limits<std::size_t>::max();
  // Check the number of bins of each dimension
  if (range_x > (max_size - 1) / inverse_leaf_size_x ||
      range_y > (max_size - 1) / inverse_leaf_size_y ||
      range_z > (max_size - 1) / inverse_leaf_size_z)
    return 0;

  const std::size_t dx =
      static_cast<std::size_t>(std::floor(range_x * inverse_leaf_size_x)) + 1;
  const std::size_t dy =
      static_cast<std::size_t>(std::floor(range_y * inverse_leaf_size_y)) + 1;
  const std::size_t dz =
      static_cast<std::size_t>(std::floor(range_z * inverse_leaf_size_y)) + 1;

  // Check hashing range
  if (dy > max_size / dx || dx * dy > max_size / dz)
    return 0;
  else
    return dx * dy * dz;
}

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

protected:
  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
  bool downsample_all_data_ = true;

  /** \brief Minimum number of points per voxel for the centroid to be computed */
  std::size_t min_points_per_voxel_ = 0;
};

} // namespace experimental
} // namespace pcl
