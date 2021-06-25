/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/experimental/grid_filter.h>

#include <boost/optional.hpp> // std::optional for C++17

#include <unordered_map>

namespace pcl {
namespace experimental {

struct Voxel {
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  std::size_t num_pt = 0;
};

template <typename PointT>
class VoxelStructT {
public:
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  using Grid = std::unordered_map<std::size_t, Voxel>;

  /** \brief Set the minimum number of points required for a voxel to be used.
   * \param[in] min_points_per_voxel the minimum number of points for required for a
   * voxel to be used
   */
  inline void
  setMinimumPointsNumberPerVoxel(unsigned int min_points_per_voxel)
  {
    min_points_per_voxel_ = min_points_per_voxel;
  }

  /** \brief Set the voxel grid leaf size.
   * \param[in] leaf_size the voxel grid leaf size
   */
  inline void
  setLeafSize(const Eigen::Vector4f& leaf_size)
  {
    leaf_size_ = leaf_size;
    // Avoid division errors
    if (leaf_size_[3] == 0)
      leaf_size_[3] = 1;
    // Use multiplications instead of divisions
    inverse_leaf_size_ = 1 / leaf_size_.array();
  }

  /** \brief Set the voxel grid leaf size.
   * \param[in] lx the leaf size for X
   * \param[in] ly the leaf size for Y
   * \param[in] lz the leaf size for Z
   */
  inline void
  setLeafSize(float lx, float ly, float lz)
  {
    leaf_size_[0] = lx;
    leaf_size_[1] = ly;
    leaf_size_[2] = lz;
    // Avoid division errors
    if (leaf_size_[3] == 0)
      leaf_size_[3] = 1;
    // Use multiplications instead of divisions
    inverse_leaf_size_ = 1 / leaf_size_.array();
  }

  /** \brief Get the voxel grid leaf size. */
  inline Eigen::Vector3f
  getLeafSize() const
  {
    return (leaf_size_.head<3>());
  }

  /** \brief Get the minimum coordinates of the bounding box (after
   * filtering is performed).
   */
  inline Eigen::Vector3i
  getMinBoxCoordinates() const
  {
    return (min_b_.head<3>());
  }

  /** \brief Get the minimum coordinates of the bounding box (after
   * filtering is performed).
   */
  inline Eigen::Vector3i
  getMaxBoxCoordinates() const
  {
    return (max_b_.head<3>());
  }

  /** \brief Get the number of divisions along all 3 axes (after filtering
   * is performed).
   */
  inline Eigen::Vector3i
  getNrDivisions() const
  {
    return (div_b_.head<3>());
  }

  /** \brief Get the multipliers to be applied to the grid coordinates in
   * order to find the centroid index (after filtering is performed).
   */
  inline Eigen::Vector3i
  getDivisionMultiplier() const
  {
    return (divb_mul_.head<3>());
  }

protected:
  void
  setUp(const GridFilter<VoxelStructT>* grid_filter)
  {
    double filter_limit_min, filter_limit_max;
    grid_filter->getFilterLimits(filter_limit_min, filter_limit_max);

    const PointCloudConstPtr input = grid_filter->getInputCloud();
    const IndicesConstPtr indices = grid_filter->getIndices();
    const std::string& filter_field_name = grid_filter->getFilterFieldName();

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    // if (!filter_field_name.empty()) // If we don't want to process the entire
    // cloud...
    //   getMinMax3D<PointT>(input,
    //                       *indices,
    //                       filter_field_name,
    //                       static_cast<float>(filter_limit_min),
    //                       static_cast<float>(filter_limit_max),
    //                       min_p,
    //                       max_p,
    //                       grid_filter->getFilterLimitsNegative());
    // else
    getMinMax3D<PointT>(*input, *indices, min_p, max_p);

    min_b_ = (min_p.array() * inverse_leaf_size_.array()).floor().cast<int>();
    max_b_ = (max_p.array() * inverse_leaf_size_.array()).floor().cast<int>();

    div_b_ = (max_b_ - min_b_).array() + 1;
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);
  }

  // accessing GridFilter
  const auto
  getDerived()
  {
    return static_cast<GridFilter<VoxelStructT>*>(this);
  }

  std::size_t
  hashPoint(const PointT& pt)
  {
    const std::size_t ijk0 = std::floor(pt.x * inverse_leaf_size_[0]) - min_b_[0];
    const std::size_t ijk1 = std::floor(pt.y * inverse_leaf_size_[1]) - min_b_[1];
    const std::size_t ijk2 = std::floor(pt.z * inverse_leaf_size_[2]) - min_b_[2];
    return ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
  }

  void
  addPointToGrid(Grid& grid, const PointT& pt)
  {
    // can be batch/running mode
    const std::size_t h = hashPoint(pt);
    grid[h].centroid += Eigen::Vector3f{pt.x, pt.y, pt.z};
    grid[h].num_pt++;
  }

  boost::optional<PointT>
  filterGrid(Grid::iterator grid_it, Grid& grid)
  {
    // Suppress unused warning
    (void)grid;

    auto& voxel = grid_it->second;
    if (voxel.num_pt >= min_points_per_voxel_) {
      voxel.centroid.array() /= voxel.num_pt;
      return PointT(voxel.centroid[0], voxel.centroid[1], voxel.centroid[2]);
    }

    return boost::none;
  }

protected:
  const std::string filter_name_ = "VoxelGrid";

  /** \brief The minimum and maximum bin coordinates, the number of divisions, and the
   * division multiplier. */
  Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

  /** \brief The size of a leaf. */
  Eigen::Vector4f leaf_size_;

  /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
  Eigen::Vector4f inverse_leaf_size_;

  /** \brief Minimum number of points per voxel for the centroid to be computed */
  std::size_t min_points_per_voxel_;
};

template <typename PointT>
using VoxelGrid = GridFilter<VoxelStructT<PointT>>;

} // namespace experimental
} // namespace pcl