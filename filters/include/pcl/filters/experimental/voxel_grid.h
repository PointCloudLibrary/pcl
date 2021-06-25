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

  /** \brief Empty constructor. */
  VoxelStructT()
  : filter_name_("VoxelGrid")
  , leaf_size_(Eigen::Vector4f::Zero())
  , inverse_leaf_size_(Eigen::Array4f::Zero())
  , min_b_(Eigen::Vector4i::Zero())
  , max_b_(Eigen::Vector4i::Zero())
  , div_b_(Eigen::Vector4i::Zero())
  , divb_mul_(Eigen::Vector4i::Zero())
  , min_points_per_voxel_(0)
  {}

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
    return leaf_size_.head<3>();
  }

  /** \brief Get the minimum coordinates of the bounding box (after
   * filtering is performed).
   */
  inline Eigen::Vector3i
  getMinBoxCoordinates() const
  {
    return min_b_.head<3>();
  }

  /** \brief Get the minimum coordinates of the bounding box (after
   * filtering is performed).
   */
  inline Eigen::Vector3i
  getMaxBoxCoordinates() const
  {
    return max_b_.head<3>();
  }

  /** \brief Get the number of divisions along all 3 axes (after filtering
   * is performed).
   */
  inline Eigen::Vector3i
  getNrDivisions() const
  {
    return div_b_.head<3>();
  }

  /** \brief Get the multipliers to be applied to the grid coordinates in
   * order to find the centroid index (after filtering is performed).
   */
  inline Eigen::Vector3i
  getDivisionMultiplier() const
  {
    return divb_mul_.head<3>();
  }

  /** \brief Returns the index in the resulting downsampled cloud of the specified
   * point.
   *
   * \note for efficiency, user must make sure that the saving of the leaf layout is
   * enabled and filtering performed, and that the point is inside the grid, to avoid
   * invalid access (or use getGridCoordinates+getCentroidIndexAt)
   *
   * \param[in] p the point to get the index at
   */
  inline int
  getCentroidIndex(const PointT& p) const
  {
    return leaf_layout_.at(hashPoint(p));
  }

  /** \brief Returns the indices in the resulting downsampled cloud of the points at the
   * specified grid coordinates, relative to the grid coordinates of the specified point
   * (or -1 if the cell was empty/out of bounds). \param[in] reference_point the
   * coordinates of the reference point (corresponding cell is allowed to be empty/out
   * of bounds) \param[in] relative_coordinates matrix with the columns being the
   * coordinates of the requested cells, relative to the reference point's cell \note
   * for efficiency, user must make sure that the saving of the leaf layout is enabled
   * and filtering performed
   */
  inline std::vector<int>
  getNeighborCentroidIndices(const PointT& reference_point,
                             const Eigen::MatrixXi& relative_coordinates) const
  {
    Eigen::Vector4i ijk(
        static_cast<int>(std::floor(reference_point.x * inverse_leaf_size_[0])),
        static_cast<int>(std::floor(reference_point.y * inverse_leaf_size_[1])),
        static_cast<int>(std::floor(reference_point.z * inverse_leaf_size_[2])),
        0);
    Eigen::Array4i diff2min = min_b_ - ijk;
    Eigen::Array4i diff2max = max_b_ - ijk;
    std::vector<int> neighbors(relative_coordinates.cols());
    for (Eigen::Index ni = 0; ni < relative_coordinates.cols(); ni++) {
      Eigen::Vector4i displacement =
          (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
      // checking if the specified cell is in the grid
      if ((diff2min <= displacement.array()).all() &&
          (diff2max >= displacement.array()).all())
        neighbors[ni] = leaf_layout_[(
            (ijk + displacement - min_b_).dot(divb_mul_))]; // .at() can be omitted
      else
        neighbors[ni] = -1; // cell is out of bounds, consider it empty
    }
    return neighbors;
  }

  /** \brief Returns the layout of the leafs for fast access to cells relative to
   * current position. \note position at (i-min_x) + (j-min_y)*div_x +
   * (k-min_z)*div_x*div_y holds the index of the element at coordinates (i,j,k) in the
   * grid (-1 if empty)
   */
  inline std::vector<int>
  getLeafLayout() const
  {
    return leaf_layout_;
  }

  /** \brief Returns the corresponding (i,j,k) coordinates in the grid of point (x,y,z).
   * \param[in] x the X point coordinate to get the (i, j, k) index at
   * \param[in] y the Y point coordinate to get the (i, j, k) index at
   * \param[in] z the Z point coordinate to get the (i, j, k) index at
   */
  inline Eigen::Vector3i
  getGridCoordinates(float x, float y, float z) const
  {
    return Eigen::Vector3i(static_cast<int>(std::floor(x * inverse_leaf_size_[0])),
                           static_cast<int>(std::floor(y * inverse_leaf_size_[1])),
                           static_cast<int>(std::floor(z * inverse_leaf_size_[2])));
  }

  /** \brief Returns the index in the downsampled cloud corresponding to a given set of
   * coordinates. \param[in] ijk the coordinates (i,j,k) in the grid (-1 if empty)
   */
  inline int
  getCentroidIndexAt(const Eigen::Vector3i& ijk) const
  {
    int idx = ((Eigen::Vector4i() << ijk, 0).finished() - min_b_).dot(divb_mul_);
    if (idx < 0 ||
        idx >= static_cast<int>(
                   leaf_layout_.size())) // this checks also if leaf_layout_.size () ==
                                         // 0 i.e. everything was computed as needed
    {
      // if (verbose)
      //  PCL_ERROR ("[pcl::%s::getCentroidIndexAt] Specified coordinate is outside grid
      //  bounds, or leaf layout is not saved, make sure to call setSaveLeafLayout(true)
      //  and filter(output) first!\n", getClassName ().c_str ());
      return (-1);
    }
    return leaf_layout_[idx];
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

    min_b_ = (min_p.array() * inverse_leaf_size_).floor().cast<int>();
    max_b_ = (max_p.array() * inverse_leaf_size_).floor().cast<int>();

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
  /** \brief The filter name. */
  const std::string filter_name_;

  /** \brief The size of a leaf. */
  Eigen::Vector4f leaf_size_;

  /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
  Eigen::Array4f inverse_leaf_size_;

  /** \brief The leaf layout information for fast access to cells relative to current
   * position **/
  std::vector<int> leaf_layout_;

  /** \brief The minimum and maximum bin coordinates, the number of divisions, and the
   * division multiplier. */
  Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

  /** \brief Minimum number of points per voxel for the centroid to be computed */
  std::size_t min_points_per_voxel_;
};

template <typename PointT>
using VoxelGrid = GridFilter<VoxelStructT<PointT>>;

} // namespace experimental
} // namespace pcl