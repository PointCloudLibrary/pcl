/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2021-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/experimental/cartesian_filter.h>
#include <pcl/filters/filter.h>

#include <boost/variant.hpp>

#include <unordered_map>

namespace pcl {
namespace experimental {

template <typename PointT>
struct Voxel {

  Voxel(const bool downsample_all_data) : downsample_all_data_(downsample_all_data)
  {
    num_pt_ = 0;
    if (downsample_all_data_)
      centroid_ = CentroidPoint<PointT>();
    else
      centroid_ = Eigen::Array4f::Zero();
  }

  inline void
  add(const PointT& pt)
  {
    num_pt_++;
    if (downsample_all_data_)
      boost::get<CentroidPoint<PointT>>(centroid_).add(pt);
    else
      boost::get<Eigen::Array4f>(centroid_) += pt.getArray4fMap();
  }

  inline PointT
  get() const
  {
    PointT pt;
    if (downsample_all_data_)
      boost::get<CentroidPoint<PointT>>(centroid_).get(pt);
    else
      pt.getArray4fMap() = boost::get<Eigen::Array4f>(centroid_) / num_pt_;
    return pt;
  }

  inline void
  clear()
  {
    num_pt_ = 0;
    if (downsample_all_data_)
      boost::get<CentroidPoint<PointT>>(centroid_).clear();
    else
      boost::get<Eigen::Array4f>(centroid_).setZero();
  }

  inline std::size_t
  size() const
  {
    return num_pt_;
  }

protected:
  const bool downsample_all_data_;
  boost::variant<CentroidPoint<PointT>, Eigen::Array4f> centroid_; // std::variant for C++17
  std::size_t num_pt_;
};

/**
 * \brief VoxelStruct defines the transformation operations and the voxel grid of
 * VoxelGrid filter
 * \ingroup filters
 */
template <typename VoxelT, typename PointT>
class VoxelStruct {
public:
  // read by CartesianFilter to deduce point type
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;
  using Grid = typename std::unordered_map<std::size_t, VoxelT>;
  using iterator = typename Grid::iterator;

  /** \brief Constructor.
   */
  VoxelStruct() { filter_name_ = "VoxelGrid"; }

  /** \brief Empty destructor. */
  ~VoxelStruct() {}

  /** \brief Get the number of non-empty voxels in the grid. */
  inline std::size_t
  size() const
  {
    return num_voxels_;
  }

  /** \brief Get the begin iterator of the grid. */
  inline iterator
  begin()
  {
    return grid_.begin();
  }

  /** \brief Get the end iterator of the grid. */
  inline iterator
  end()
  {
    return grid_.end();
  }

  /** \brief Set up the voxel grid variables needed for filtering.
   *  \param[in] transform_filter pointer to the TransformFilter object
   */
  template <template <typename> class FilterBase>
  inline bool
  setUp(CartesianFilter<FilterBase, VoxelStruct>& castesian_filter)
  {
    return setUp(castesian_filter.getInputCloud(),
                 castesian_filter.getDownsampleAllData(),
                 castesian_filter.getMinimumPointsNumberPerVoxel());
  }

  /** \brief Add a point to a voxel based on its value.
   * \param[in] pt the point to add to a voxel
   */
  inline void
  addPointToGrid(const PointT& pt)
  {
    const std::size_t h =
        hashPoint<PointT>(pt, inverse_leaf_size_, min_b_, divb_mul_[1], divb_mul_[2]);

    // TODO: try_emplace for c++17
    auto it = grid_.find(h);
    if (it == grid_.end())
      it = grid_.emplace(h, downsample_all_data_).first;
    it->second.add(pt);
  }

  /** \brief Filter out or compute the centroid of a voxel
   * \param[in] grid_it the iterator of the voxel
   */
  inline experimental::optional<PointT>
  filterGrid(const iterator grid_it)
  {
    const auto& voxel = grid_it->second;
    if (voxel.size() >= min_points_per_voxel_) {

      if (save_leaf_layout_)
        leaf_layout_[grid_it->first] = num_voxels_++;

      return voxel.get();
    }

    return boost::none;
  }

  /** \brief The filter name. */
  std::string filter_name_;

  /** \brief The size of a leaf. */
  Eigen::Vector4f leaf_size_ = Eigen::Vector4f::Zero();

  /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
  Eigen::Array4f inverse_leaf_size_ = Eigen::Array4f::Zero();

  /** \brief Set to true if leaf layout information needs to be saved in \a
   * leaf_layout_. */
  bool save_leaf_layout_ = false;

  /** \brief The leaf layout information for fast access to cells relative to current
   * position **/
  std::vector<int> leaf_layout_;

  /** \brief The minimum and maximum bin coordinates, the number of divisions, and the
   * division multiplier. */
  Eigen::Vector4i min_b_ = Eigen::Vector4i::Zero(), max_b_ = Eigen::Vector4i::Zero(),
                  div_b_ = Eigen::Vector4i::Zero(), divb_mul_ = Eigen::Vector4i::Zero();

  /** \brief Iterable grid object storing the data of voxels. */
  Grid grid_;

  /** \brief maximum load factor of the unordered_map type grid. */
  float grid_max_load_factor_ = 0.5;

protected:
  /** \brief Set up the voxel grid variables needed for filtering.
   * \param[in] input pointer to the input point cloud
   * \param[in] downsample_all_data the state of the internal downsampling parameter
   * \param[in] min_points_per_voxel minimum number of points required for a voxel to be
   * used
   */
  bool
  setUp(const PointCloudConstPtr input,
        const bool downsample_all_data,
        const std::size_t min_points_per_voxel)
  {
    num_voxels_ = 0;
    if (downsample_all_data_ != downsample_all_data)
      grid_ = Grid();
    else
      std::for_each(
          grid_.begin(), grid_.end(), [](auto& cell) { cell.second.clear(); });

    downsample_all_data_ = downsample_all_data;
    min_points_per_voxel_ = min_points_per_voxel;

    // Get the minimum and maximum dimensions
    Eigen::Vector4f min_p, max_p;
    getMinMax3D<PointT>(*input, min_p, max_p);

    // Compute the minimum and maximum bounding box values
    min_b_ = (min_p.array() * inverse_leaf_size_).floor().template cast<int>();
    max_b_ = (max_p.array() * inverse_leaf_size_).floor().template cast<int>();

    // Compute the number of divisions needed along all axis
    div_b_ = (max_b_ - min_b_).array() + 1;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // Check that the leaf size is not too small, given the size of the data
    const std::size_t hash_range =
        checkHashRange3D(min_p, max_p, inverse_leaf_size_.head<3>());
    if (hash_range != 0) {
      grid_.max_load_factor(grid_max_load_factor_);
      grid_.reserve(std::min(hash_range, input->size()) / grid_max_load_factor_);
    }
    else {
      PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. "
               "Integer indices would overflow.\n",
               filter_name_.c_str());
      return false;
    }

    if (save_leaf_layout_) {
      // TODO: stop using std::vector<int> for leaf_layout_ because the range of int is
      // smaller than size_t

      // Resizing won't reset old elements to -1.  If leaf_layout_ has been used
      // previously, it needs to be re-initialized to -1
      const std::size_t new_layout_size = div_b_[0] * div_b_[1] * div_b_[2];

      // This is the number of elements that need to be re-initialized to -1
      const std::size_t reset_size = std::min(new_layout_size, leaf_layout_.size());
      std::fill(leaf_layout_.begin(), leaf_layout_.begin() + reset_size, -1);

      try {
        leaf_layout_.resize(new_layout_size, -1);
      } catch (std::bad_alloc&) {
        throw PCLException(
            "VoxelGrid bin size is too low; impossible to allocate memory for layout",
            "voxel_grid.hpp",
            "applyFilter");
      } catch (std::length_error&) {
        throw PCLException(
            "VoxelGrid bin size is too low; impossible to allocate memory for layout",
            "voxel_grid.hpp",
            "applyFilter");
      }
    }

    return true;
  }

  /** \brief Total number of voxels in the output cloud */
  std::size_t num_voxels_;

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
  bool downsample_all_data_ = true;

  /** \brief Minimum number of points per voxel for the centroid to be computed */
  std::size_t min_points_per_voxel_;
};

/**
 * \brief VoxelFilter represents the base class for voxel filters.
 * \details Used as the base class for grid based filters: VoxelGrid, VoxelGridLabel,
 * ApproximateVoxelGrid, etc.
 * \ingroup filters
 */
template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class VoxelFilter : public CartesianFilter<pcl::Filter, GridStruct> {
  using CartesianFilter<pcl::Filter, GridStruct>::getGridStruct;

public:
  /** \brief Set the voxel grid leaf size.
   * \param[in] leaf_size the voxel grid leaf size
   */
  inline void
  setLeafSize(const Eigen::Vector4f& leaf_size)
  {
    Eigen::Vector4f& leaf_size_ = getGridStruct().leaf_size_;
    Eigen::Array4f& inverse_leaf_size_ = getGridStruct().inverse_leaf_size_;

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
  setLeafSize(const float lx, const float ly, const float lz)
  {
    Eigen::Vector4f& leaf_size_ = getGridStruct().leaf_size_;
    Eigen::Array4f& inverse_leaf_size_ = getGridStruct().inverse_leaf_size_;

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
    const Eigen::Vector4f& leaf_size_ = getGridStruct().leaf_size_;
    return leaf_size_.head<3>();
  }

  /** \brief Get the minimum coordinates of the bounding box (after filtering is
   * performed).
   */
  inline Eigen::Vector3i
  getMinBoxCoordinates() const
  {
    const Eigen::Vector4i& min_b_ = getGridStruct().min_b_;
    return min_b_.head<3>();
  }

  /** \brief Get the minimum coordinates of the bounding box (after filtering is
   * performed).
   */
  inline Eigen::Vector3i
  getMaxBoxCoordinates() const
  {
    const Eigen::Vector4i max_b_ = getGridStruct().max_b_;
    return max_b_.head<3>();
  }

  /** \brief Get the number of divisions along all 3 axes (after filtering is
   * performed).
   */
  inline Eigen::Vector3i
  getNrDivisions() const
  {
    const Eigen::Vector4i& div_b_ = getGridStruct().div_b_;
    return div_b_.head<3>();
  }

  /** \brief Get the multipliers to be applied to the grid coordinates in order to find
   * the centroid index (after filtering is performed).
   */
  inline Eigen::Vector3i
  getDivisionMultiplier() const
  {
    const Eigen::Vector4i& divb_mul_ = getGridStruct().divb_mul_;
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
  getCentroidIndex(const PointT& pt) const
  {
    const std::vector<int>& leaf_layout_ = getGridStruct().leaf_layout_;
    const Eigen::Array4f& inverse_leaf_size_ = getGridStruct().inverse_leaf_size_;
    const Eigen::Vector4i& min_b_ = getGridStruct().min_b_;
    const Eigen::Vector4i& divb_mul_ = getGridStruct().divb_mul_;

    return leaf_layout_.at(
        hashPoint(pt, inverse_leaf_size_, min_b_, divb_mul_[1], divb_mul_[2]));
  }

  /** \brief Returns the indices in the resulting downsampled cloud of the points at the
   * specified grid coordinates, relative to the grid coordinates of the specified point
   * (or -1 if the cell was empty/out of bounds).
   * \param[in] x the X coordinate of the reference point (corresponding cell is allowed
   * to be empty/out of bounds)
   * \param[in] y the Y coordinate of the reference point (corresponding cell is allowed
   * to be empty/out of bounds)
   * \param[in] z the Z coordinate of the reference point (corresponding cell is allowed
   * to be empty/out of bounds)
   * \param[out] relative_coordinates matrix with the columns being the coordinates of
   * the requested cells, relative to the reference point's cell
   * \note for efficiency,user must make sure that the saving of the leaf layout is
   * enabled and filtering performed
   */
  inline std::vector<int>
  getNeighborCentroidIndices(const float x,
                             const float y,
                             const float z,
                             const Eigen::MatrixXi& relative_coordinates) const
  {
    const std::vector<int>& leaf_layout_ = getGridStruct().leaf_layout_;
    const Eigen::Vector4i& min_b_ = getGridStruct().min_b_;
    const Eigen::Vector4i& max_b_ = getGridStruct().min_b_;
    const Eigen::Vector4i& divb_mul_ = getGridStruct().divb_mul_;

    const Eigen::Vector4i ijk(
        (Eigen::Vector4i() << getGridCoordinates(x, y, z), 0).finished());
    const Eigen::Array4i diff2min = min_b_ - ijk;
    const Eigen::Array4i diff2max = max_b_ - ijk;

    std::vector<int> neighbors(relative_coordinates.cols());
    for (Eigen::Index ni = 0; ni < relative_coordinates.cols(); ni++) {
      const Eigen::Vector4i displacement =
          (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();

      // checking if the specified cell is in the grid
      if ((diff2min <= displacement.array()).all() &&
          (diff2max >= displacement.array()).all())
        // .at() can be omitted
        neighbors[ni] = leaf_layout_[((ijk + displacement - min_b_).dot(divb_mul_))];
      else
        // cell is out of bounds, consider it empty
        neighbors[ni] = -1;
    }
    return neighbors;
  }

  /** \brief Returns the indices in the resulting downsampled cloud of the points at the
   * specified grid coordinates, relative to the grid coordinates of the specified point
   * (or -1 if the cell was empty/out of bounds).
   * \param[in] x the X coordinate of the reference point (corresponding cell is allowed
   * to be empty/out of bounds)
   * \param[in] y the Y coordinate of the reference point (corresponding cell is allowed
   * to be empty/out of bounds)
   * \param[in] z the Z coordinate of the reference point (corresponding cell is allowed
   * to be empty/out of bounds)
   * \param[out] relative_coordinates vector with the elements being the coordinates of
   * the requested cells, relative to the reference point's cell
   * \note for efficiency, user must make sure that the saving of the leaf layout is
   * enabled and filtering performed
   */
  inline std::vector<int>
  getNeighborCentroidIndices(
      const float x,
      const float y,
      const float z,
      const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>&
          relative_coordinates) const
  {
    const std::vector<int>& leaf_layout_ = getGridStruct().leaf_layout_;
    const Eigen::Vector4i& min_b_ = getGridStruct().min_b_;
    const Eigen::Vector4i& divb_mul_ = getGridStruct().divb_mul_;

    const Eigen::Vector4i ijk(
        (Eigen::Vector4i() << getGridCoordinates(x, y, z), 0).finished());
    std::vector<int> neighbors;
    neighbors.reserve(relative_coordinates.size());
    for (const auto& relative_coordinate : relative_coordinates)
      neighbors.push_back(
          leaf_layout_[(ijk + (Eigen::Vector4i() << relative_coordinate, 0).finished() -
                        min_b_)
                           .dot(divb_mul_)]);
    return neighbors;
  }

  /** \brief Returns the indices in the resulting downsampled cloud of the points at the
   * specified grid coordinates, relative to the grid coordinates of the specified point
   * (or -1 if the cell was empty/out of bounds).
   * \param[in] reference_point the coordinates of the reference point (corresponding
   * cell is allowed to be empty/out of bounds)
   * \param[in] relative_coordinates matrix with the columns being the coordinates of
   * the requested cells, relative to the reference point's cell
   * \note for efficiency, user must make sure that the saving of the leaf layout is
   * enabled and filtering performed
   */
  inline std::vector<int>
  getNeighborCentroidIndices(const PointT& reference_point,
                             const Eigen::MatrixXi& relative_coordinates) const
  {
    return getNeighborCentroidIndices(
        reference_point.x, reference_point.y, reference_point.z, relative_coordinates);
  }

  /** \brief Set to true if leaf layout information needs to be saved for later access.
   * \param[in] save_leaf_layout the new value (true/false)
   */
  inline void
  setSaveLeafLayout(const bool save_leaf_layout)
  {
    getGridStruct().save_leaf_layout_ = save_leaf_layout;
  }

  /** \brief Returns true if leaf layout information will to be saved for later access.
   */
  inline bool
  getSaveLeafLayout() const
  {
    return getGridStruct().save_leaf_layout_;
  }

  /** \brief Returns the layout of the leafs for fast access to cells relative to
   * current position.
   * \note position at (i-min_x) + (j-min_y)*div_x + (k-min_z)*div_x*div_y holds the
   * index of the element at coordinates (i,j,k) in the grid (-1 if empty)
   */
  inline std::vector<int>
  getLeafLayout() const
  {
    return getGridStruct().leaf_layout_;
  }

  /** \brief Returns the corresponding (i,j,k) coordinates in the grid of point (x,y,z).
   * \param[in] x the X point coordinate to get the (i, j, k) index at
   * \param[in] y the Y point coordinate to get the (i, j, k) index at
   * \param[in] z the Z point coordinate to get the (i, j, k) index at
   */
  inline Eigen::Vector3i
  getGridCoordinates(const float x, const float y, const float z) const
  {
    const Eigen::Array4f& inverse_leaf_size_ = getGridStruct().inverse_leaf_size_;
    return Eigen::Vector3i(static_cast<int>(std::floor(x * inverse_leaf_size_[0])),
                           static_cast<int>(std::floor(y * inverse_leaf_size_[1])),
                           static_cast<int>(std::floor(z * inverse_leaf_size_[2])));
  }

  /** \brief Returns the index in the downsampled cloud corresponding to a given set of
   * coordinates.
   * \param[in] ijk the coordinates (i,j,k) in the grid (-1 if empty)
   */
  inline int
  getCentroidIndexAt(const Eigen::Vector3i& ijk) const
  {
    const std::vector<int>& leaf_layout_ = getGridStruct().leaf_layout_;
    const Eigen::Vector4i& min_b_ = getGridStruct().min_b_;
    const Eigen::Vector4i& divb_mul_ = getGridStruct().divb_mul_;

    const int idx = ((Eigen::Vector4i() << ijk, 0).finished() - min_b_).dot(divb_mul_);
    // this checks also if leaf_layout_.size () == 0 i.e. everything was computed as
    // needed
    if (idx < 0 || idx >= static_cast<int>(leaf_layout_.size()))
      return -1;

    return leaf_layout_[idx];
  }

  /** \brief Return the maximum load factor of unordered_map type grid
   */
  inline float
  getMaxLoadFactor()
  {
    return getGridStruct().max_load_factor_;
  }

  /** \brief Set the maximum load factor of unordered_map type grid
   * \note The smaller is the factor, the faster is the overall runtime. It is a trade
   * off of memory verus speed.
   */
  inline void
  setMaxLoadFactor(const float load_factor)
  {
    getGridStruct().max_load_factor_ = load_factor;
  }
};

/** \brief VoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples
 * + filters the data.
 *
 * The VoxelGrid class creates a *3D voxel grid* (think about a voxel
 * grid as a set of tiny 3D boxes in space) over the input point cloud data.
 * Then, in each *voxel* (i.e., 3D box), all the points present will be
 * approximated (i.e., *downsampled*) with their centroid. This approach is
 * a bit slower than approximating them with the center of the voxel, but it
 * represents the underlying surface more accurately.
 *
 * \ingroup filters
 */
template <typename PointT>
using VoxelGrid = VoxelFilter<VoxelStruct<Voxel<PointT>, PointT>>;

} // namespace experimental
} // namespace pcl