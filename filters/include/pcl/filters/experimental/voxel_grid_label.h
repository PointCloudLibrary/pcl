/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/experimental/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

#include <map>

namespace pcl {
namespace experimental {

struct LabeledVoxel {
  union Centroid {
    Centroid() {}
    ~Centroid() {}

    CentroidPoint<PointXYZRGBL> all_fields;
    Eigen::Array4f xyz;
  };

  LabeledVoxel(const bool downsample_all_data)
  : downsample_all_data_(downsample_all_data)
  {
    num_pt_ = 0;
    if (downsample_all_data_)
      centroid_.all_fields = CentroidPoint<PointXYZRGBL>();
    else
      centroid_.xyz = Eigen::Array4f::Zero();
  }
  ~LabeledVoxel()
  {
    if (downsample_all_data_)
      centroid_.all_fields.~CentroidPoint<PointXYZRGBL>();
    else
      centroid_.xyz.~Array();
  }

  inline void
  add(const PointXYZRGBL& pt)
  {
    num_pt_++;
    if (downsample_all_data_) {
      centroid_.all_fields.add(pt);
    }
    else {
      centroid_.xyz += pt.getArray4fMap();
      labels[pt.label]++;
    }
  }

  inline PointXYZRGBL
  get() const
  {
    PointXYZRGBL pt;
    if (downsample_all_data_)
      centroid_.all_fields.get(pt);
    else {
      pt.getArray4fMap() = centroid_.xyz / num_pt_;
      std::size_t max = 0;
      for (const auto& label : labels) {
        if (label.second > max) {
          max = label.second;
          pt.label = label.first;
        }
      }
    }

    return pt;
  }

  inline void
  clear()
  {
    num_pt_ = 0;
    if (downsample_all_data_) {
      centroid_.all_fields.clear();
    }
    else {
      centroid_.xyz.setZero();
      labels.clear();
    }
  }

  inline std::size_t
  size() const
  {
    return num_pt_;
  }

private:
  const bool downsample_all_data_;
  Centroid centroid_;
  std::size_t num_pt_;
  std::map<std::uint32_t, std::size_t> labels;
};

/**
 * \brief LabeledVoxelStruct defines the transformation operations and the voxel grid of
 * VoxelGridLabel filter
 * \ingroup filters
 */
class LabeledVoxelStruct : public VoxelStruct<LabeledVoxel, PointXYZRGBL> {
public:
  using VoxelStruct<LabeledVoxel, PointXYZRGBL>::filter_name_;

  /** \brief Constructor. */
  LabeledVoxelStruct() { filter_name_ = "VoxelGridLabel"; }

  /** \brief Empty destructor. */
  ~LabeledVoxelStruct() {}

  /** \brief Set up the voxel grid variables needed for filtering.
   *  \param[in] transform_filter pointer to the TransformFilter object
   */
  bool
  setUp(TransformFilter<Filter, LabeledVoxelStruct> const* transform_filter)
  {
    auto cartesian_filter =
        static_cast<CartesianFilter<Filter, LabeledVoxelStruct> const*>(
            transform_filter);
    return VoxelStruct<LabeledVoxel, PointXYZRGBL>::setUp(
        cartesian_filter->getInputCloud(),
        cartesian_filter->getDownsampleAllData(),
        cartesian_filter->getMinimumPointsNumberPerVoxel());
  }
};

/** \brief VoxelGridLabel assembles a local 3D grid over a PointXYZRGBL PointCloud, and
 * downsamples + filters the data. Each cell is labeled by majority vote according
 * to the label of the points.
 * \ingroup filters
 */
using VoxelGridLabel = VoxelFilter<LabeledVoxelStruct>;

} // namespace experimental
} // namespace pcl