/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026, Open Perception, Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/gpu/containers/device_array.h>

#include <cuda_runtime.h>

#include <cstddef>

namespace pcl::device {
/** \brief GPU implementation of the voxel grid filter.
 *
 * The class holds the device memory that is reused by consecutive filter calls, so that
 * the buffers only have to be (re)allocated if the number of points of the input cloud
 * changes. */
class VoxelGridImpl {
public:
  /** \brief Applies the voxel grid filter to a point cloud.
   *
   * The points have to be passed as float4 arrays (as pcl::PointXYZ has a size of 16
   * bytes, it is binary compatible to float4). All points with non-finite coordinates
   * are discarded. If the voxel indices would overflow, no output is computed and
   * voxel_index_overflow is set to true.
   *
   * \param[in] input the input cloud
   * \param[in] leaf_size the leaf size of the voxel grid
   * \param[in] min_points_per_voxel the minimum number of points per voxel
   * \param[out] output the downsampled cloud
   * \param[out] voxel_index_overflow true if the voxel indices would overflow
   */
  void
  applyFilter(const DeviceArray<float4>& input,
              const float3& leaf_size,
              unsigned int min_points_per_voxel,
              DeviceArray<float4>& output,
              bool& voxel_index_overflow);

private:
  /** \brief Voxel index of every point, afterwards sorted. */
  DeviceArray<unsigned int> voxel_indices;
  /** \brief Index of every point, sorted together with voxel_indices. */
  DeviceArray<unsigned int> point_indices;
  /** \brief Number of points of every voxel, ordered by the voxel index. */
  DeviceArray<unsigned int> run_counts;
  /** \brief Index of the first point of every voxel in the sorted arrays. */
  DeviceArray<unsigned int> run_starts;
  /** \brief 1 if the respective voxel is part of the output cloud, 0 otherwise. */
  DeviceArray<unsigned int> run_flags;
  /** \brief Output position of every voxel, obtained by a prefix sum over run_flags. */
  DeviceArray<unsigned int> run_positions;

  /** \brief Runs the filter pipeline and returns the downsampled cloud. */
  DeviceArray<float4>
  run(const DeviceArray<float4>& input,
      const float3& leaf_size,
      unsigned int min_points_per_voxel,
      bool& voxel_index_overflow);
};
} // namespace pcl::device
