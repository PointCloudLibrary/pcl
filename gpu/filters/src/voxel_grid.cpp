/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026, Open Perception, Inc.
 *
 *  All rights reserved
 */

#include <pcl/common/point_tests.h>
#include <pcl/console/print.h>
#include <pcl/gpu/filters/voxel_grid.h>

#include "internal.hpp"

#include <algorithm>
#include <cstdint>

pcl::gpu::VoxelGrid::VoxelGrid()
: leaf_size_(Eigen::Vector3f::Zero())
, min_points_per_voxel_(1)
, impl_(std::make_shared<pcl::device::VoxelGridImpl>())
{
  static_assert(sizeof(PointType) == sizeof(float4),
                "pcl::PointXYZ and float4 have to be of the same size");
}

pcl::gpu::VoxelGrid::~VoxelGrid() = default;

void
pcl::gpu::VoxelGrid::setLeafSize(const float lx, const float ly, const float lz)
{
  leaf_size_ = Eigen::Vector3f(lx, ly, lz);
}

void
pcl::gpu::VoxelGrid::setLeafSize(const Eigen::Vector3f& leaf_size)
{
  leaf_size_ = leaf_size;
}

Eigen::Vector3f
pcl::gpu::VoxelGrid::getLeafSize() const
{
  return leaf_size_;
}

void
pcl::gpu::VoxelGrid::setMinimumPointsNumberPerVoxel(const unsigned int min_points_per_voxel)
{
  min_points_per_voxel_ = min_points_per_voxel;
}

unsigned int
pcl::gpu::VoxelGrid::getMinimumPointsNumberPerVoxel() const
{
  return min_points_per_voxel_;
}

void
pcl::gpu::VoxelGrid::setInputCloud(const PointCloud& input)
{
  input_ = input;
  input_header_ = pcl::PCLHeader();
  input_sensor_origin_ = Eigen::Vector4f::Zero();
  input_sensor_orientation_ = Eigen::Quaternionf::Identity();
}

void
pcl::gpu::VoxelGrid::setInputCloud(const PointCloudHost& input)
{
  input_header_ = input.header;
  input_sensor_origin_ = input.sensor_origin_;
  input_sensor_orientation_ = input.sensor_orientation_;

  // Detach from any caller-owned device buffer before uploading the host cloud.
  input_ = PointCloud();
  if (input.empty())
    return;
  input_.upload(input.data(), input.size());
}

void
pcl::gpu::VoxelGrid::filter(PointCloud& output)
{
  if (input_.empty()) {
    PCL_WARN("[pcl::gpu::VoxelGrid::filter] No input dataset given!\n");
    output = PointCloud();
    return;
  }

  if (leaf_size_.minCoeff() <= 0.f) {
    PCL_ERROR("[pcl::gpu::VoxelGrid::filter] Leaf size is invalid, use setLeafSize() "
              "to set it!\n");
    output = PointCloud();
    return;
  }

  // Keep a reference to the input buffer, the caller may pass the input cloud as output
  const PointCloud input = input_;

  bool voxel_index_overflow = false;
  impl_->applyFilter(reinterpret_cast<const DeviceArray<float4>&>(input),
                     make_float3(leaf_size_[0], leaf_size_[1], leaf_size_[2]),
                     min_points_per_voxel_,
                     reinterpret_cast<DeviceArray<float4>&>(output),
                     voxel_index_overflow);

  if (voxel_index_overflow) {
    PCL_WARN(
        "[pcl::gpu::VoxelGrid::filter] Leaf size is too small for the input dataset. "
        "Integer indices would overflow.\n");
    // Like pcl::VoxelGrid, return the input cloud unchanged
    input.copyTo(output);
  }
}

void
pcl::gpu::VoxelGrid::filter(PointCloudHost& output)
{
  PointCloud output_device;
  filter(output_device);

  const std::size_t output_size = output_device.size();
  output.points.resize(output_size);
  output.width = static_cast<std::uint32_t>(output_size);
  output.height = 1;
  output.header = input_header_;
  output.sensor_origin_ = input_sensor_origin_;
  output.sensor_orientation_ = input_sensor_orientation_;
  if (output_size > 0)
    output_device.download(output.points.data());
  // Index overflow returns the original cloud, which may contain non-finite points.
  output.is_dense = std::all_of(output.begin(), output.end(), [](const PointType& point) {
    return pcl::isFinite(point);
  });
}
