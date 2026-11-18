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
#include <pcl/memory.h>
#include <pcl/pcl_exports.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <memory>

namespace pcl {
namespace device {
/** \brief GPU implementation of the voxel grid filter, used by pcl::gpu::VoxelGrid. */
class VoxelGridImpl;
} // namespace device

namespace gpu {
/** \brief VoxelGrid assembles a local 3D grid over a given point cloud, and downsamples
 * and filters the data on the GPU.
 *
 * Only pcl::PointXYZ clouds are supported.
 *
 * \ingroup filters
 */
class PCL_EXPORTS VoxelGrid {
public:
  using PointType = pcl::PointXYZ; // todo: support
  using PointCloud = DeviceArray<PointType>;
  using PointCloudHost = pcl::PointCloud<PointType>;

  using Ptr = shared_ptr<VoxelGrid>;
  using ConstPtr = shared_ptr<const VoxelGrid>;

  /** \brief Empty constructor. */
  VoxelGrid();

  /** \brief Destructor. */
  ~VoxelGrid();

  VoxelGrid(const VoxelGrid&) = delete;
  VoxelGrid&
  operator=(const VoxelGrid&) = delete;

  /** \brief Set the voxel grid leaf size.
   * \param[in] lx the leaf size for X
   * \param[in] ly the leaf size for Y
   * \param[in] lz the leaf size for Z
   */
  void
  setLeafSize(float lx, float ly, float lz);

  /** \brief Set the voxel grid leaf size.
   * \param[in] leaf_size the voxel grid leaf size
   */
  void
  setLeafSize(const Eigen::Vector3f& leaf_size);

  /** \brief Get the voxel grid leaf size. */
  Eigen::Vector3f
  getLeafSize() const;

  /** \brief Set the minimum number of points required for a voxel to be used.
   * \param[in] min_points_per_voxel the minimum number of points required for a voxel
   * to be used
   */
  void
  setMinimumPointsNumberPerVoxel(unsigned int min_points_per_voxel);

  /** \brief Return the minimum number of points required for a voxel to be used. */
  unsigned int
  getMinimumPointsNumberPerVoxel() const;

  /** \brief Provide the input cloud in GPU memory. Host output metadata is reset to
   * the default values of PointCloudHost.
   * \param[in] input the input cloud
   */
  void
  setInputCloud(const PointCloud& input);

  /** \brief Provide the input cloud in host memory. The cloud is copied to the GPU and
   * is used as it is for every consecutive filter call. Its header and sensor pose
   * are preserved in host output clouds.
   * \param[in] input the input cloud
   */
  void
  setInputCloud(const PointCloudHost& input);

  /** \brief Downsample the input cloud and store the result in GPU memory.
   * \param[out] output the downsampled cloud
   */
  void
  filter(PointCloud& output);

  /** \brief Downsample the input cloud and store the result in host memory.
   * \param[out] output the downsampled cloud
   */
  void
  filter(PointCloudHost& output);

private:
  Eigen::Vector3f leaf_size_;
  unsigned int min_points_per_voxel_;
  PointCloud input_;
  pcl::PCLHeader input_header_;
  Eigen::Vector4f input_sensor_origin_ = Eigen::Vector4f::Zero();
  Eigen::Quaternionf input_sensor_orientation_ = Eigen::Quaternionf::Identity();
  std::shared_ptr<device::VoxelGridImpl> impl_;
};
} // namespace gpu
} // namespace pcl
