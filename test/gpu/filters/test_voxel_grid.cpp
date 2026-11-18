/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026, Open Perception, Inc.
 *
 *  All rights reserved
 */

#include <pcl/filters/voxel_grid.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/filters/voxel_grid.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>

namespace {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/** \brief Creates a cloud with uniformly distributed random points. */
PointCloud::Ptr
createRandomCloud(std::size_t size, float range, unsigned int seed, bool dense = true)
{
  auto cloud = pcl::make_shared<PointCloud>();
  cloud->resize(size);
  cloud->width = static_cast<std::uint32_t>(size);
  cloud->height = 1;
  cloud->is_dense = dense;

  std::mt19937 random_engine(seed);
  std::uniform_real_distribution<float> uniform(-range, range);
  for (auto& point : cloud->points) {
    point.x = uniform(random_engine);
    point.y = uniform(random_engine);
    point.z = uniform(random_engine);
  }
  return cloud;
}

/** \brief Copies a device cloud to host memory. */
PointCloud
download(const pcl::gpu::VoxelGrid::PointCloud& device_cloud)
{
  PointCloud cloud;
  cloud.points.resize(device_cloud.size());
  if (!device_cloud.empty())
    device_cloud.download(cloud.points.data());
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

/** \brief Runs pcl::VoxelGrid on the given cloud. */
PointCloud
filterOnCpu(const PointCloud::Ptr& input,
            float leaf_size,
            unsigned int min_points_per_voxel = 1)
{
  PointCloud output;
  pcl::VoxelGrid<pcl::PointXYZ> cpu_filter;
  cpu_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  cpu_filter.setMinimumPointsNumberPerVoxel(min_points_per_voxel);
  cpu_filter.setInputCloud(input);
  cpu_filter.filter(output);
  return output;
}

/** \brief Checks that two clouds contain the same points. */
void
expectEqualClouds(const PointCloud& expected, const PointCloud& actual)
{
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(expected[i].x, actual[i].x, 1e-4f) << "point " << i;
    EXPECT_NEAR(expected[i].y, actual[i].y, 1e-4f) << "point " << i;
    EXPECT_NEAR(expected[i].z, actual[i].z, 1e-4f) << "point " << i;
  }
}

/** \brief Runs pcl::VoxelGrid and pcl::gpu::VoxelGrid on the same cloud and compares
 * the results. */
void
compareToCpu(const PointCloud::Ptr& input,
             float leaf_size,
             unsigned int min_points_per_voxel = 1)
{
  const PointCloud cpu_output = filterOnCpu(input, leaf_size, min_points_per_voxel);

  pcl::gpu::VoxelGrid gpu_filter;
  gpu_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  gpu_filter.setMinimumPointsNumberPerVoxel(min_points_per_voxel);
  gpu_filter.setInputCloud(*input);

  PointCloud gpu_output;
  gpu_filter.filter(gpu_output);

  ASSERT_EQ(gpu_output.width, gpu_output.size());
  ASSERT_EQ(gpu_output.height, 1u);
  EXPECT_TRUE(gpu_output.is_dense);
  expectEqualClouds(cpu_output, gpu_output);
}

} // namespace

TEST(PCL_GPU_VoxelGrid, MatchesCpuFilter)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  // More points than voxels, points and voxels of the same order of magnitude, and
  // fewer points than voxels
  compareToCpu(createRandomCloud(1000000, 1.f, 42), 0.05f);
  compareToCpu(createRandomCloud(10000, 10.f, 43), 0.5f);
  compareToCpu(createRandomCloud(1000, 1.f, 44), 1.5f);
}

TEST(PCL_GPU_VoxelGrid, MinimumPointsNumberPerVoxel)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  compareToCpu(createRandomCloud(50000, 1.f, 45), 0.05f, 5);
  compareToCpu(createRandomCloud(50000, 1.f, 46), 0.1f, 1000);
}

TEST(PCL_GPU_VoxelGrid, NonFinitePoints)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  auto cloud = createRandomCloud(10000, 1.f, 47, false);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  cloud->points.emplace_back(nan, 0.f, 0.f);
  cloud->points.emplace_back(0.f, nan, 0.f);
  cloud->points.emplace_back(0.f, 0.f, inf);
  cloud->points.emplace_back(inf, inf, -inf);
  cloud->width = static_cast<std::uint32_t>(cloud->size());

  compareToCpu(cloud, 0.1f);
}

TEST(PCL_GPU_VoxelGrid, DeviceToDevice)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  const auto input = createRandomCloud(50000, 1.f, 48);
  constexpr float leaf_size = 0.1f;

  pcl::gpu::VoxelGrid::PointCloud device_input;
  device_input.upload(input->data(), input->size());

  pcl::gpu::VoxelGrid gpu_filter;
  gpu_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  gpu_filter.setInputCloud(device_input);

  pcl::gpu::VoxelGrid::PointCloud device_output;
  gpu_filter.filter(device_output);

  expectEqualClouds(filterOnCpu(input, leaf_size), download(device_output));
}

TEST(PCL_GPU_VoxelGrid, FilterInPlace)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  const auto input = createRandomCloud(50000, 1.f, 50);
  constexpr float leaf_size = 0.1f;

  pcl::gpu::VoxelGrid::PointCloud device_cloud;
  device_cloud.upload(input->data(), input->size());

  pcl::gpu::VoxelGrid gpu_filter;
  gpu_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  gpu_filter.setInputCloud(device_cloud);

  gpu_filter.filter(device_cloud);

  expectEqualClouds(filterOnCpu(input, leaf_size), download(device_cloud));
}

TEST(PCL_GPU_VoxelGrid, EmptyCloud)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  pcl::gpu::VoxelGrid gpu_filter;
  gpu_filter.setLeafSize(0.1f, 0.1f, 0.1f);
  gpu_filter.setInputCloud(PointCloud());

  PointCloud output;
  gpu_filter.filter(output);
  EXPECT_TRUE(output.empty());
}

TEST(PCL_GPU_VoxelGrid, LeafSizeTooSmall)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  const auto input = createRandomCloud(1000, 50.f, 49);

  // PCL_WARN is printed, the input cloud is returned unchanged
  compareToCpu(input, 1e-4f);
}

TEST(PCL_GPU_VoxelGrid, HostInputDoesNotOverwriteDeviceCloud)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  const auto device_source = createRandomCloud(1000, 1.f, 51);
  const auto host_input = createRandomCloud(device_source->size(), 1.f, 52);
  pcl::gpu::VoxelGrid::PointCloud device_input;
  device_input.upload(device_source->data(), device_source->size());

  pcl::gpu::VoxelGrid filter;
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.setInputCloud(device_input);
  filter.setInputCloud(*host_input);

  expectEqualClouds(*device_source, download(device_input));
  PointCloud output;
  filter.filter(output);
  expectEqualClouds(filterOnCpu(host_input, 0.1f), output);
}

TEST(PCL_GPU_VoxelGrid, NonFinitePointsInOverflowFallback)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  PointCloud input;
  input.emplace_back(-50.f, -50.f, -50.f);
  input.emplace_back(50.f, 50.f, 50.f);
  input.emplace_back(std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f);
  input.emplace_back(0.f, std::numeric_limits<float>::infinity(), 0.f);
  input.is_dense = false;

  pcl::gpu::VoxelGrid filter;
  filter.setLeafSize(1e-4f, 1e-4f, 1e-4f);
  pcl::gpu::VoxelGrid::PointCloud device_input;
  device_input.upload(input.data(), input.size());

  PointCloud output;
  for (const bool use_host_input : {true, false}) {
    if (use_host_input)
      filter.setInputCloud(input);
    else
      filter.setInputCloud(device_input);
    output.is_dense = true;
    filter.filter(output);

    ASSERT_EQ(input.size(), output.size());
    EXPECT_FALSE(output.is_dense);
    EXPECT_FLOAT_EQ(output[0].x, input[0].x);
    EXPECT_FLOAT_EQ(output[1].x, input[1].x);
    EXPECT_TRUE(std::isnan(output[2].x));
    EXPECT_TRUE(std::isinf(output[3].y));
  }

  // Reusing the output for ordinary filtering must restore the dense flag.
  filter.setLeafSize(1.f, 1.f, 1.f);
  filter.filter(output);
  ASSERT_EQ(output.size(), 2u);
  EXPECT_TRUE(output.is_dense);
}

TEST(PCL_GPU_VoxelGrid, PreservesHostMetadata)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  auto input = createRandomCloud(1000, 50.f, 53);
  input->header.seq = 17;
  input->header.stamp = 123456789;
  input->header.frame_id = "sensor";
  input->sensor_origin_ = Eigen::Vector4f(1.f, 2.f, 3.f, 0.f);
  input->sensor_orientation_ =
      Eigen::Quaternionf(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitZ()));

  // Exercise ordinary filtering, overflow fallback, and an empty input.
  for (const float leaf_size : {0.5f, 1e-4f, 1.f}) {
    if (leaf_size == 1.f)
      input->clear();
    const auto expected_header = input->header;
    const auto expected_origin = input->sensor_origin_;
    const auto expected_orientation = input->sensor_orientation_;
    pcl::gpu::VoxelGrid filter;
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.setInputCloud(*input);

    // Metadata, like the uploaded points, is a snapshot of the input.
    input->header.stamp += 1;
    input->header.frame_id += "_next";
    input->sensor_origin_.x() += 1.f;
    input->sensor_orientation_ = Eigen::Quaternionf::Identity();

    PointCloud output;
    for (int repeat = 0; repeat < 2; ++repeat) {
      filter.filter(output);
      EXPECT_EQ(output.header.seq, expected_header.seq);
      EXPECT_EQ(output.header.stamp, expected_header.stamp);
      EXPECT_EQ(output.header.frame_id, expected_header.frame_id);
      EXPECT_TRUE(output.sensor_origin_.isApprox(expected_origin));
      EXPECT_TRUE(output.sensor_orientation_.isApprox(expected_orientation));
      output.header.frame_id = "stale";
      output.sensor_origin_.setZero();
      output.sensor_orientation_ = Eigen::Quaternionf::Identity();
    }
  }
}

TEST(PCL_GPU_VoxelGrid, DeviceInputResetsHostMetadata)
{
  if (pcl::gpu::getCudaEnabledDeviceCount() == 0)
    GTEST_SKIP() << "No CUDA device available";

  auto input = createRandomCloud(1000, 1.f, 54);
  input->header.seq = 23;
  input->header.stamp = 987654321;
  input->header.frame_id = "host_sensor";
  input->sensor_origin_ = Eigen::Vector4f(3.f, 2.f, 1.f, 0.f);
  input->sensor_orientation_ =
      Eigen::Quaternionf(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitX()));

  pcl::gpu::VoxelGrid filter;
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.setInputCloud(*input);
  PointCloud output;
  filter.filter(output);

  pcl::gpu::VoxelGrid::PointCloud device_input;
  device_input.upload(input->data(), input->size());
  filter.setInputCloud(device_input);
  filter.filter(output);

  const PointCloud defaults;
  EXPECT_EQ(output.header.seq, defaults.header.seq);
  EXPECT_EQ(output.header.stamp, defaults.header.stamp);
  EXPECT_EQ(output.header.frame_id, defaults.header.frame_id);
  EXPECT_TRUE(output.sensor_origin_.isApprox(defaults.sensor_origin_));
  EXPECT_TRUE(output.sensor_orientation_.isApprox(defaults.sensor_orientation_));
  expectEqualClouds(filterOnCpu(input, 0.1f), output);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
