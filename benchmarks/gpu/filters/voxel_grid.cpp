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
#include <pcl/io/pcd_io.h>   // for PCDReader
#include <pcl/point_types.h> // for pcl::PointXYZ

#include <benchmark/benchmark.h>

#include <iostream>
#include <string>
#include <utility>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

//  run pcl::VoxelGrid, used as a baseline
static void
BM_VoxelGridCpu(benchmark::State& state, const std::string& file)
{
  auto cloud = pcl::make_shared<PointCloud>();
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(0.01f, 0.01f, 0.01f);
  filter.setInputCloud(cloud);

  PointCloud output;
  for (auto _ : state) {
    // This code gets timed
    filter.filter(output);
  }
}

// Runs pcl::gpu::VoxelGrid, both clouds stay in GPU memory.
static void
BM_VoxelGridGpu(benchmark::State& state, const std::string& file)
{
  auto cloud = pcl::make_shared<PointCloud>();
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::gpu::VoxelGrid::PointCloud device_input;
  device_input.upload(cloud->data(), cloud->size());

  pcl::gpu::VoxelGrid filter;
  filter.setLeafSize(0.01f, 0.01f, 0.01f);
  filter.setInputCloud(device_input);

  pcl::gpu::VoxelGrid::PointCloud device_output;
  filter.filter(device_output); // warm up

  for (auto _ : state) {
    // This code gets timed
    filter.filter(device_output);
  }
}

// Runs pcl::gpu::VoxelGrid, the result is copied back to host memory.
static void
BM_VoxelGridGpuWithDownload(benchmark::State& state, const std::string& file)
{
  auto cloud = pcl::make_shared<PointCloud>();
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::gpu::VoxelGrid filter;
  filter.setLeafSize(0.01f, 0.01f, 0.01f);
  filter.setInputCloud(*cloud);

  PointCloud output;
  for (auto _ : state) {
    // This code gets timed
    filter.filter(output);
  }
}

int
main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr
        << "No test files given. Please download `table_scene_mug_stereo_textured.pcd` "
           "and `milk_cartoon_all_small_clorox.pcd`, and pass their paths to the test."
        << std::endl;
    return (-1);
  }

  const bool has_gpu = pcl::gpu::getCudaEnabledDeviceCount() > 0;
  if (!has_gpu)
    std::cerr << "No CUDA device available, only running the CPU benchmarks."
              << std::endl;
  else {
    // Initialize the CUDA context and load the kernels, so that this does not happen
    // while the first GPU benchmark is measured
    PointCloud host_cloud(1, 1);
    pcl::gpu::VoxelGrid::PointCloud device_cloud;
    device_cloud.upload(host_cloud.data(), host_cloud.size());
    pcl::gpu::VoxelGrid warm_up;
    warm_up.setLeafSize(0.01f, 0.01f, 0.01f);
    warm_up.setInputCloud(device_cloud);
    pcl::gpu::VoxelGrid::PointCloud device_output;
    warm_up.filter(device_output);
  }

  const std::pair<std::string, std::string> clouds[] = {{"mug", argv[1]},
                                                        {"milk", argv[2]}};
  for (const auto& [name, file] : clouds) {
    benchmark::RegisterBenchmark(
        ("BM_VoxelGridCpu_" + name).c_str(), &BM_VoxelGridCpu, file)
        ->Unit(benchmark::kMillisecond);
    if (has_gpu) {
      benchmark::RegisterBenchmark(
          ("BM_VoxelGridGpu_" + name).c_str(), &BM_VoxelGridGpu, file)
          ->Unit(benchmark::kMillisecond);
      benchmark::RegisterBenchmark(("BM_VoxelGridGpuWithDownload_" + name).c_str(),
                                   &BM_VoxelGridGpuWithDownload,
                                   file)
          ->Unit(benchmark::kMillisecond);
    }
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
