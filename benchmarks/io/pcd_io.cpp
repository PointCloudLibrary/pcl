/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/io/pcd_io.h>   // for PCDWriter
#include <pcl/point_types.h> // for pcl::PointXYZ, ...

#include <benchmark/benchmark.h>

#include <cstdint> // for std::uint32_t
#include <cstdio>  // for std::remove
#include <numeric> // for std::iota
#include <string>

template <typename PointT>
static void
BM_WriteBinary(benchmark::State& state, const std::string& file)
{
  // Perform setup here
  const pcl::PointCloud<PointT> cloud(static_cast<std::uint32_t>(state.range(0)), 1);
  pcl::PCDWriter writer;
  for (auto _ : state) {
    // This code gets timed
    writer.writeBinary(file, cloud);
  }
  std::remove(file.c_str());
}

template <typename PointT>
static void
BM_WriteBinaryIndices(benchmark::State& state, const std::string& file)
{
  // Perform setup here
  const pcl::PointCloud<PointT> cloud(static_cast<std::uint32_t>(state.range(0)), 1);
  pcl::Indices indices(cloud.size());
  std::iota(indices.begin(), indices.end(), 0);
  pcl::PCDWriter writer;
  for (auto _ : state) {
    // This code gets timed
    writer.writeBinary(file, cloud, indices);
  }
  std::remove(file.c_str());
}

template <typename PointT>
static void
registerBenchmarks(const std::string& point_type, const std::string& file)
{
  benchmark::RegisterBenchmark(
      ("BM_WriteBinary<" + point_type + ">").c_str(), &BM_WriteBinary<PointT>, file)
      ->Arg(100000)
      ->Arg(1000000)
      ->Unit(benchmark::kMillisecond);
  benchmark::RegisterBenchmark(("BM_WriteBinaryIndices<" + point_type + ">").c_str(),
                               &BM_WriteBinaryIndices<PointT>,
                               file)
      ->Arg(100000)
      ->Arg(1000000)
      ->Unit(benchmark::kMillisecond);
}

int
main(int argc, char** argv)
{
  benchmark::Initialize(&argc, argv);

  // The benchmarks write to this file. Pass a path on a RAM disk (for example in
  // /dev/shm) to measure the time spent in PCL and not in the storage device.
  const std::string file = (argc > 1) ? argv[1] : "benchmark_pcd_io.pcd";

  registerBenchmarks<pcl::PointXYZ>("PointXYZ", file);
  registerBenchmarks<pcl::PointXYZI>("PointXYZI", file);
  registerBenchmarks<pcl::PointNormal>("PointNormal", file);
  registerBenchmarks<pcl::PointXYZRGBNormal>("PointXYZRGBNormal", file);

  benchmark::RunSpecifiedBenchmarks();
}
