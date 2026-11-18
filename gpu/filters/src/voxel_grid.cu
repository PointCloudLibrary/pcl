/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026, Open Perception, Inc.
 *
 *  All rights reserved
 */

#include <pcl/gpu/utils/safe_call.hpp>

#include <thrust/device_ptr.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/discard_iterator.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>
#include <thrust/sort.h>

#include "internal.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace pcl::device {
namespace {
/** \brief Voxel index of points with non-finite coordinates. As the voxel indices of
 * all finite points fit into a signed 32 bit integer, this value is never used by a
 * regular voxel. */
constexpr unsigned int invalid_voxel_index = 0xFFFFFFFFu;

/** \brief Number of threads per block that is used for all kernels of this file. */
constexpr unsigned int block_size = 256;

/** \brief Number of threads of a warp (the centroid kernel assigns one warp to every
 * voxel). */
constexpr unsigned int warp_size = 32;

struct PointStats {
  float4 min_point;
  float4 max_point;
  unsigned int finite_points;
};

/** \brief Statistics of a cloud that contains only the given point. Non-finite points
 * do not contribute to the statistics. */
struct PointStatsFunctor {
  __host__ __device__ PointStats
  operator()(const float4& point) const
  {
    PointStats stats;
    if (isfinite(point.x) && isfinite(point.y) && isfinite(point.z)) {
      stats.min_point = point;
      stats.max_point = point;
      stats.finite_points = 1;
    }
    else {
      stats.min_point = make_float4(INFINITY, INFINITY, INFINITY, INFINITY);
      stats.max_point = make_float4(-INFINITY, -INFINITY, -INFINITY, -INFINITY);
      stats.finite_points = 0;
    }
    return stats;
  }
};

struct PointStatsOp {
  __host__ __device__ PointStats
  operator()(const PointStats& a, const PointStats& b) const
  {
    PointStats stats;
    stats.min_point = make_float4(fminf(a.min_point.x, b.min_point.x),
                                  fminf(a.min_point.y, b.min_point.y),
                                  fminf(a.min_point.z, b.min_point.z),
                                  fminf(a.min_point.w, b.min_point.w));
    stats.max_point = make_float4(fmaxf(a.max_point.x, b.max_point.x),
                                  fmaxf(a.max_point.y, b.max_point.y),
                                  fmaxf(a.max_point.z, b.max_point.z),
                                  fmaxf(a.max_point.w, b.max_point.w));
    stats.finite_points = a.finite_points + b.finite_points;
    return stats;
  }
};

/** \brief Computes the voxel index and the original index of every point. The voxel
 * index is the index of the voxel in the aligned bounding box of the input cloud,
 * computed row-major, exactly like in pcl::VoxelGrid. */
__global__ void
computeVoxelIndicesKernel(const float4* __restrict__ points,
                          const std::size_t point_count,
                          const float3 inverse_leaf_size,
                          const int3 min_box,
                          const int3 division,
                          unsigned int* __restrict__ voxel_indices,
                          unsigned int* __restrict__ point_indices)
{
  const std::size_t index =
      blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (index >= point_count)
    return;

  point_indices[index] = static_cast<unsigned int>(index);

  const float4 point = points[index];
  if (!isfinite(point.x) || !isfinite(point.y) || !isfinite(point.z)) {
    voxel_indices[index] = invalid_voxel_index;
    return;
  }

  const int ijk0 = static_cast<int>(floorf(point.x * inverse_leaf_size.x)) - min_box.x;
  const int ijk1 = static_cast<int>(floorf(point.y * inverse_leaf_size.y)) - min_box.y;
  const int ijk2 = static_cast<int>(floorf(point.z * inverse_leaf_size.z)) - min_box.z;

  voxel_indices[index] = static_cast<unsigned int>(ijk0 + ijk1 * division.x +
                                                   ijk2 * division.x * division.y);
}

/** \brief Marks the voxels that are part of the output cloud. */
__global__ void
markKeptVoxelsKernel(const unsigned int* __restrict__ run_counts,
                     const unsigned int run_count,
                     const unsigned int valid_run_count,
                     const unsigned int min_points_per_voxel,
                     unsigned int* __restrict__ run_flags)
{
  const unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= run_count)
    return;

  // The last run contains the non-finite points, if there are any
  const bool kept =
      index < valid_run_count && run_counts[index] >= min_points_per_voxel;
  run_flags[index] = kept ? 1u : 0u;
}

/** \brief Computes the centroid of every voxel. One warp computes the centroid of one
 * voxel, voxels that are not part of the output cloud are skipped. */
__global__ void
computeCentroidsKernel(const float4* __restrict__ points,
                       const unsigned int* __restrict__ point_indices,
                       const unsigned int* __restrict__ run_starts,
                       const unsigned int* __restrict__ run_counts,
                       const unsigned int* __restrict__ run_flags,
                       const unsigned int* __restrict__ run_positions,
                       const unsigned int run_count,
                       float4* __restrict__ centroids)
{
  const unsigned int run_index =
      blockIdx.x * (blockDim.x / warp_size) + threadIdx.x / warp_size;
  if (run_index >= run_count || run_flags[run_index] == 0)
    return;

  const unsigned int lane = threadIdx.x % warp_size;
  const unsigned int begin = run_starts[run_index];
  const unsigned int count = run_counts[run_index];

  float4 sum = make_float4(0.f, 0.f, 0.f, 0.f);
  for (unsigned int i = begin + lane; i < begin + count; i += warp_size) {
    const float4 point = points[point_indices[i]];
    sum.x += point.x;
    sum.y += point.y;
    sum.z += point.z;
    sum.w += point.w;
  }

#pragma unroll
  for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
    sum.x += __shfl_down_sync(0xFFFFFFFFu, sum.x, offset);
    sum.y += __shfl_down_sync(0xFFFFFFFFu, sum.y, offset);
    sum.z += __shfl_down_sync(0xFFFFFFFFu, sum.z, offset);
    sum.w += __shfl_down_sync(0xFFFFFFFFu, sum.w, offset);
  }

  if (lane == 0) {
    const float scale = 1.f / static_cast<float>(count);
    centroids[run_positions[run_index]] =
        make_float4(sum.x * scale, sum.y * scale, sum.z * scale, sum.w * scale);
  }
}
} // namespace

/** \brief Runs the filter pipeline and returns the downsampled cloud. */
DeviceArray<float4>
VoxelGridImpl::run(const DeviceArray<float4>& input,
                   const float3& leaf_size,
                   const unsigned int min_points_per_voxel,
                   bool& voxel_index_overflow)
{
  voxel_index_overflow = false;

  const std::size_t point_count = input.size();
  if (point_count == 0)
    return {};

  // Determine the bounding box and the number of finite points. Non-finite points are
  // ignored, just like in pcl::VoxelGrid (where this is done if the cloud is not
  // dense).
  PointStats init;
  init.min_point = make_float4(INFINITY, INFINITY, INFINITY, INFINITY);
  init.max_point = make_float4(-INFINITY, -INFINITY, -INFINITY, -INFINITY);
  init.finite_points = 0;

  const thrust::device_ptr<const float4> points_begin(input.ptr());
  const auto [min_point, max_point, finite_points] =
      thrust::transform_reduce(points_begin,
                               points_begin + point_count,
                               PointStatsFunctor(),
                               init,
                               PointStatsOp());

  if (finite_points == 0)
    return {};

  const float3 inverse_leaf_size =
      make_float3(1.f / leaf_size.x, 1.f / leaf_size.y, 1.f / leaf_size.z);

  // The grid is aligned to the bounding box of the input cloud
  const int3 min_box =
      make_int3(static_cast<int>(floorf(min_point.x * inverse_leaf_size.x)),
                static_cast<int>(floorf(min_point.y * inverse_leaf_size.y)),
                static_cast<int>(floorf(min_point.z * inverse_leaf_size.z)));
  const int3 max_box =
      make_int3(static_cast<int>(floorf(max_point.x * inverse_leaf_size.x)),
                static_cast<int>(floorf(max_point.y * inverse_leaf_size.y)),
                static_cast<int>(floorf(max_point.z * inverse_leaf_size.z)));
  const int3 division = make_int3(
      max_box.x - min_box.x + 1, max_box.y - min_box.y + 1, max_box.z - min_box.z + 1);

  // Check that the leaf size is not too small, given the size of the data
  const std::int64_t dx =
      static_cast<std::int64_t>((max_point.x - min_point.x) * inverse_leaf_size.x) + 1;
  const std::int64_t dy =
      static_cast<std::int64_t>((max_point.y - min_point.y) * inverse_leaf_size.y) + 1;
  const std::int64_t dz =
      static_cast<std::int64_t>((max_point.z - min_point.z) * inverse_leaf_size.z) + 1;
  if (dx * dy * dz >
      static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
    voxel_index_overflow = true;
    return {};
  }

  voxel_indices.create(point_count);
  point_indices.create(point_count);

  const auto index_grid_size =
      static_cast<unsigned int>((point_count + block_size - 1) / block_size);
  computeVoxelIndicesKernel<<<index_grid_size, block_size>>>(input.ptr(),
                                                             point_count,
                                                             inverse_leaf_size,
                                                             min_box,
                                                             division,
                                                             voxel_indices.ptr(),
                                                             point_indices.ptr());
  cudaSafeCall(cudaGetLastError());

  // Sort the points by their voxel index, so that all points of one voxel are adjacent
  const thrust::device_ptr<unsigned int> voxel_indices_begin(voxel_indices.ptr());
  const thrust::device_ptr<unsigned int> point_indices_begin(point_indices.ptr());
  thrust::stable_sort_by_key(
      voxel_indices_begin, voxel_indices_begin + point_count, point_indices_begin);

  run_counts.create(point_count);
  run_starts.create(point_count);
  run_flags.create(point_count);
  run_positions.create(point_count);

  const thrust::device_ptr<unsigned int> run_counts_begin(run_counts.ptr());
  const auto runs_end = thrust::reduce_by_key(voxel_indices_begin,
                                              voxel_indices_begin + point_count,
                                              thrust::make_constant_iterator(1u),
                                              thrust::make_discard_iterator(),
                                              run_counts_begin);
  const std::size_t run_count = runs_end.second - run_counts_begin;

  const thrust::device_ptr<unsigned int> run_starts_begin(run_starts.ptr());
  thrust::exclusive_scan(
      run_counts_begin, run_counts_begin + run_count, run_starts_begin);

  // The non-finite points have the largest voxel index, so they are collected in the
  // last voxel, which is never part of the output cloud
  const unsigned int valid_run_count =
      static_cast<unsigned int>(run_count) - (finite_points == point_count ? 0u : 1u);

  const thrust::device_ptr<unsigned int> run_flags_begin(run_flags.ptr());
  const auto run_grid_size =
      static_cast<unsigned int>((run_count + block_size - 1) / block_size);
  markKeptVoxelsKernel<<<run_grid_size, block_size>>>(
      run_counts_begin.get(),
      static_cast<unsigned int>(run_count),
      valid_run_count,
      min_points_per_voxel,
      run_flags_begin.get());
  cudaSafeCall(cudaGetLastError());

  const thrust::device_ptr<unsigned int> run_positions_begin(run_positions.ptr());
  thrust::exclusive_scan(
      run_flags_begin, run_flags_begin + run_count, run_positions_begin);
  const unsigned int output_count =
      thrust::reduce(run_flags_begin, run_flags_begin + run_count, 0u);

  DeviceArray<float4> result;
  result.create(output_count);
  if (output_count == 0)
    return result;

  const auto centroid_grid_size =
      static_cast<unsigned int>((run_count * warp_size + block_size - 1) / block_size);
  computeCentroidsKernel<<<centroid_grid_size, block_size>>>(
      input.ptr(),
      point_indices_begin.get(),
      run_starts_begin.get(),
      run_counts_begin.get(),
      run_flags_begin.get(),
      run_positions_begin.get(),
      static_cast<unsigned int>(run_count),
      result.ptr());
  cudaSafeCall(cudaGetLastError());

  return result;
}

void
VoxelGridImpl::applyFilter(const DeviceArray<float4>& input,
                          const float3& leaf_size,
                          const unsigned int min_points_per_voxel,
                          DeviceArray<float4>& output,
                          bool& voxel_index_overflow)
{
  // The result is computed before output is assigned, so that the filter can also be
  // applied in place (i.e. if output is the same array as input)
  output = run(input, leaf_size, min_points_per_voxel, voxel_index_overflow);
}
} // namespace pcl::device
