/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#include "internal.hpp"
#include "pcl/gpu/utils/device/warp.hpp"
#include "utils/approx_nearest_utils.hpp"
#include "utils/boxutils.hpp"
#include "utils/copygen.hpp"
#include "utils/scan_block.hpp"
#include <assert.h>

#include <limits>
#include <tuple>

namespace pcl {
namespace device {
namespace appnearest_search {
using PointType = OctreeImpl::PointType;

struct Batch {
  const PointType* queries;

  const int* indices;
  const float* points;
  int points_step; // elem step

  OctreeGlobalWithBox octree;

  int queries_num;
  mutable int* output;
  mutable float* sqr_distance;
};

struct KernelPolicy {
  enum {
    CTA_SIZE = 512,

    LOG_WARP_SIZE = 5,
    WARP_SIZE = 1 << LOG_WARP_SIZE,
    WARPS_COUNT = CTA_SIZE / WARP_SIZE,
  };
};

struct Warp_appNearestSearch {
public:
  const Batch& batch;

  int query_index;
  float3 query;
  int result_idx;
  float sqr_dist;

  __device__ __forceinline__
  Warp_appNearestSearch(const Batch& batch_arg, int query_index_arg)
  : batch(batch_arg), query_index(query_index_arg)
  {}

  __device__ __forceinline__ void
  launch(bool active)
  {
    int node_idx = -1;
    if (active) {
      PointType q = batch.queries[query_index];
      query = make_float3(q.x, q.y, q.z);

      const float3& minp = batch.octree.minp;
      const float3& maxp = batch.octree.maxp;

      node_idx = pcl::device::findNode(minp, maxp, query, batch.octree.nodes);
    }

    processNode(node_idx);

    if (active)
    {
      batch.output[query_index] = batch.indices[result_idx];
      batch.sqr_distance[query_index] = sqr_dist;
    }
  }

private:
  __device__ __forceinline__ void
  processNode(const int node_idx)
  {
    __shared__ volatile int per_warp_buffer[KernelPolicy::WARPS_COUNT];

    int mask = __ballot_sync(0xFFFFFFFF, node_idx != -1);

    while (mask) {
      const unsigned int laneId = Warp::laneId();

      const int active_lane = __ffs(mask) - 1; //[0..31]
      mask &= ~(1 << active_lane);

      // broadcast beg and end
      int fbeg, fend;
      if (active_lane == laneId) {
        fbeg = batch.octree.begs[node_idx];
        fend = batch.octree.ends[node_idx];
      }
      const int beg = __shfl_sync(0xFFFFFFFF, fbeg, active_lane);
      const int end = __shfl_sync(0xFFFFFFFF, fend, active_lane);

      // broadcast warp_query
      const float3 active_query =
          make_float3(__shfl_sync(0xFFFFFFFF, query.x, active_lane),
                      __shfl_sync(0xFFFFFFFF, query.y, active_lane),
                      __shfl_sync(0xFFFFFFFF, query.z, active_lane));

      const auto nearestPoint = NearestWarpKernel<KernelPolicy::CTA_SIZE>(
          beg, batch.points_step, end - beg, active_query);

      if (active_lane == laneId)
      {
        result_idx = beg + nearestPoint.first;
        sqr_dist = nearestPoint.second;
      }
    }
  }

  template <int CTA_SIZE>
  __device__ std::pair<int, float>
  NearestWarpKernel(const int beg,
                    const int field_step,
                    const int length,
                    const float3& active_query)

  {
    int index = 0;
    float dist_squared = std::numeric_limits<float>::max();

    // serial step
    for (int idx = Warp::laneId(); idx < length; idx += Warp::STRIDE) {
      const float dx = batch.points[beg + idx] - active_query.x;
      const float dy = batch.points[beg + idx + field_step] - active_query.y;
      const float dz = batch.points[beg + idx + field_step * 2] - active_query.z;

      const float d2 = dx * dx + dy * dy + dz * dz;

      if (dist_squared > d2) {
        dist_squared = d2;
        index = idx;
      }
    }

    // find minimum distance among warp threads
    constexpr unsigned FULL_MASK = 0xFFFFFFFF;
    static_assert(KernelPolicy::WARP_SIZE <= 8 * sizeof(FULL_MASK),
                  "WARP_SIZE exceeds size of bit_offset.");

    for (unsigned int bit_offset = KernelPolicy::WARP_SIZE / 2; bit_offset > 0;
         bit_offset /= 2) {
      const float next = __shfl_down_sync(FULL_MASK, dist_squared, bit_offset);
      const int next_index = __shfl_down_sync(FULL_MASK, index, bit_offset);

      if (dist_squared > next) {
        dist_squared = next;
        index = next_index;
      }
    }

    // retrieve index and distance
    index = __shfl_sync(FULL_MASK, index, 0);
    dist_squared = __shfl_sync(FULL_MASK, dist_squared, 0);

    return {index, dist_squared};
  }
};

__global__ void
KernelAN(const Batch batch)
{
  const int query_index = blockIdx.x * blockDim.x + threadIdx.x;

  const bool active = query_index < batch.queries_num;

  if (__all_sync(0xFFFFFFFF, active == false))
    return;

  Warp_appNearestSearch search(batch, query_index);
  search.launch(active);
}

} // namespace appnearest_search
} // namespace device
} // namespace pcl

void
pcl::device::OctreeImpl::approxNearestSearch(const Queries& queries,
                                             NeighborIndices& results,
                                             BatchResultSqrDists& sqr_distance) const
{
  using BatchType = pcl::device::appnearest_search::Batch;

  BatchType batch;
  batch.indices = indices;
  batch.octree = octreeGlobal;

  batch.queries_num = (int)queries.size();
  batch.output = results.data;
  batch.sqr_distance = sqr_distance;

  batch.points = points_sorted;
  batch.points_step = (int)points_sorted.elem_step();
  batch.queries = queries;

  int block = pcl::device::appnearest_search::KernelPolicy::CTA_SIZE;
  int grid = (batch.queries_num + block - 1) / block;

  cudaSafeCall(cudaFuncSetCacheConfig(pcl::device::appnearest_search::KernelAN,
                                      cudaFuncCachePreferL1));

  pcl::device::appnearest_search::KernelAN<<<grid, block>>>(batch);
  cudaSafeCall(cudaGetLastError());
  cudaSafeCall(cudaDeviceSynchronize());
}