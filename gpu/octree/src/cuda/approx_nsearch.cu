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

#include <assert.h>
#include <limits>
#include <tuple>

#include "internal.hpp"
#include "pcl/gpu/utils/device/warp.hpp"

#include "utils/copygen.hpp"
#include "utils/boxutils.hpp"
#include "utils/scan_block.hpp"


namespace pcl { namespace device { namespace appnearest_search
{
    using PointType = OctreeImpl::PointType;

	__host__ __device__  std::pair<uint3, std::uint8_t> nearestVoxel(const float3 query, const unsigned& level, const std::uint8_t& mask, const float3& minp, const float3& maxp, const uint3& index)
	{
		assert(mask != 0);
		//identify closest voxel
		float closest_distance = std::numeric_limits<float>::max();
		unsigned closest_index = 0;
		uint3 closest = make_uint3(0,0,0);

		for (unsigned i = 0; i < 8; ++i)
		{
			if ((mask & (1<<i)) == 0)   //no child
				continue;

			const uint3 child = make_uint3(
				(index.x << 1) + (i & 1),
				(index.y << 1) + ((i>>1) & 1),
				(index.z << 1) + ((i>>2) & 1));

			//find center of child cell
			const unsigned voxel_width = 1 << (level + 2);
			const float3 voxel_center = make_float3(
				minp.x + (maxp.x - minp.x) * (2*child.x + 1) / voxel_width,
				minp.y + (maxp.y - minp.y) * (2*child.y + 1) / voxel_width,
				minp.z + (maxp.z - minp.z) * (2*child.z + 1) / voxel_width);

			//compute distance to centroid
			const float3 dist = make_float3(voxel_center.x - query.x, voxel_center.y - query.y, voxel_center.z - query.z);

			const float distance_to_query = dist.x * dist.x + dist.y * dist.y + dist.z * dist.z;

			//compare distance
			if (distance_to_query < closest_distance)
			{
				closest_distance = distance_to_query;
				closest_index = i;
				closest = child;
			}
		}

		return std::pair<uint3, std::uint8_t>(closest, 1<<closest_index);
	}

	struct Batch
	{   
		const PointType* queries;

		const int *indices;
		const float* points;
		int points_step; // elem step

		OctreeGlobalWithBox octree;
	    
		int queries_num;                
		mutable int* output;                
	};

	struct KernelPolicy
	{
		enum 
		{
			CTA_SIZE = 512,

			LOG_WARP_SIZE = 5,
			WARP_SIZE = 1 << LOG_WARP_SIZE,
			WARPS_COUNT = CTA_SIZE/WARP_SIZE,                    
		};	
	};

	struct Warp_appNearestSearch
	{   
	public:                		
		const Batch& batch;

		int query_index;        
		float3 query;  
		int result_idx;

		__device__ __forceinline__ Warp_appNearestSearch(const Batch& batch_arg, int query_index_arg) 
			: batch(batch_arg), query_index(query_index_arg){}

		__device__ __forceinline__ void launch(bool active)
		{              
			int node_idx = -1;
			if (active)
			{
				PointType q = batch.queries[query_index];
				query = make_float3(q.x, q.y, q.z);                        

				node_idx = findNode();
			}           

			processNode(node_idx);                    

			if (active)
				batch.output[query_index] = batch.indices[result_idx];
		}    

	private:

		__device__ __forceinline__ int findNode()
		{
			const float3& minp = batch.octree.minp;
			const float3& maxp = batch.octree.maxp;

			size_t node_idx = 0;
			const auto code = CalcMorton(minp, maxp)(query);
			unsigned level = 0;

			bool voxel_traversal = false;
			uint3 index = Morton::decomposeCode(code);
			std::uint8_t mask_pos;

			while(true)
			{
				const auto node = batch.octree.nodes[node_idx];
				const std::uint8_t mask = node & 0xFF;

				if(!mask)  // leaf
					return node_idx;

					if (voxel_traversal)    // empty voxel already encountered, performing nearest-centroid based traversal
					{
						const auto nearest_voxel = nearestVoxel(query, level, mask, minp, maxp, index);
						index = nearest_voxel.first;
						mask_pos = nearest_voxel.second;
					}
					else
					{
						mask_pos = 1 << Morton::extractLevelCode(code, level);

						if (!(mask & mask_pos)) // child doesn't exist
						{
							const auto remaining_depth = Morton::levels - level;
							index.x >>= remaining_depth;
							index.y >>= remaining_depth;
							index.z >>= remaining_depth;

							voxel_traversal = true;
							const auto nearest_voxel = nearestVoxel(query, level, mask, minp, maxp, index);
							index = nearest_voxel.first;
							mask_pos = nearest_voxel.second;
						}
					}

				node_idx = (node >> 8) + __popc(mask & (mask_pos - 1));
				++level;
			}
		};


		__device__ __forceinline__ void processNode(const int node_idx)
		{
			int mask = __ballot_sync(0xFFFFFFFF, node_idx != -1);

			while(mask)
			{
				const unsigned int laneId = Warp::laneId();

				const int active_lane = __ffs(mask) - 1; //[0..31]
				mask &= ~(1 << active_lane);

                //broadcast beg and end
                int fbeg, fend;
                if (active_lane == laneId)
                {
                    fbeg = batch.octree.begs[node_idx];
                    fend = batch.octree.ends[node_idx];
                }
                const int beg = __shfl_sync(0xFFFFFFFF, fbeg, active_lane);
                const int end = __shfl_sync(0xFFFFFFFF, fend, active_lane);

                //broadcast warp_query
                const float3 active_query = make_float3(
                    __shfl_sync(0xFFFFFFFF, query.x, active_lane),
                    __shfl_sync(0xFFFFFFFF, query.y, active_lane),
                    __shfl_sync(0xFFFFFFFF, query.z, active_lane)
                );

                const auto nearestPoint = NearestWarpKernel<KernelPolicy::CTA_SIZE>(beg, batch.points_step, end - beg, active_query);

				if (active_lane == laneId)
					result_idx = beg + nearestPoint.first;
			}
		}

        template <int CTA_SIZE>
        __device__  std::pair<int, float>
        NearestWarpKernel(const int beg,
                          const int field_step,
                          const int length,
                          const float3& active_query)

        {
          int index = 0;
          float dist2 = std::numeric_limits<float>::max();

          // serial step
          for (int idx = Warp::laneId(); idx < length; idx += Warp::STRIDE) {
            const float dx = batch.points[beg + idx] - active_query.x;
            const float dy = batch.points[beg + idx + field_step] - active_query.y;
            const float dz = batch.points[beg + idx + field_step * 2] - active_query.z;

            const float d2 = dx * dx + dy * dy + dz * dz;

            if (dist2 > d2) {
              dist2 = d2;
              index = idx;
            }
          }

          // find minimum distance among warp threads
          constexpr unsigned FULL_MASK = 0xFFFFFFFF;
          static_assert(KernelPolicy::WARP_SIZE <= 8*sizeof(unsigned int));

          for (unsigned int bit_offset = KernelPolicy::WARP_SIZE / 2; bit_offset > 0;
               bit_offset /= 2) {
            const float next = __shfl_down_sync(FULL_MASK, dist2, bit_offset);
            const int next_index = __shfl_down_sync(FULL_MASK, index, bit_offset);

            if (dist2 > next) {
              dist2 = next;
              index = next_index;
            }
          }

          // retrieve index and distance
          index = __shfl_sync(FULL_MASK, index, 0);
          const float dist = sqrt(__shfl_sync(FULL_MASK, dist2, 0));

          return std::make_pair(index, dist);
        }
	};
	
	__global__ void KernelAN(const Batch batch) 
	{         
		const int query_index = blockIdx.x * blockDim.x + threadIdx.x;

		const bool active = query_index < batch.queries_num;

		if (__all_sync(0xFFFFFFFF, active == false)) 
			return;

		Warp_appNearestSearch search(batch, query_index);
		search.launch(active); 
	}

} } }


void pcl::device::OctreeImpl::approxNearestSearch(const Queries& queries, NeighborIndices& results) const
{
    using BatchType = pcl::device::appnearest_search::Batch;

    BatchType batch;
    batch.indices = indices;
    batch.octree = octreeGlobal;

    batch.queries_num = (int)queries.size();        
    batch.output = results.data;     

    batch.points = points_sorted;
    batch.points_step = (int)points_sorted.elem_step();
    batch.queries = queries;

    int block = pcl::device::appnearest_search::KernelPolicy::CTA_SIZE;
    int grid = (batch.queries_num + block - 1) / block;    

    cudaSafeCall( cudaFuncSetCacheConfig(pcl::device::appnearest_search::KernelAN, cudaFuncCachePreferL1) );

    pcl::device::appnearest_search::KernelAN<<<grid, block>>>(batch);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}