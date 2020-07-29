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

#include <limits>

#include "internal.hpp"
#include "pcl/gpu/utils/device/warp.hpp"

#include "utils/copygen.hpp"
#include "utils/boxutils.hpp"
#include "utils/scan_block.hpp"


namespace pcl { namespace device { namespace appnearest_search
{   
    using PointType = OctreeImpl::PointType;
	
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
			float3 minp = batch.octree.minp;
			float3 maxp = batch.octree.maxp;

			int node_idx = 0;
			int code = CalcMorton(minp, maxp)(query);
			int level = 0;

			bool centroid_traversal = false;
			int mask_pos;
			int x, y, z;

			for(;;)
			{
				int node = batch.octree.nodes[node_idx];
				int mask = node & 0xFF;

				float3 query_point;
				query_point.x = query.x;
				query_point.y = query.y;
				query_point.z = query.z;

				if(__popc(mask) == 0)  // leaf
				{
					//printf ("node x %d\n", node_idx);
					return node_idx;
				}

				if (!centroid_traversal)    // no empty voxel encountered yet, performing morton code based traversal
				{
					mask_pos = 1 << Morton::extractLevelCode(code, level);

					if ( (mask & mask_pos) == 0) // no child
					{

						//find current cell
						Morton::decomposeCode(code, x, y, z);

						x >>= (Morton::levels - level);
						y >>= (Morton::levels - level);
						z >>= (Morton::levels - level);

						centroid_traversal = true;  //switch to nearest-centroid based traversal
						mask_pos = nearestVoxelTraversal(query_point, level, mask, minp, maxp, x, y, z);
					}
				}

				else
					mask_pos = nearestVoxelTraversal(query_point, level, mask, minp, maxp, x, y, z);

				node_idx = (node >> 8) + __popc(mask & (mask_pos - 1));
				++level;
			}
		};

		__device__ int nearestVoxelTraversal(float3 query, int level, int mask, float3 minp, float3 maxp, int& x, int& y, int& z)
		{
			//identify closest voxel
			float closest_distance = std::numeric_limits<float>::max();
			int closest_index = 0, closest_x = 0, closest_y = 0, closest_z = 0;

			for (int i = 0; i < 8; ++i)
			{
				if ((mask & (1<<i)) == 0)   //no child
					continue;

				//calculate  x,y,z offset for voxel
				int x_cord = i & 1;
				int y_cord = (i>>1) & 1;
				int z_cord = (i>>2) & 1;

				int x_child, y_child, z_child;
				x_child = x*2 + x_cord;
				y_child = y*2 + y_cord;
				z_child = z*2 + z_cord;

				//find center of child cell
				float3 voxel_center;
				voxel_center.x = minp.x + (maxp.x - minp.x) * (2*x_child + 1) / (2 * 1<<(level + 1));
				voxel_center.y = minp.y + (maxp.y - minp.y) * (2*y_child + 1) / (2 * 1<<(level + 1));
				voxel_center.z = minp.z + (maxp.z - minp.z) * (2*z_child + 1) / (2 * 1<<(level + 1));

				//compute distance to centroid
				float dx = (voxel_center.x - query.x);
				float dy = (voxel_center.y - query.y);
				float dz = (voxel_center.z - query.z);
				float distance_to_query = dx * dx + dy * dy + dz * dz;

				//compare distance
				if (distance_to_query < closest_distance)
				{
					closest_distance = distance_to_query;
					closest_index = i;
					closest_x = x_child;
					closest_y = y_child;
					closest_z = z_child;
				}
			}

			x = closest_x;
			y = closest_y;
			z = closest_z;

			return  (1<<closest_index);
		}

		__device__ __forceinline__ void processNode(int node_idx)
		{   
            __shared__ volatile int  per_warp_buffer[KernelPolicy::WARPS_COUNT];

			int mask = __ballot_sync(0xFFFFFFFF, node_idx != -1);                        

			while(mask)
			{                
				unsigned int laneId = Warp::laneId();
				unsigned int warpId = threadIdx.x/warpSize;            

				int active_lane = __ffs(mask) - 1; //[0..31]                        
				mask &= ~(1 << active_lane);   

				volatile int* warp_buffer = &per_warp_buffer[warpId];

				//broadcast beg
				if (active_lane == laneId)
					*warp_buffer = batch.octree.begs[node_idx];                    
				int beg = *warp_buffer;

				//broadcast end
				if (active_lane == laneId)
					*warp_buffer = batch.octree.ends[node_idx];
				int end = *warp_buffer;

				float3 active_query;
				volatile float* warp_buffer_float = (float*)&per_warp_buffer[warpId];

				//broadcast warp_query
				if (active_lane == laneId)
					*warp_buffer_float = query.x;
				active_query.x = *warp_buffer_float;

				if (active_lane == laneId)
					*warp_buffer_float = query.y;
				active_query.y = *warp_buffer_float;

				if (active_lane == laneId)
					*warp_buffer_float = query.z;
				active_query.z = *warp_buffer_float;                            

				int offset = NearestWarpKernel<KernelPolicy::CTA_SIZE>(beg, batch.points_step, end - beg, active_query);                    

				if (active_lane == laneId)
					result_idx = beg + offset;
			}
		}

        template<int CTA_SIZE>
		__device__ __forceinline__ int NearestWarpKernel(int beg, int points_step, int length, const float3& active_query)
		{                        						
            __shared__ volatile float dist2[CTA_SIZE];
            __shared__ volatile int   index[CTA_SIZE];
			
            int tid = threadIdx.x;
			dist2[tid] = std::numeric_limits<float>::max();

			//serial step
            for (int idx = Warp::laneId(); idx < length; idx += Warp::STRIDE)
			{
				float dx = batch.points[beg + idx                  ] - active_query.x;
				float dy = batch.points[beg + idx + points_step    ] - active_query.y;
				float dz = batch.points[beg + idx + points_step * 2] - active_query.z;

				float d2 = dx * dx + dy * dy + dz * dz;

				if (dist2[tid] > d2)
				{
					dist2[tid] = d2;
					index[tid] = idx;                            
				}
			}
			//parallel step
			unsigned int lane = Warp::laneId();

			float mind2 = dist2[tid];

			if (lane < 16)
			{
				float next = dist2[tid + 16];
				if (mind2 > next) 
				{ 
					dist2[tid] = mind2 = next; 
					index[tid] = index[tid + 16]; 
				}                        
			}

			if (lane < 8)
			{
				float next = dist2[tid + 8];
				if (mind2 > next) 
				{ 
					dist2[tid] = mind2 = next; 
					index[tid] = index[tid + 8]; 
				}                        
			}

			if (lane < 4)
			{
				float next = dist2[tid + 4];
				if (mind2 > next) 
				{ 
					dist2[tid] = mind2 = next; 
					index[tid] = index[tid + 4]; 
				}                        
			}

			if (lane < 2)
			{
				float next = dist2[tid + 2];
				if (mind2 > next) 
				{ 
					dist2[tid] = mind2 = next; 
					index[tid] = index[tid + 2]; 
				}                        
			}

			if (lane < 1)
			{
				float next = dist2[tid + 1];
				if (mind2 > next) 
				{ 
					dist2[tid] = mind2 = next; 
					index[tid] = index[tid + 1]; 
				}                        
			}        

			return index[tid - lane];
		}
	};
	
	__global__ void KernelAN(const Batch batch) 
	{         
		int query_index = blockIdx.x * blockDim.x + threadIdx.x;

		bool active = query_index < batch.queries_num;

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