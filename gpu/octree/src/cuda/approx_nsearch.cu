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
#include "pcl/gpu/utils/device/limits.hpp"
#include "pcl/gpu/utils/device/warp.hpp"

#include "utils/copygen.hpp"
#include "utils/boxutils.hpp"
#include "utils/scan_block.hpp"


namespace pcl { namespace device { namespace appnearest_search
{   
    typedef OctreeImpl::PointType PointType;
	
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

			if(query.x < minp.x || query.y < minp.y ||  query.z < minp.z)
				return 0;

			if(query.x > maxp.x || query.y > maxp.y ||  query.z > maxp.z)
				return 0;

			int node_idx = 0;
			int code = CalcMorton(minp, maxp)(query);
			int level = 0;

			for(;;)
			{
				int mask_pos = 1 << Morton::extractLevelCode(code, level);

				int node = batch.octree.nodes[node_idx];
				int mask = node & 0xFF;

				if(__popc(mask) == 0)  // leaf
					return node_idx; 

				if ( (mask & mask_pos) == 0) // no child
					return node_idx;                         

				node_idx = (node >> 8) + __popc(mask & (mask_pos - 1));
				++level;
			}
		};

		__device__ __forceinline__ void processNode(int node_idx)
		{   
            __shared__ volatile int  per_warp_buffer[KernelPolicy::WARPS_COUNT];

			int mask = __ballot(node_idx != -1);                        

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

				int offset = NearestWarpKernel<KernelPolicy::CTA_SIZE>(batch.points + beg, batch.points_step, end - beg, active_query);                    

				if (active_lane == laneId)
					result_idx = beg + offset;
			}
		}

        template<int CTA_SIZE>
		__device__ __forceinline__ int NearestWarpKernel(const float* points, int points_step, int length, const float3& active_query)
		{                        						
            __shared__ volatile float dist2[CTA_SIZE];
            __shared__ volatile int   index[CTA_SIZE];
			
            int tid = threadIdx.x;
			dist2[tid] = pcl::device::numeric_limits<float>::max();

			//serial step
            for (int idx = Warp::laneId(); idx < length; idx += Warp::STRIDE)
			{
				float dx = points[idx                  ] - active_query.x;
				float dy = points[idx + points_step    ] - active_query.y;
				float dz = points[idx + points_step * 2] - active_query.z;

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

		if (__all(active == false)) 
			return;

		Warp_appNearestSearch search(batch, query_index);
		search.launch(active); 
	}

} } }


void pcl::device::OctreeImpl::approxNearestSearch(const Queries& queries, NeighborIndices& results) const
{
    typedef pcl::device::appnearest_search::Batch BatchType;

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