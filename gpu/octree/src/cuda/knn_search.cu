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
#include "utils/bitonic_sort.hpp"
#include "octree_iterator.hpp"

namespace pcl { namespace device { namespace knn_search
{   
    typedef OctreeImpl::PointType PointType;
    
    struct Batch
    {           
        const PointType* queries;

        //int k == 1;

        const int *indices;

        // X1 X2 X3 X4 ..
        // Y1 Y2 Y3 Y4 ..
        // Z1 Z2 Z3 Z4 ..
        const float* points;
        int points_step; // elem step

        OctreeGlobalWithBox octree;

        int queries_num;                
        mutable int* output;                        
        mutable int* sizes;
    };

    struct KernelPolicy
    {
        enum 
        {
            CTA_SIZE = 512,
            WARP_SIZE = 32,
            WARPS_COUNT = CTA_SIZE/WARP_SIZE,                    
        };
    };

    struct Warp_knnSearch
    {   
    public:                        
        typedef OctreeIteratorDeviceNS OctreeIterator;

        const Batch& batch;

        int query_index;        
        float3 query;  
        
        float min_distance;
        int min_idx;

        OctreeIterator iterator;     

        __device__ __forceinline__ Warp_knnSearch(const Batch& batch_arg, int query_index_arg) 
            : batch(batch_arg), query_index(query_index_arg), min_distance(numeric_limits<float>::max()), min_idx(0), iterator(batch.octree) { }

        __device__ __forceinline__ void launch(bool active)
        {              
            if (active)
            {
                PointType q = batch.queries[query_index];
                query = make_float3(q.x, q.y, q.z);
            }
            else                
                query_index = -1;    

            while(__any(active))
            {                
                int leaf = -1;
           
                if (active)
                    leaf = examineNode(iterator);             

                processLeaf(leaf);      

                active = active && iterator.level >= 0;
            }
            if (query_index != -1)
            {
				batch.output[query_index] = batch.indices[min_idx];

                if (batch.sizes)
                    batch.sizes[query_index]  = 1;
            }
        }
    private:

        __device__ __forceinline__ int examineNode(OctreeIterator& iterator)
        {                        
            int node_idx = *iterator;
            int code = batch.octree.codes[node_idx];

            float3 node_minp = batch.octree.minp;
            float3 node_maxp = batch.octree.maxp;        
            calcBoundingBox(iterator.level, code, node_minp, node_maxp);

            //if true, take nothing, and go to next
            if (checkIfNodeOutsideSphere(node_minp, node_maxp, query, min_distance))
            {     
                ++iterator;
                return -1;                
            }                    

            //need to go to next level
            int node = batch.octree.nodes[node_idx];
            int children_mask = node & 0xFF;            
            bool isLeaf = children_mask == 0;            

            if (isLeaf)
            {
                ++iterator;
                return node_idx;
            }

            //goto next level
            int first = node >> 8;
            int len   = __popc(children_mask);
            iterator.gotoNextLevel(first, len);                    
            return -1;
        };

        __device__ __forceinline__ void processLeaf(int node_idx)
        {   
            int mask = __ballot(node_idx != -1);            

            unsigned int laneId = Warp::laneId();
            unsigned int warpId = Warp::id();

            __shared__ volatile int per_warp_buffer[KernelPolicy::WARPS_COUNT];

            while(mask)
            {
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

                //broadcast query_index
                if (active_lane == laneId)
                    *warp_buffer = query_index;
                float active_query_index = *warp_buffer;                            

                float dist;
                int offset = NearestWarpKernel<KernelPolicy::CTA_SIZE>(batch.points + beg, batch.points_step, end - beg, active_query, dist);
                
                if (active_lane == laneId)
                    if (min_distance > dist)
                    {
                       min_distance = dist;   
                       min_idx = beg + offset;
                    }                                                          
            }
        }

        template<int CTA_SIZE>
		__device__ __forceinline__ int NearestWarpKernel(const float* points, int points_step, int length, const float3& active_query, float& dist)
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

            dist = sqrt(dist2[tid - lane]);
			return index[tid - lane];
		}          
    };
    
    __global__ void KernelKNN(const Batch batch) 
    {           
        int query_index = blockIdx.x * blockDim.x + threadIdx.x;
                                
        bool active = query_index < batch.queries_num;

        if (__all(active == false)) 
            return;

        Warp_knnSearch search(batch, query_index);
        search.launch(active); 
    }

} } }


void pcl::device::OctreeImpl::nearestKSearchBatch(const Queries& queries, int /*k*/, NeighborIndices& results) const
{              
    typedef pcl::device::knn_search::Batch BatchType;

    BatchType batch;      
    batch.octree = octreeGlobal;
    batch.indices = indices;

    batch.queries_num = (int)queries.size();        
    batch.queries = queries;
    
    batch.output = results.data;
    batch.sizes  = results.sizes;
    
    batch.points = points_sorted;
    batch.points_step = points_sorted.step()/points_sorted.elem_size;

    cudaSafeCall( cudaFuncSetCacheConfig(pcl::device::knn_search::KernelKNN, cudaFuncCachePreferL1) );    

    int block = pcl::device::knn_search::KernelPolicy::CTA_SIZE;
    int grid = (batch.queries_num + block - 1) / block;        

    pcl::device::knn_search::KernelKNN<<<grid, block>>>(batch);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}

