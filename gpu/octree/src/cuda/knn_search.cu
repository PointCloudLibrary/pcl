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
#include "utils/bitonic_sort.hpp"
#include "octree_iterator.hpp"
#include <tuple>

namespace pcl { namespace device { namespace knn_search
{   
    using PointType = OctreeImpl::PointType;
    
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
        mutable float* sqr_distances;
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
        using OctreeIterator = OctreeIteratorDeviceNS;

        const Batch& batch;

        int query_index;        
        float3 query;  
        
        float min_distance;
        float min_distance_squared;
        int min_idx;

        OctreeIterator iterator;     

        __device__ __forceinline__ Warp_knnSearch(const Batch& batch_arg, const int query_index_arg)
            : batch(batch_arg), query_index(query_index_arg), min_distance(std::numeric_limits<float>::max()), min_distance_squared(std::numeric_limits<float>::max()), min_idx(0), iterator(batch.octree) { }

        __device__ __forceinline__ void launch(bool active)
        {              
            if (active)
            {
                PointType q = batch.queries[query_index];
                query = make_float3(q.x, q.y, q.z);
            }
            else                
                query_index = -1;    

            while(__any_sync(0xFFFFFFFF, active))
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
                batch.sqr_distances[query_index] = min_distance_squared;

                if (batch.sizes)
                    batch.sizes[query_index]  = 1;
            }
        }
    private:

        __device__ __forceinline__ int examineNode(OctreeIterator& iterator)
        {                        
            const int node_idx = *iterator;
            const int code = batch.octree.codes[node_idx];

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
            const int node = batch.octree.nodes[node_idx];
            const int children_mask = node & 0xFF;            
            const bool isLeaf = children_mask == 0;            

            if (isLeaf)
            {
                ++iterator;
                return node_idx;
            }

            //goto next level
            const int first = node >> 8;
            const int len   = __popc(children_mask);
            iterator.gotoNextLevel(first, len);                    
            return -1;
        };

        __device__ __forceinline__ void processLeaf(const int node_idx)
        {   
            int mask = __ballot_sync(0xFFFFFFFF, node_idx != -1);            

            const unsigned int laneId = Warp::laneId();

            __shared__ volatile int per_warp_buffer[KernelPolicy::WARPS_COUNT];

            while(mask)
            {
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
                    if (min_distance_squared > nearestPoint.second)
                    {
                        min_distance_squared = nearestPoint.second;
                       min_idx = beg + nearestPoint.first;
                       min_distance = sqrt(nearestPoint.second);
                    }
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
          static_assert(KernelPolicy::WARP_SIZE <= 8*sizeof(unsigned int), "WARP_SIZE exceeds size of bit_offset.");

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
    
    __global__ void KernelKNN(const Batch batch) 
    {           
        const int query_index = blockIdx.x * blockDim.x + threadIdx.x;
                                
        const bool active = query_index < batch.queries_num;

        if (__all_sync(0xFFFFFFFF, active == false)) 
            return;

        Warp_knnSearch search(batch, query_index);
        search.launch(active); 
    }

} } }


void pcl::device::OctreeImpl::nearestKSearchBatch(const Queries& queries, int /*k*/, NeighborIndices& results, BatchResultSqrDists& sqr_distances) const
{              
    using BatchType = pcl::device::knn_search::Batch;

    BatchType batch;      
    batch.octree = octreeGlobal;
    batch.indices = indices;

    batch.queries_num = (int)queries.size();        
    batch.queries = queries;
    
    batch.output = results.data;
    batch.sizes  = results.sizes;
    batch.sqr_distances = sqr_distances;
    
    batch.points = points_sorted;
    batch.points_step = points_sorted.step()/points_sorted.elem_size;

    cudaSafeCall( cudaFuncSetCacheConfig(pcl::device::knn_search::KernelKNN, cudaFuncCachePreferL1) );    

    int block = pcl::device::knn_search::KernelPolicy::CTA_SIZE;
    int grid = (batch.queries_num + block - 1) / block;        

    pcl::device::knn_search::KernelKNN<<<grid, block>>>(batch);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}

