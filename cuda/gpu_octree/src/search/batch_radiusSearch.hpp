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

#ifndef PCL_GPU_OCTREE_BATCH_SEARCH_GPU
#define PCL_GPU_OCTREE_BATCH_SEARCH_GPU

#include "octree_global.hpp"

#include "utils/laneid.hpp"
#include "utils/copygen.hpp"
#include "utils/boxutils.hpp"
#include "utils/devmem2d.hpp"
#include "utils/scan_block.hpp"

#include "search/octree_iterator.hpp"


namespace pcl
{
    namespace device
    {
        namespace batch_radius_search
        {   
            template<typename PointT, typename RadiusStrategy>
            struct Batch : public RadiusStrategy
            {   
                typedef PointT PointType;

                const int *indices;

                const float* points;
                int points_step; // elem step

                OctreeGlobalWithBox octree;

                const PointType* queries;

                int queries_num;
                int max_results;

                mutable int* output;
                mutable int* output_sizes;        
            };

            struct SharedRadius
            {
                float radius;
                __device__ __forceinline__ float getRadius(int /*index*/) const { return radius; }                

                __device__ __forceinline__ float bradcastRadius2(float* /*ptr*/, bool /*active*/, float& /*radius_reg*/) const
                {
                    return radius * radius;
                }
            };

            struct IndividualRadius
            {
                const float* radiuses;
                __device__ __forceinline__ float getRadius(int index) const { return radiuses[index]; }
                __device__ __forceinline__ float bradcastRadius2(float* ptr, bool active, float& radius_reg) const
                {
                    if (active)
                        *ptr = radius_reg * radius_reg;
                    return *ptr;
                }
            };

            struct KernelPolicy
            {
                enum 
                {
                    CTA_SIZE = 512,

                    WARP_SIZE = 32,
                    WARPS_COUNT = CTA_SIZE/WARP_SIZE,

                    MAX_LEVELS_PLUS_ROOT = 11,

                    CHECK_FLAG = 1 << 31
                };

                struct SmemStorage
                {         
                    //int paths[MAX_LEVELS_PLUS_ROOT][CTA_SIZE];
                    volatile int per_warp_buffer[WARPS_COUNT];
                    volatile int scan_buffer[CTA_SIZE];
                };
                
                /*template<bool OctreeWithParentIterator = OctreeGlobalWithBox::HasParent>
                struct SmemStorageT
                {            
                    volatile int per_warp_buffer[WARPS_COUNT];
                    volatile int scan_buffer[CTA_SIZE];
                };

                template<>
                struct SmemStorageT<false> : public SmemStorageT<true>
                {
                    int paths[MAX_LEVELS_PLUS_ROOT][CTA_SIZE];
                };

                typedef SmemStorageT<> SmemStorage;*/
            };

            __shared__ KernelPolicy::SmemStorage storage;
         

            template<typename BatchType>
            struct Warp_radiusSearch
            {   
            public:

                //typedef OctreeIteratorDevice<KernelPolicy::CTA_SIZE, KernelPolicy::MAX_LEVELS_PLUS_ROOT> OctreeIterator;
                typedef OctreeIteratorDeviceNS OctreeIterator;
                typedef typename BatchType::PointType PointType;
                
                const BatchType& batch;
                OctreeIterator iterator;        

                int found_count;
                int query_index;        
                float3 query;
                float radius;

                __device__ __forceinline__ Warp_radiusSearch(const BatchType& batch_arg, int query_index_arg) 
                    : batch(batch_arg), iterator(/**/batch.octree/*storage.paths/**/), found_count(0), query_index(query_index_arg){}

                __device__ __forceinline__ void launch(bool active)
                {                                 
                    if (active)
                    {
                        PointType q = batch.queries[query_index];
                        query = make_float3(q.x, q.y, q.z);
                        radius = batch.getRadius(query_index);
                    }
                    else                
                        query_index = -1;

                    while(__any(active))
                    {                
                        int leaf = -1;                

                        if (active)
                            leaf = examineNode(iterator);             

                        processLeaf(leaf);                           

                        active = active && iterator.level >= 0 && found_count < batch.max_results;
                    }            

                    if (query_index != -1)
                        batch.output_sizes[query_index] = found_count;
                }    

            private:

                __device__ __forceinline__ int examineNode(OctreeIterator& iterator)
                {                        
                    using namespace pcl::gpu;

                    int node_idx = *iterator;
                    int code = batch.octree.codes[node_idx];

                    float3 node_minp = batch.octree.minp;
                    float3 node_maxp = batch.octree.maxp;        
                    calcBoundingBox(iterator.level, code, node_minp, node_maxp);

                    //if true, take nothing, and go to next
                    if (checkIfNodeOutsideSphere(node_minp, node_maxp, query, radius))
                    {     
                        ++iterator;
                        return -1;                
                    }

                    if (checkIfNodeInsideSphere(node_minp, node_maxp, query, radius))
                    {   
                        ++iterator;       
                        return node_idx; //return node to copy
                    }                              

                    //need to go to next level
                    int node = batch.octree.nodes[node_idx];
                    int children_mask = node & 0xFF;            
                    bool isLeaf = children_mask == 0;            

                    if (isLeaf)
                    {
                        ++iterator;
                        return (node_idx | KernelPolicy::CHECK_FLAG); // return node to check                                                              
                    }

                    //goto next level
                    int first = node >> 8;
                    int len   = __popc(children_mask);
                    iterator.gotoNextLevel(first, len);                    
                    return -1;
                };

                __device__ __forceinline__ void processLeaf(int leaf)
                {   
                    int mask = __ballot(leaf != -1);            

                    while(mask)
                    {                
                        unsigned int laneId = LaneId();
                        unsigned int warpId = threadIdx.x/warpSize;            

                        int active_lane = __ffs(mask) - 1; //[0..31]
                        

                        mask &= ~(1 << active_lane);              

                        //broadcast active_found_count                                
                        if (active_lane == laneId)                
                            storage.per_warp_buffer[warpId] = found_count;                                            
                        int active_found_count = storage.per_warp_buffer[warpId];

                        int node_idx = leaf & ~KernelPolicy::CHECK_FLAG;

                        //broadcast beg
                        if (active_lane == laneId)
                            storage.per_warp_buffer[warpId] = batch.octree.begs[node_idx];                    
                        int beg = storage.per_warp_buffer[warpId];

                        //broadcast end
                        if (active_lane == laneId)
                            storage.per_warp_buffer[warpId] = batch.octree.ends[node_idx];
                        int end = storage.per_warp_buffer[warpId];

                        //broadcast active_query_index
                        if (active_lane == laneId)
                            storage.per_warp_buffer[warpId] = query_index;
                        int active_query_index = storage.per_warp_buffer[warpId];

                        int length = end - beg;

                        int *out = batch.output + active_query_index * batch.max_results + active_found_count;                    
                        int length_left = batch.max_results - active_found_count;

                        int test = __any(active_lane == laneId && (leaf & KernelPolicy::CHECK_FLAG));

                        if (test)
                        {                                        
                            float3 active_query;

                            //broadcast warp_query
                            if (active_lane == laneId)
                                storage.per_warp_buffer[warpId] = __float_as_int(query.x);
                            active_query.x = __int_as_float(storage.per_warp_buffer[warpId]);

                            if (active_lane == laneId)
                                storage.per_warp_buffer[warpId] = __float_as_int(query.y);
                            active_query.y = __int_as_float(storage.per_warp_buffer[warpId]);

                            if (active_lane == laneId)
                                storage.per_warp_buffer[warpId] = __float_as_int(query.z);
                            active_query.z = __int_as_float(storage.per_warp_buffer[warpId]);                            

                            float radius2 = batch.bradcastRadius2((float*)&storage.per_warp_buffer[warpId], (active_lane == laneId), radius);                            

                            length = TestWarpKernel(batch.indices + beg, batch.points + beg, batch.points_step, active_query, radius2, length, out, length_left);                    
                        }
                        else
                        {                            
                            length = min(length, length_left);
                            CopyWarpKernel(batch.indices + beg, out, length);                    
                        }

                        if (active_lane == laneId)
                            found_count += length;
                    }            
                }    

                __device__ __forceinline__ int TestWarpKernel(const int* indices, const float* points, int points_step, float3 active_query, float radius2, int length, int* out, int length_left)
                {                        
                    unsigned int idx = LaneId();
                    const int STRIDE = warpSize;            
                    int last_threadIdx = threadIdx.x - idx + 31;            

                    int total_new = 0;

                    for(;;)
                    {                
                        int take = 0;
                        
                        if (idx < length)
                        {                                                                                                            
                            float dx = points[idx                  ] - active_query.x;
                            float dy = points[idx + points_step    ] - active_query.y;
                            float dz = points[idx + points_step * 2] - active_query.z;

                            float d2 = dx * dx + dy * dy + dz * dz;

                            if (d2 < radius2)
                                take = 1;
                        }

                        storage.scan_buffer[threadIdx.x] = take;

                        int offset = scan_warp<exclusive>(storage.scan_buffer);

                        //ensure that we copy
                        bool out_of_bounds = (offset + total_new) >= length_left;                              

                        if (take && !out_of_bounds)
                            out[offset] = indices[idx];

                        int new_nodes = storage.scan_buffer[last_threadIdx];

                        idx += STRIDE;

                        total_new += new_nodes;
                        out += new_nodes;                

                        if (__all(idx >= length) || __any(out_of_bounds) || total_new == length_left)
                            break;
                    }
                    return min(total_new, length_left);
                }
            };
            
            template<typename BatchType>
            __global__ void KernelB(const BatchType batch) 
            {         
                int query_index = blockIdx.x * blockDim.x + threadIdx.x;

                bool active = query_index < batch.queries_num;

                if (__all(active == false)) 
                    return;

                Warp_radiusSearch<BatchType> search(batch, query_index);
                search.launch(active); 
            }
        }
    }
}

#endif /* PCL_GPU_OCTREE_BATCH_SEARCH_GPU */
