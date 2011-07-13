#pragma once

#include "octree_global.hpp"

#include "utils/LaneId.hpp"
#include "utils/copygen.hpp"
#include "utils/boxutils.hpp"

#include <cstdio>

namespace batch_radius_search
{
    template<class T>
    struct DevMem2D_
    {
        int cols;
        int rows;

        T* data;
        size_t step;

        DevMem2D_() : cols(0), rows(0), data(0), step(0) {}

        //DevMem2D_(const DeviceArray2D_<T>& device_array) : cols(0), rows(0), dat(0), step(0) {}


        __host__ __device__ __forceinline__ T* ptr(int y = 0) { return (T*)( (char*)data + y * step); }
        __host__ __device__ __forceinline__ const T* ptr(int y = 0) const { return (const T*)( (const char*)data + y * step); }
    };

  
    struct PackedSiblingInfo
    {        
        int data;

        __device__ __forceinline__ PackedSiblingInfo() {}
        __device__ __forceinline__ PackedSiblingInfo(int first, int len) : data( (first << 8) + len ) {}
        __device__ __forceinline__ int getNodeIndex() const { return data >> 8; }

        __device__ __forceinline__ bool gotoNextSibling()
        {
            data = data + (1 << 8) - 1;
            return (data & 0xFF) != 0;
        }
    };

    struct KernelPolicy
    {
        enum 
        {
            CTA_SIZE = 512,

            WARP_SIZE = 32,
            WAPRS_COUNT = CTA_SIZE/WARP_SIZE,

            MAX_LEVELS_PLUS_ROOT = 11,

            CHECK_FLAG = 1 << 31
        };

        struct SmemStorage
        {            
            PackedSiblingInfo path[MAX_LEVELS_PLUS_ROOT][CTA_SIZE];
            int per_warp_buf[WARP_SIZE];
            int scan_buffer[CTA_SIZE];
        };
    };
  
    struct BatchRadiusSearchShared
    {   
        typedef typename KernelPolicy::SmemStorage SmemStorage;

        const int *indices;
        const float3* points;
        OctreeGlobalWithBox octree;

        const float3* queries;
        float radius;

        DevMem2D_<int> output;
        int* output_sizes;
        
        __device__ __forceinline__ void operator() () const;
    };

    struct RadiusSearch
    {        
        typedef typename KernelPolicy::SmemStorage SmemStorage;

        struct TransversalHelper
        {                    
            int level;
            bool children_visited;
            SmemStorage& storage;

            __device__ __forceinline__ TransversalHelper(SmemStorage& storage_arg) : storage(storage_arg)
            {
                level = 0; // root level
                storage.path[level][threadIdx.x] = PackedSiblingInfo(0, 1);
                children_visited = false;
            }

            __device__ __forceinline__  int getNode() const 
            { 
                return storage.path[level][threadIdx.x].getNodeIndex(); }
            ;

            __device__ __forceinline__ void gotoNextLevel(int first, int len) 
            {   
                ++level;
                storage.path[level][threadIdx.x] = PackedSiblingInfo(first, len);            
                children_visited = false;            
            }

            __device__ __forceinline__ bool gotoSibling()
            {
                children_visited = false;
                return storage.path[level][threadIdx.x].gotoNextSibling();            
            }

            __device__ __forceinline__ void gotoParent()
            {
                --level;
                children_visited = true;
            }    
        };


                        
        SmemStorage& storage;
        const BatchRadiusSearchShared& batch;       
                
        int found_count;        
        int node4warp;        

        int query_idx;
        float3 query;
                
        __device__ __forceinline__ RadiusSearch(SmemStorage& storage_arg, const BatchRadiusSearchShared& batch_arg, int query_idx_arg) 
            : storage(storage_arg), batch(batch_arg), found_count(0), node4warp(-1), query_idx(query_idx_arg)
        {
            query = batch.queries[query_idx];
        }
        
        __device__ __forceinline__ int CheckWarpKernel(const int* indices, const float3* points, int* out, int length)
        {
            int STRIDE = warpSize;
            unsigned int laneId = LaneId();
            
            int total_new = 0;

            int lastWarpThread = (threadIdx.x/warpSize) * warpSize + 31;


            for (int idx = laneId; idx < length; idx += STRIDE) 
            {
                float3 pt = points[idx];

                float d2 = (pt.x - query.x) * (pt.x - query.x) + (pt.y - query.y) * (pt.y - query.y) + (pt.z - query.z) * (pt.z - query.z);

                int ok = (d2 < batch.radius * batch.radius) ? 1 : 0;
                storage.scan_buffer[threadIdx.x] = ok;
                
                int offset = scan_warp<ScanKind::exclusive>(storage.scan_buffer);
                total_new += storage.scan_buffer[lastWarpThread];
                
                if (ok)
                    out[offset] = indices[idx];
            }
            return total_new;
        }

        __device__ __forceinline__ int processNode4Warp()
        {            
            int mask = __ballot(node4warp != -1);
            unsigned int laneId = LaneId();
            unsigned int warpId = threadIdx.x/warpSize;

            while(mask)
            {
                int lane = __ffs(mask) - 1; //[0..31]
                mask &= ~(1 << lane);                
                                
                //broadcast offset
                if (lane == laneId)
                    storage.per_warp_buf[warpId] = batch.output.ptr(query_idx) + found_count - batch.output.ptr(0);                        
                int offset = storage.per_warp_buf[warpId];

                int beg, length;

                //cacl beg & length
                if (lane == laneId)
                {
                    int node_idx = node4warp & ~KernelPolicy::CHECK_FLAG;                    

                    beg = batch.octree.begs[node_idx];

                    int end = batch.octree.ends[node_idx];
                    length = min(found_count + end - beg, batch.output.cols) - found_count;
                }

                //broadcast beg
                if (lane == laneId)
                    storage.per_warp_buf[warpId] = beg;                    
                beg = storage.per_warp_buf[warpId];

                //broadcast length
                if (lane == laneId)
                    storage.per_warp_buf[warpId] = length;
                length = storage.per_warp_buf[warpId];

                int check = __any(lane == laneId && node4warp & KernelPolicy::CHECK_FLAG);

                if (check)
                {                                                         
                    int tota_new = CheckWarpKernel(batch.indices + beg, batch.points + beg, batch.output.data + offset, length);

                    if (lane == laneId)
                        found_count += tota_new;

                } /* just copy */
                else
                {                  
                    CopyWarpKernel(batch.indices + beg, batch.output.data + offset, length);

                    if (lane == laneId)
                        found_count += length;
                }                
            } 

            node4warp = -1; // for whole warp, after 
            return __all(batch.output.cols == found_count); // All are full. This means that current warp has finished its work
        }

        __device__ __forceinline__ int operator()()
        {
            TransversalHelper helper(storage);   
            node4warp = -1;            

            while(helper.level >= 0)
            //for(int i = 0; i < 9; ++i)
            {
                printf("Level = %d\n", helper.level);

                printf("Len = %d\n", helper.storage.path[helper.level][threadIdx.x].data & 0xFF);

                if (processNode4Warp())
                    return found_count;                

                //just returned from last child
                if (helper.children_visited)
                {
                    printf("children visited, goto to next\n");
                    if (!helper.gotoSibling())
                        helper.gotoParent();
                    
                    continue;
                }

                // neither chilren nor current node were visited, so test it
                int node_idx = helper.getNode();
                int code = batch.octree.codes[node_idx];

                float3 node_minp = batch.octree.minp, node_maxp = batch.octree.maxp;        
                calcBoundingBox(helper.level, code, node_minp, node_maxp);

                //if true, take nothing, and go to next
                if (checkIfNodeOutsideSphere(node_minp, node_maxp, query, batch.radius))
                {                
                    printf("outsize, left %d\n", helper.storage.path[helper.level][threadIdx.x].data & 0xFF);                    
                    if (!helper.gotoSibling())
                    {
                        printf("goto parent");
                        helper.gotoParent();
                    }
                    continue;
                }

                printf("+++after first\n");

                //if true, take all, and go to next
                if (checkIfNodeInsideSphere(node_minp, node_maxp, query, batch.radius))
                {            
                    node4warp = node_idx;
                    if (!helper.gotoSibling())
                        helper.gotoParent();

                    printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++all\n");
                    continue;
                }
                      
                printf("+++after second\n");

                /* TEST_CHILDREN */

                int node = batch.octree.nodes[node_idx];
                int children_mask = node & 0xFF;
                int first         = node >> 8;
                bool isLeaf = children_mask == 0;

                printf("children_mask = %d\n", children_mask);

                if (!isLeaf)
                {
                    printf("+++next level %d -> %d\n", helper.level, helper.level + 1);
                    printf("+++========================================================================================= \n");

                    helper.gotoNextLevel(first, __popc(children_mask));                    
                }
                else
                {
                    printf("+++check\n");
                    node4warp = node_idx | KernelPolicy::CHECK_FLAG;
                    if (!helper.gotoSibling())
                        helper.gotoParent();
                }
            }

            processNode4Warp();
            return found_count;
        }        
    };
   
    __device__ __forceinline__ void BatchRadiusSearchShared::operator() () const
    {        
        __shared__ SmemStorage storage;

        int total = output.rows;
        int query_idx = threadIdx.x + blockIdx.x * blockDim.x;
        
        if (query_idx < total)
        {
            RadiusSearch search(storage, *this, query_idx);

            int found_count = search();
            output_sizes[query_idx] = found_count;
        }
    };        

    __global__ void Kernel(BatchRadiusSearchShared batch) { batch(); }
}