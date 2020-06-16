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

#include <cfloat>
#include "internal.hpp"

#include "pcl/gpu/utils/timers_cuda.hpp"
#include "pcl/gpu/utils/device/funcattrib.hpp"
#include "pcl/gpu/utils/device/algorithm.hpp"
#include "pcl/gpu/utils/device/static_check.hpp"
#include "utils/scan_block.hpp"
#include "utils/morton.hpp"

#include <thrust/device_ptr.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/device_ptr.h>

using namespace pcl::gpu;
using namespace thrust;

namespace pcl 
{
    namespace device
    {
        template<typename PointType>
        struct SelectMinPoint
        {
	        __host__ __device__ __forceinline__ PointType operator()(const PointType& e1, const PointType& e2) const
	        {
                PointType result;
                result.x = fmin(e1.x, e2.x);
                result.y = fmin(e1.y, e2.y);
                result.z = fmin(e1.z, e2.z);
                return result;		        
	        }	
        };

        template<typename PointType>
        struct SelectMaxPoint
        {
	        __host__ __device__ __forceinline__ PointType operator()(const PointType& e1, const PointType& e2) const 
	        {		
                PointType result;
                result.x = fmax(e1.x, e2.x);
                result.y = fmax(e1.y, e2.y);
                result.z = fmax(e1.z, e2.z);
                return result;		    
	        }	
        };


        template<typename PointType>
        struct PointType_to_tuple
        {
            __device__ __forceinline__ thrust::tuple<float, float, float> operator()(const PointType& arg) const
            {
                thrust::tuple<float, float, float> res;
                res.get<0>() = arg.x;
                res.get<1>() = arg.y;
                res.get<2>() = arg.z;
                return res;
            }
        };
    }
}
namespace pcl
{
    namespace device
    {
        const static int max_points_per_leaf = 96;

        enum 
        { 
            GRID_SIZE = 1,
            CTA_SIZE = 1024-32,
            STRIDE = CTA_SIZE,

            LEVEL_BITS_NUM = 3,
            ARITY = 1 << LEVEL_BITS_NUM
        };

        __shared__ int nodes_num;
        __shared__ int tasks_beg;
        __shared__ int tasks_end;
        __shared__ int total_new;
        __shared__ volatile int offsets[CTA_SIZE];                

        struct SingleStepBuild
        {
            const int* codes;
            int points_number;
            mutable OctreeGlobal octree;

            static __device__ __forceinline__ int divUp(int total, int grain) { return (total + grain - 1) / grain; };

            __device__ __forceinline__ int FindCells(int task, int level, int cell_begs[], char cell_code[]) const
            {               
                int cell_count = 0;

                int beg = octree.begs[task];
                int end = octree.ends[task];                

                if (end - beg < max_points_per_leaf)
                {                        
                    //cell_count == 0;
                }
                else
                {
                    int cur_code = Morton::extractLevelCode(codes[beg], level);

                    cell_begs[cell_count] = beg;
                    cell_code[cell_count] = cur_code;     
                    ++cell_count;                        

                    int last_code = Morton::extractLevelCode(codes[end - 1], level);
                    if (last_code == cur_code)
                    {
                        cell_begs[cell_count] = end;                                         
                    }
                    else
                    {
                        for(;;)
                        {
                            int search_code = cur_code + 1;
                            if (search_code == 8)
                            {
                                cell_begs[cell_count] = end;
                                break;
                            }

                            int morton_code = Morton::shiftLevelCode(search_code, level);
                            int pos = lower_bound(codes + beg, codes + end, morton_code, CompareByLevelCode(level)) - codes; 

                            if (pos == end)
                            {
                                cell_begs[cell_count] = end;
                                break;
                            }
                            cur_code = Morton::extractLevelCode(codes[pos], level);

                            cell_begs[cell_count] = pos;
                            cell_code[cell_count] = cur_code;
                            ++cell_count;
                            beg = pos;
                        }        
                    }
                }
                return cell_count;
            }


            __device__  __forceinline__ void operator()() const
            {             
                //32 is a performance penalty step for search
                Static<(max_points_per_leaf % 32) == 0>::check();                 

                if (threadIdx.x == 0)
                {
                    //init root
                    octree.codes[0] = 0;
                    octree.nodes[0] = 0;
                    octree. begs[0] = 0;
                    octree. ends[0] = points_number;
                    octree.parent[0] = -1;

                    //init shared                    
                    nodes_num = 1;
                    tasks_beg = 0;
                    tasks_end = 1;
                    total_new = 0;
                }

                int level = 0;

                int  cell_begs[ARITY + 1];
                char cell_code[ARITY];

                __syncthreads();

                while (tasks_beg < tasks_end && level < Morton::levels)
                {                  
                    int task_count = tasks_end - tasks_beg;                    
                    int iters = divUp(task_count, CTA_SIZE);

                    int task = tasks_beg + threadIdx.x;

                    //__syncthreads(); // extra??

                    for(int it = 0; it < iters; ++it, task += STRIDE)
                    {   
                        int cell_count = (task < tasks_end) ? FindCells(task, level, cell_begs, cell_code) : 0;
                        
                        offsets[threadIdx.x] = cell_count;
                        __syncthreads();

                        scan_block<pcl::device::exclusive>(offsets);
                        
                        //__syncthreads();  //because sync is inside the scan above

                        if (task < tasks_end)
                        {
                            if (cell_count > 0)
                            {
                                int parent_code_shifted = octree.codes[task] << LEVEL_BITS_NUM;
                                int offset = nodes_num + offsets[threadIdx.x];

                                int mask = 0;
                                for(int i = 0; i < cell_count; ++i)
                                {
                                    octree.begs [offset + i] = cell_begs[i];
                                    octree.ends [offset + i] = cell_begs[i + 1];
                                    octree.codes[offset + i] = parent_code_shifted + cell_code[i];
                                    octree.nodes[offset + i] = 0;
                                    octree.parent[offset + i] = task;
                                    mask |= (1 << cell_code[i]);
                                }
                                octree.nodes[task] = (offset << 8) + mask;
                            }
                            else
                                octree.nodes[task] = 0;
                        }

                        __syncthreads();
                        if (threadIdx.x == CTA_SIZE - 1)
                        {                            
                            total_new += cell_count + offsets[threadIdx.x];
                            nodes_num += cell_count + offsets[threadIdx.x];
                        }    
                        __syncthreads(); 

                    } /* for(int it = 0; it < iters; ++it, task += STRIDE) */

                    //__syncthreads(); //extra ??

                    if (threadIdx.x == CTA_SIZE - 1)
                    {                       
                        tasks_beg  = tasks_end;
                        tasks_end += total_new;        
                        total_new = 0;
                    }
                    ++level;
                    __syncthreads();                    
                }

                if (threadIdx.x == CTA_SIZE - 1)
                    *octree.nodes_num = nodes_num;
            }
        };

        __global__ void __launch_bounds__(CTA_SIZE) singleStepKernel(const SingleStepBuild ssb) { ssb(); }
    }
}

void pcl::device::OctreeImpl::build()
{       
    using namespace pcl::device;
    host_octree.downloaded = false;

    int points_num = (int)points.size();

    //allocatations
    {
        //ScopeTimer timer("new_allocs"); 
        //+1 codes               * points_num * sizeof(int)
        //+1 indices             * points_num * sizeof(int)
        //+1 octreeGlobal.nodes  * points_num * sizeof(int)

        //+1 octreeGlobal.codes  * points_num * sizeof(int)
        //+1 octreeGlobal.begs   * points_num * sizeof(int)
        //+1 octreeGlobal.ends   * points_num * sizeof(int)

        //+1 octreeGlobal.parent * points_num * sizeof(int)

        //+3 points_sorted       * points_num * sizeof(float)
        //==
        // 10 rows   

        //left 
        //octreeGlobal.nodes_num * 1 * sizeof(int)          
        //==
        // 3 * sizeof(int) => +1 row        

        const int transaction_size = 128 / sizeof(int);
        int cols = max<int>(points_num, transaction_size * 4);
        int rows = 10 + 1; // = 13
            
        storage.create(rows, cols);
        
        codes   = DeviceArray<int>(storage.ptr(0), points_num);
        indices = DeviceArray<int>(storage.ptr(1), points_num);
        
        octreeGlobal.nodes   = storage.ptr(2);
        octreeGlobal.codes   = storage.ptr(3);
        octreeGlobal.begs    = storage.ptr(4);
        octreeGlobal.ends    = storage.ptr(5);
        octreeGlobal.parent  = storage.ptr(6);

        octreeGlobal.nodes_num = storage.ptr(7);

        points_sorted = DeviceArray2D<float>(3, points_num, storage.ptr(8), storage.step());
    }
    
    {
        //ScopeTimer timer("reduce-morton-sort-permutations"); 
    	
        device_ptr<PointType> beg(points.ptr());
        device_ptr<PointType> end = beg + points.size();

        {
            PointType atmax, atmin;
            atmax.x = atmax.y = atmax.z = FLT_MAX;
            atmin.x = atmin.y = atmin.z = -FLT_MAX;
            atmax.w = atmin.w = 0;

            //ScopeTimer timer("reduce"); 
            PointType minp = thrust::reduce(beg, end, atmax, SelectMinPoint<PointType>());
            PointType maxp = thrust::reduce(beg, end, atmin, SelectMaxPoint<PointType>());

            octreeGlobal.minp = make_float3(minp.x, minp.y, minp.z);
            octreeGlobal.maxp = make_float3(maxp.x, maxp.y, maxp.z);
        }
    		
        device_ptr<int> codes_beg(codes.ptr());
        device_ptr<int> codes_end = codes_beg + codes.size();
        {
            //ScopeTimer timer("morton"); 
	        thrust::transform(beg, end, codes_beg, CalcMorton(octreeGlobal.minp, octreeGlobal.maxp));
        }

        device_ptr<int> indices_beg(indices.ptr());
        device_ptr<int> indices_end = indices_beg + indices.size();
        {
            //ScopeTimer timer("sort"); 
            thrust::sequence(indices_beg, indices_end);
            thrust::sort_by_key(codes_beg, codes_end, indices_beg );		
        }
        {
            ////ScopeTimer timer("perm"); 
            //thrust::copy(make_permutation_iterator(beg, indices_beg),
            //                  make_permutation_iterator(end, indices_end), device_ptr<float3>(points_sorted.ptr()));    

             
        }

        {
            device_ptr<float> xs(points_sorted.ptr(0));
            device_ptr<float> ys(points_sorted.ptr(1));
            device_ptr<float> zs(points_sorted.ptr(2));
            //ScopeTimer timer("perm2"); 
            thrust::transform(make_permutation_iterator(beg, indices_beg),
                              make_permutation_iterator(end, indices_end), 
                              make_zip_iterator(make_tuple(xs, ys, zs)), PointType_to_tuple<PointType>());

        
        }
    }
    
    SingleStepBuild ssb;
    ssb.octree = octreeGlobal;
    ssb.codes  = codes;
    ssb.points_number = (int)codes.size();
    //printFuncAttrib(singleStepKernel);

    cudaSafeCall( cudaFuncSetCacheConfig(singleStepKernel, cudaFuncCachePreferL1) );

    singleStepKernel<<<GRID_SIZE, CTA_SIZE>>>(ssb);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}
