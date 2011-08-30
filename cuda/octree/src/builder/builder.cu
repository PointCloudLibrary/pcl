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

#include "pcl/gpu/utils/timers_cuda.hpp"

#include "internal.hpp"

#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>

#include "utils/morton.hpp"

#include "builder/cta_initial.hpp"
#include "builder/cta_syncsstep.hpp"

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


void pcl::gpu::OctreeImpl::build()
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
        //+1 tasksGlobal.line[0] * points_num * sizeof(int)
        //+1 tasksGlobal.line[1] * points_num * sizeof(int)

        //+3 points_sorted       * points_num * sizeof(float)
        //==
        // 12 rows   

        //left 
        //octreeGlobal.nodes_num * 1 * sizeof(int)  
        //tasksGlobal.count[0]   * 1 * sizeof(int)
        //tasksGlobal.count[1]   * 1 * sizeof(int)
        //==
        // 3 * sizeof(int) => +1 row

        //left
        //points_sorted    * points_num + sizeof(float3)


        const int transaction_size = 128 / sizeof(int);
        int cols = max<int>(points_num, transaction_size * 4);
        int rows = 12 + 1; // = 13
            
        storage.create(rows, cols);
        
        codes   = DeviceArray<int>(storage.ptr(0), points_num);
        indices = DeviceArray<int>(storage.ptr(1), points_num);
        
        octreeGlobal.nodes   = storage.ptr(2);
        octreeGlobal.codes   = storage.ptr(3);
        octreeGlobal.begs    = storage.ptr(4);
        octreeGlobal.ends    = storage.ptr(5);
        octreeGlobal.parent  = storage.ptr(6);

        octreeGlobal.nodes_num = storage.ptr(7);
        tasksGlobal.count[0]   = storage.ptr(7) + transaction_size;
        tasksGlobal.count[1]   = storage.ptr(7) + transaction_size * 2;

        tasksGlobal.line[0]  = storage.ptr(8);
        tasksGlobal.line[1]  = storage.ptr(9);

        points_sorted = DeviceArray2D<float>(3, points_num, storage.ptr(10), storage.step);
        tasksGlobal.active_selector = 0;        
    }

    {
        //ScopeTimer timer("reduce-morton-sort-permutations"); 
    	
        device_ptr<PointType> beg(points.ptr());
        device_ptr<PointType> end = beg + points.size();

        {
            PointType atmax, atmin;
            atmax.x = atmax.y = atmax.z = FLT_MAX;
            atmin.x = atmin.y = atmin.z = -FLT_MAX;

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
      
    {
        typedef initial::KernelInitialPolicyGlobOctree KernelPolicy;
        //printFuncAttrib(initial::Kernel<KernelPolicy>);        

        //ScopeTimer timer("KernelInitial"); 
        initial::Kernel<KernelPolicy><<<KernelPolicy::GRID_SIZE, max_threads_x>>>(tasksGlobal, octreeGlobal, codes.ptr(), points_num);
        cudaSafeCall( cudaGetLastError() );
        cudaSafeCall( cudaDeviceSynchronize() );
    }

    {
        typedef syncstep::KernelSyncPolicy KernelPolicy;
        //printFuncAttrib(syncstep::Kernel<KernelPolicy>);

        //ScopeTimer timer("Kernelsync"); 
        syncstep::Kernel<KernelPolicy><<<KernelPolicy::GRID_SIZE, max_threads_x>>>(tasksGlobal, octreeGlobal, codes);
        cudaSafeCall( cudaGetLastError() );
        cudaSafeCall( cudaDeviceSynchronize() );
    }    
}
