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
#include "pcl/gpu/utils/safe_call.hpp"
#include "pcl/gpu/utils/device/warp.hpp"

#include <thrust/scan.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/unique.h>

namespace pcl
{
    namespace device
    {
        __device__ int total_after_repack;

        struct IndsRepack
        {
            enum 
            { 
                CTA_SIZE = 256,
                WARPS = CTA_SIZE/Warp::WARP_SIZE
            };

            const int *offsets;    
            PtrStep<int> nindices;
            const int *sizes;
            int work_size;

            mutable int *output;

            __device__ void operator()() const
            {
                int idx = WARPS * blockIdx.x + Warp::id();

                if (idx >= work_size)
                    return;

                int size = sizes[idx];
                const int *ninds_beg = nindices.ptr(idx);        
                const int *ninds_end = ninds_beg + size;
                const int before = offsets[idx];

                Warp::copy(ninds_beg, ninds_end, output + before);

                if (idx == work_size - 1 && Warp::laneId() == 0)
                    total_after_repack = before + size;
            }
        };

        __global__ void IndsRepackKernel(const IndsRepack irpk) { irpk(); } 

        __global__ void createLookupKernel(const int* inds, int total, int* output)
        {
            int idx = threadIdx.x + blockIdx.x * blockDim.x;

            if (idx < total)
                output[inds[idx]] = idx;
        }
    }

}


int pcl::device::computeUniqueIndices(std::size_t surface_size, const NeighborIndices& neighbours, DeviceArray<int>& unique_indices, DeviceArray<int>& lookup)
{
    unique_indices.create(neighbours.data.size());
    lookup.create(surface_size);

    thrust::device_vector<int> scan(neighbours.sizes.size());
    thrust::device_ptr<int> beg((int*)neighbours.sizes.ptr());
    thrust::device_ptr<int> end = beg + neighbours.sizes.size();
    thrust::exclusive_scan(beg, end, scan.begin());  

    IndsRepack irpk;
    irpk.offsets = raw_pointer_cast(&scan[0]);
    irpk.sizes = neighbours.sizes;
    irpk.nindices = neighbours;    
    irpk.output = unique_indices;
    irpk.work_size = (int)neighbours.sizes.size();

    int block = IndsRepack::CTA_SIZE;
    int grid = divUp((int)neighbours.sizes.size(), IndsRepack::WARPS);

    IndsRepackKernel<<<grid, block>>>(irpk);
    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());

    int total;
    cudaSafeCall( cudaMemcpyFromSymbol(&total, total_after_repack, sizeof(total)) );
    cudaSafeCall(cudaDeviceSynchronize());

    thrust::device_ptr<int> begu(unique_indices.ptr());
    thrust::device_ptr<int> endu = begu + total;    

    thrust::sort(begu, endu);
    total = (int)(thrust::unique(begu, endu) - begu);   

    thrust::device_ptr<int> begl(lookup.ptr());
    thrust::device_ptr<int> endl = begl + lookup.size();
    thrust::fill(begl, endl, 0);
    
    createLookupKernel<<<divUp((int)unique_indices.size(), 256), 256>>>(unique_indices, total, lookup);
    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());

    return total;
}