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
*  Original author: Duane Merrill
*  Integrated by 
*      Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
*/

#ifndef PCL_CUDA_UTILS_GLOBAL_BARRIER_HPP_
#define PCL_CUDA_UTILS_GLOBAL_BARRIER_HPP_


#include <modified_load.hpp>
#include "pcl/gpu/common/safe_call.hpp"

namespace pcl
{
    namespace device
    {
        namespace util 
        {
            template <typename T>
            __global__ void MemsetKernel(T *d_out, T value, int length)
            {
                const int STRIDE = gridDim.x * blockDim.x;

                for (int idx = (blockIdx.x * blockDim.x) + threadIdx.x; idx < length; idx += STRIDE) 
                {
                    d_out[idx] = value;
                }
            } 

            /**
            * Manages device storage needed for implementing a global software barrier between CTAs in a single grid
            */
            class GlobalBarrier
            {
            public:

                typedef unsigned int SyncFlag;

            protected :

                // Counters in global device memory
                SyncFlag* d_sync;

                //Simple wrapper for returning a CG-loaded SyncFlag at the specified pointer        
                __device__ __forceinline__ SyncFlag LoadCG(SyncFlag* d_ptr) const
                {
                    SyncFlag retval;
                    pcl::device::util::ld::ModifiedLoad<util::ld::cg>::Ld(retval, d_ptr);
                    return retval;
                }

            public:

                GlobalBarrier() : d_sync(0) {}


                //Synchronize
                __device__ __forceinline__ void Sync() const
                {
                    // Threadfence and syncthreads to make sure global writes are visible before
                    // thread-0 reports in with its sync counter
                    __threadfence();
                    __syncthreads();

                    if (blockIdx.x == 0) 
                    {

                        // Report in ourselves
                        if (threadIdx.x == 0) 
                            d_sync[blockIdx.x] = 1;

                        __syncthreads();

                        // Wait for everyone else to report in
                        for (int peer_block = threadIdx.x; peer_block < gridDim.x; peer_block += blockDim.x) 
                        {
                            while (LoadCG(d_sync + peer_block) == 0) 
                                __threadfence_block();
                        }

                        __syncthreads();

                        // Let everyone know it's safe to read their prefix sums
                        for (int peer_block = threadIdx.x; peer_block < gridDim.x; peer_block += blockDim.x) 
                        {
                            d_sync[peer_block] = 0;
                        }

                    } 
                    else 
                    {

                        if (threadIdx.x == 0) 
                        {
                            // Report in
                            d_sync[blockIdx.x] = 1;

                            // Wait for acknowledgement
                            while (LoadCG(d_sync + blockIdx.x) == 1) 
                                __threadfence_block();
                        }

                        __syncthreads();
                    }
                }
            };


            /**
            * Version of global barrier with storage lifetime management.
            *
            * We can use this in host enactors, and pass the base GlobalBarrier as parameters to kernels.
            */
            class GlobalBarrierLifetime : public GlobalBarrier
            {
            protected:

                // Number of bytes backed by d_sync
                size_t sync_bytes;

            public:

                GlobalBarrierLifetime() : GlobalBarrier(), sync_bytes(0) {}


                /**
                * Deallocates and resets the progress counters
                */
                void HostReset()
                {            
                    if (d_sync) 
                    {
                        cudaSafeCall( cudaFree(d_sync) );
                        d_sync = 0;
                    }
                    sync_bytes = 0;            
                }

                virtual ~GlobalBarrierLifetime()
                {
                    HostReset();
                }


                /**
                * Sets up the progress counters for the next kernel launch (lazily allocating and initializing them if necessary)
                */
                void Setup(int sweep_grid_size)
                {            
                    size_t new_sync_bytes = sweep_grid_size * sizeof(SyncFlag);

                    if (new_sync_bytes > sync_bytes) 
                    {                
                        HostReset();

                        sync_bytes = new_sync_bytes;
                        cudaSafeCall( cudaMalloc((void**) &d_sync, sync_bytes) );

                        // Initialize to zero
                        util::MemsetKernel<SyncFlag><<<(sweep_grid_size + 128 - 1) / 128, 128>>>(d_sync, 0, sweep_grid_size);
                        cudaSafeCall( cudaThreadSynchronize() );                    
                    }         
                }
            };
        } // namespace util

    } // namespace cuda

} // namespace pcl


#endif /*P CL_CUDA_UTILS_GLOBAL_BARRIER_HPP_ */