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
#include "device.hpp"
#include "pcl/gpu/utils/device/funcattrib.hpp"
#include "pcl/gpu/utils/device/block.hpp"
#include "pcl/gpu/utils/device/warp.hpp"

namespace pcl
{
    namespace device
    {        
        __device__ int global_count = 0;
        __shared__ int shared_count;

        const static int MAX_SHARED_COUNT = 256*3;
        __shared__ float storage[3][MAX_SHARED_COUNT];
#if 0
        struct FullScan26
        {
            enum 
            {
                CTA_SIZE_X = 32,
                CTA_SIZE_Y = 8
            };
            PtrStep<volume_elem_type> volume;
            float3 cell_size;

            __device__ __forceinline__ float fetch(int x, int y, int z, int& weight) const
            {
                float tsdf;
                unpack_tsdf(volume.ptr(VOLUME_Y * z + y)[x], tsdf, weight);
                return tsdf;                
            }

            __device__ __forceinline__ void operator()()const
            {
                int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
                int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

                if (x >= VOLUME_X || y >= VOLUME_Y)
                    return;

                if (threadIdx.x == 0 && threadIdx.y == 0)
                    shared_count = 0;

                float3 V;
                V.x = (x + 0.5f) * cell_size.x;
                V.y = (y + 0.5f) * cell_size.y;

                for(int z = 0; z < VOLUME_Z - 1; ++z)
                {
                    int W;
                    float F = fetch(x, y, z, W);

                    if (W == 0 || F == 1.f)
                        continue;
                    
                    V.z = (z + 0.5f) * cell_size.z;

                    //front 3x3
                    int dz = 1;                
                    for(int dy = -1; dy < 2; ++dy)
                    {
                        if (y + dy >= VOLUME_X || y + dy < 0)
                            continue;

                        for(int dx = -1; dx < 2; ++dx)
                        {
                            if (x + dx >= VOLUME_X || x + dx < 0)
                                continue;

                            int Wn;
                            float Fn = fetch(x+dx, y+dy, z+dz, Wn);
                                               
                            if (Wn == 0 || Fn == 1.f)
                                continue;
                    
                            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                            {
                                float3 Vn = V;
                                Vn.x += dx * cell_size.x;
                                Vn.y += dy * cell_size.y;
                                Vn.z += dz * cell_size.z;
                                   
                                float d_inv = 1.f/(fabs(F) + fabs(Fn));
                                float3 p = (V * fabs(Fn) + Vn * fabs(F)) * d_inv;

                                if(!store(p))
                                    return;
                            }
                        }
                    }

                    //middle 3x1 + 1
                    dz = 0;
                    for(int dy = 0; dy < 2; ++dy)
                    {
                        if (y + dy >= VOLUME_Y)
                            continue;

                        for(int dx = -1; dx < dy * 2; ++dx)
                        {
                            if (x + dx >= VOLUME_X || x + dx < 0)
                                continue;
                            
                            int Wn;
                            float Fn = fetch(x+dx, y+dy, z+dz, Wn);
                                               
                            if (Wn == 0 || Fn == 1.f)
                                continue;
                    
                            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                            {
                                float3 Vn = V;
                                Vn.x += dx * cell_size.x;
                                Vn.y += dy * cell_size.y;
                                Vn.z += dz * cell_size.z;
                                   
                                float d_inv = 1.f/(fabs(F) + fabs(Fn));
                                float3 p = (V * fabs(Fn) + Vn * fabs(F)) * d_inv;

                                if(!store(p))
                                    return;
                            }                 
                        }
                    }

                } /* for(int z = 0; z < VOLUME_Z - 1; ++z) */
            } /* operator() */

            __device__ __forceinline__ bool store(const float3& p) const
            {                
                //__ffs __ballot(1);

                return true;
            }
        };

#endif     
        struct FullScan6
        {
            enum 
            {
                CTA_SIZE_X = 32,
                CTA_SIZE_Y = 8
            };
            PtrStep<volume_elem_type> volume;
            float3 cell_size;

            __device__ __forceinline__ float fetch(int x, int y, int z, int& weight) const
            {
                float tsdf;
                unpack_tsdf(volume.ptr(VOLUME_Y * z + y)[x], tsdf, weight);
                return tsdf;                
            }

            __device__ __forceinline__ void operator()()const
            {
                int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
                int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

                if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0)
                    shared_count = 0;

                if (__all(x >= VOLUME_X) || __all(y >= VOLUME_Y))
                    return;
                
                float3 V;
                V.x = (x + 0.5f) * cell_size.x;
                V.y = (y + 0.5f) * cell_size.y;
                                
                for(int z = 0; z < VOLUME_Z - 1; ++z)
                {
                    float3 points[3];
                    int local_count = 0;

                    if (x < VOLUME_X && y < VOLUME_Y)
                    {
                        int W;
                        float F = fetch(x, y, z, W);

                        if (W == 0 || F == 1.f)
                            continue;
                        
                        V.z = (z + 0.5f) * cell_size.z;
                        
                        //process dx
                        if (x + 1 < VOLUME_X)
                        {
                            int Wn;
                            float Fn = fetch(x+1, y, z, Wn);

                            if (Wn == 0 || Fn == 1.f)
                                continue;

                            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                            {
                                float3 p;
                                p.y = V.y;
                                p.z = V.z;
                                                                           
                                float Vnx = V.x + cell_size.x;

                                float d_inv = 1.f/(fabs(F) + fabs(Fn));
                                p.x = (V.x * fabs(Fn) + Vnx * fabs(F)) * d_inv;

                                points[local_count++] = p;                                
                            }
                        }

                        //process dy
                        if (y + 1 < VOLUME_Y)
                        {
                            int Wn;
                            float Fn = fetch(x, y+1, z, Wn);

                            if (Wn == 0 || Fn == 1.f)
                                continue;

                            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                            {
                                float3 p;
                                p.x = V.x;
                                p.z = V.z;
                                                                           
                                float Vny = V.y + cell_size.y;

                                float d_inv = 1.f/(fabs(F) + fabs(Fn));
                                p.y = (V.x * fabs(Fn) + Vny * fabs(F)) * d_inv;

                                points[local_count++] = p;
                            }
                        }

                        //process dz
                        //if (z + 1 < VOLUME_Z) // guaranteed by loop
                        {
                            int Wn;
                            float Fn = fetch(x, y, z+1, Wn);

                            if (Wn == 0 || Fn == 1.f)
                                continue;

                            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                            {
                                float3 p;
                                p.x = V.x;
                                p.y = V.y;
                                               
                                float Vnz = V.z + cell_size.z;

                                float d_inv = 1.f/(fabs(F) + fabs(Fn));
                                p.z = (V.x * fabs(Fn) + Vnz * fabs(F)) * d_inv;

                                points[local_count++] = p;
                            }
                        }
                    }

                    int total_warp = __popc(__ballot(local_count > 0)) + __popc(__ballot(local_count > 1)) + __popc(__ballot(local_count > 2));                    

                    //unsigned int value = atomicInc(&count, gridDim.x);

                    __syncthreads();

                    flush();

                } /* for(int z = 0; z < VOLUME_Z - 1; ++z) */
            } /* operator() */

            __device__ __forceinline__ bool flush() const
            {      
                if (threadIdx.x == 0 && threadIdx.y == 0)
                {
                    //offs = atimicAdd();
                }

                //Block::copy(

                //__ffs __ballot(1);
                return true;
            }
        };

        //__global__ void extractKernel(const FullScan26 fs) { fs(); }
    }
}


size_t pcl::device::extractCloud(const PtrStep<volume_elem_type>& volume, const float3& volume_size, PtrSz<PointType> output, bool connected26)
{
 //   FullScan26 fs;
 //   fs.volume = volume;
 //   fs.cell_size.x = volume_size.x / VOLUME_X;
 //   fs.cell_size.y = volume_size.y / VOLUME_Y;
 //   fs.cell_size.z = volume_size.z / VOLUME_Z;

 //   dim3 block(FullScan26::CTA_SIZE_X, FullScan26::CTA_SIZE_Y);
 //   dim3 grid(divUp(VOLUME_X, block.x), divUp(VOLUME_Y, block.y));

 //   //cudaFuncSetCacheConfig(extractKernel, cudaFuncCachePreferL1);
 //   //printFuncAttrib(extractKernel);

 //   extractKernel<<<grid, block>>>(fs);
	//cudaSafeCall( cudaGetLastError() );    	  
	//cudaSafeCall(cudaDeviceSynchronize());

    return 0;
}




