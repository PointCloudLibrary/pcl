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
#include "pcl/gpu/utils/device/block.hpp"
#include "utils/vector_operations.hpp"
#include "pcl/gpu/utils/device/funcattrib.hpp"
#include "pcl/gpu/features/device/pair_features.hpp"

using namespace pcl::gpu;

namespace pcl
{
    namespace device
    {    
        template<bool pack_rgb>
        struct Repack
        {   
            enum 
            {
                CTA_SIZE = 256,
                WARPS = CTA_SIZE/Warp::WARP_SIZE
            };

            const PointType* cloud;
            const NormalType* normals;
            int work_size;

            PtrStep<int> gindices;            
            const int* sizes;         

            mutable PtrStep<float> output;
            int max_elems;

            __device__ void operator()() const
            {                
                int idx = WARPS * blockIdx.x + Warp::id();

                if (idx >= work_size)
                    return;

                const int *nbeg = gindices.ptr(idx);
                int size = sizes[idx];
                int idx_shift = max_elems * idx;

                for(int i = Warp::laneId(); i < size; i += Warp::STRIDE)
                {
                    int cloud_index = nbeg[i];
                    
                    float3 p;
                    
                    if (pack_rgb)
                    {
                        int color;
                        p = fetchXYZRGB(cloud, cloud_index, color);
                        output.ptr(6)[i + idx_shift] = __int_as_float(color);
                    }
                    else
                        p = fetch(cloud, cloud_index);

                    output.ptr(0)[i + idx_shift] = p.x;
                    output.ptr(1)[i + idx_shift] = p.y;
                    output.ptr(2)[i + idx_shift] = p.z;
                    
                    
                    float3 n = fetch(normals, cloud_index);

                    output.ptr(3)[i + idx_shift] = n.x;
                    output.ptr(4)[i + idx_shift] = n.y;
                    output.ptr(5)[i + idx_shift] = n.z;
                }
            }

            template<class It> 
            __device__ __forceinline__ float3 fetch(It ptr, int index) const
            {
                //return tr(ptr[index]);
                return *(float3*)&ptr[index];
            }

            __forceinline__ __device__ float3 fetchXYZRGB(const PointXYZRGB* data, int index, int& color) const
            {
                float4 xyzrgb = data[index];
                color = __float_as_int(xyzrgb.w);
                return make_float3(xyzrgb.x, xyzrgb.y, xyzrgb.z);
            }
        };

        template<bool enable_rgb>
        struct Pfh125
        {
            enum 
            {
                CTA_SIZE = 256,

                NR_SPLIT = 5,
                NR_SPLIT_2 = NR_SPLIT * NR_SPLIT,
                NR_SPLIT_3 = NR_SPLIT_2 * NR_SPLIT,

                FSize = NR_SPLIT * NR_SPLIT * NR_SPLIT * (enable_rgb ? 2 : 1)
            };

            std::size_t work_size;
            const int* sizes;

            PtrStep<float> rpk;
            int max_elems;
                       
            mutable PtrStep<float> output;

            __device__ __forceinline__ void operator()() const
            {                                
                int idx = blockIdx.x;

                if (idx >= work_size)
                    return;                

                int size = sizes[idx];                
                int size2 = size * size;
                int idx_shift = max_elems * idx;

                float hist_incr = 100.f / (size2 - 1);
                
                __shared__ float pfh_histogram[FSize];
                Block::fill(pfh_histogram, pfh_histogram + FSize, 0.f);
                __syncthreads();

                // Iterate over all the points in the neighborhood
                int i = threadIdx.y * blockDim.x + threadIdx.x;
                int stride = Block::stride();

                for( ; i < size2; i += stride )
                {
                    int i_idx = i / size + idx_shift;
                    int j_idx = i % size + idx_shift;

                    if (i_idx != j_idx)
                    {
                        float3 pi, ni, pj, nj;
                        pi.x = rpk.ptr(0)[i_idx];
                        pj.x = rpk.ptr(0)[j_idx];

                        pi.y = rpk.ptr(1)[i_idx];
                        pj.y = rpk.ptr(1)[j_idx];

                        pi.z = rpk.ptr(2)[i_idx];
                        pj.z = rpk.ptr(2)[j_idx];

                        ni.x = rpk.ptr(3)[i_idx];
                        nj.x = rpk.ptr(3)[j_idx];

                        ni.y = rpk.ptr(4)[i_idx];
                        nj.y = rpk.ptr(4)[j_idx];

                        ni.z = rpk.ptr(5)[i_idx];                
                        nj.z = rpk.ptr(5)[j_idx];

                        float f1, f2, f3, f4;
                        // Compute the pair NNi to NNj                        
                        computePairFeatures (pi, ni, pj, nj, f1, f2, f3, f4);
                        //if (computePairFeatures (pi, ni, pj, nj, f1, f2, f3, f4))
                        {                            
                            // Normalize the f1, f2, f3 features and push them in the histogram
                            //Using floorf due to changes to MSVC 16.9. See details here: https://devtalk.blender.org/t/cuda-compile-error-windows-10/17886/4
                            //floorf is without std:: see why here: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=79700
                            int find0 = floorf( NR_SPLIT * ((f1 + PI) * (1.f / (2.f * PI))) );                            
                            find0 = min(NR_SPLIT - 1, max(0, find0));

                            int find1 = floorf( NR_SPLIT * ( (f2 + 1.f) * 0.5f ) );
                            find1 = min(NR_SPLIT - 1, max(0, find1));

                            int find2 = floorf( NR_SPLIT * ( (f3 + 1.f) * 0.5f ) );
                            find2 = min(NR_SPLIT - 1, max(0, find2));

                            int h_index = find0 + NR_SPLIT * find1 + NR_SPLIT_2 * find2;
                            atomicAdd(pfh_histogram + h_index, hist_incr);

                            if (enable_rgb)
                            {
                                int ci = __float_as_int(rpk.ptr(6)[i_idx]);
                                int cj = __float_as_int(rpk.ptr(6)[j_idx]);

                                float f5, f6, f7;
                                computeRGBPairFeatures_RGBOnly(ci, cj, f5, f6, f7);

                                // color ratios are in [-1, 1]
                                int find4 = floorf(NR_SPLIT * ((f5 + 1.f) * 0.5f));
                                find4 = min(NR_SPLIT - 1, max(0, find4));

                                int find5 = floorf(NR_SPLIT * ((f6 + 1.f) * 0.5f));
                                find5 = min(NR_SPLIT - 1, max(0, find5));

                                int find6 = floorf(NR_SPLIT * ((f7 + 1.f) * 0.5f));
                                find6 = min(NR_SPLIT - 1, max(0, find6));

                                // and the colors
                                h_index = NR_SPLIT_3 + find4 + NR_SPLIT * find5 + NR_SPLIT_2 * find6;
                                atomicAdd(pfh_histogram + h_index, hist_incr);               
                            }
                        }           
                    }
                }
                __syncthreads();
                Block::copy(pfh_histogram, pfh_histogram + FSize, output.ptr(idx));
            }

            template<class It> 
            __device__ __forceinline__ float3 fetch(It ptr, int index) const
            {
                //return tr(ptr[index]);
                return *(float3*)&ptr[index];
            }
        };

        __global__ void repackKernel(const Repack<false> repack) { repack(); }
        __global__ void pfhKernel(const Pfh125<false> pfh125) { pfh125(); }
    }    
}

void pcl::device::repackToAosForPfh(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<float>& data_rpk, int& max_elems_rpk)
{   
    max_elems_rpk = (neighbours.max_elems/32 + 1) * 32;
    data_rpk.create(6, (int)neighbours.sizes.size() * max_elems_rpk);

    Repack<false> rpk;
    rpk.sizes = neighbours.sizes;
    rpk.gindices = neighbours;

    rpk.cloud = cloud;
    rpk.normals = normals;
    rpk.work_size = (int)neighbours.sizes.size();
    
    rpk.output = data_rpk;    
    rpk.max_elems = max_elems_rpk;

    int block = Repack<false>::CTA_SIZE;        
    int grid = divUp(rpk.work_size, Repack<false>::WARPS);
    
    device::repackKernel<<<grid, block>>>(rpk);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );

    //printFuncAttrib(repackKernel);
}

void pcl::device::computePfh125(const DeviceArray2D<float>& data_rpk, int max_elems_rpk, const NeighborIndices& neighbours, DeviceArray2D<PFHSignature125>& features)
{
    Pfh125<false> fph;
    fph.work_size = neighbours.sizes.size();
    fph.sizes = neighbours.sizes;
    fph.rpk = data_rpk;
    fph.max_elems = max_elems_rpk;                       
    fph.output = features;

    int block = Pfh125<false>::CTA_SIZE;        
    int grid = (int)fph.work_size;    

    device::pfhKernel<<<grid, block>>>(fph);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );

    //printFuncAttrib(pfhKernel);
}


namespace pcl
{
    namespace device
    {
           
        __global__ void repackRgbKernel(const Repack<true> repack) { repack(); }
        __global__ void pfhRgbKernel(const Pfh125<true> pfhrgb125) { pfhrgb125(); }
    }
}


void pcl::device::repackToAosForPfhRgb(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<float>& data_rpk, int& max_elems_rpk)
{   
    max_elems_rpk = (neighbours.max_elems/32 + 1) * 32;
    data_rpk.create(7, (int)neighbours.sizes.size() * max_elems_rpk);

    Repack<true> rpk;
    rpk.sizes = neighbours.sizes;
    rpk.gindices = neighbours;

    rpk.cloud = cloud;
    rpk.normals = normals;
    rpk.work_size = (int)neighbours.sizes.size();
    
    rpk.output = data_rpk;    
    rpk.max_elems = max_elems_rpk;

    int block = Repack<true>::CTA_SIZE;        
    int grid = divUp(rpk.work_size, Repack<true>::WARPS);
    
    device::repackRgbKernel<<<grid, block>>>(rpk);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );

    //printFuncAttrib(repackRgbKernel);
}


void pcl::device::computePfhRgb250(const DeviceArray2D<float>& data_rpk, int max_elems_rpk, const NeighborIndices& neighbours, DeviceArray2D<PFHRGBSignature250>& features)
{
    Pfh125<true> pfhrgb;
    pfhrgb.work_size = neighbours.sizes.size();
    pfhrgb.sizes = neighbours.sizes;
    pfhrgb.rpk = data_rpk;
    pfhrgb.max_elems = max_elems_rpk;                       
    pfhrgb.output = features;

    int block = Pfh125<true>::CTA_SIZE;        
    int grid = (int)pfhrgb.work_size;    

    device::pfhRgbKernel<<<grid, block>>>(pfhrgb);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );

    //printFuncAttrib(pfhRgbKernel);

}