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
#include "pcl/gpu/utils/device/funcattrib.hpp"
#include "pcl/gpu/utils/device/warp.hpp"
#include "pcl/gpu/utils/device/block.hpp"
#include "pcl/gpu/utils/safe_call.hpp"

#include "pcl/gpu/features/device/pair_features.hpp"


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


namespace pcl
{
    namespace device
    {
        __device__ unsigned int count = 0;

        struct VfhDevice
        {
            enum
            {
                CTA_SIZE = 256,

                bins1 = 45,
                bins2 = 45,
                bins3 = 45,
                bins4 = 45,
                bins_vp = 128,

                FSize = bins1 + bins2 + bins3 + bins4 + bins_vp
            };

            float distance_normalization_factor_inv;
            float hist_incr; // Factorization constant
            float hist_incr_size_component;

            bool normalize_distances;

            float3 centroid_p;
            float3 centroid_n;

            float3 d_vp_p;


            float hist_incr_vp;

            PtrSz<int> indices;

            const PointType *points;
            const NormalType *normals;

            mutable PtrStep<float> global_buffer;
            mutable float* output;

            template<typename T> __device__ __forceinline__ float3 fetch(const T* data, int index) const
            {
                T t = data[index];
                return make_float3(t.x, t.y, t.z);
            }

            __device__ __forceinline__ void compute(float *shist_b1, float *shist_b2, float *shist_b3, float *shist_b4, float *shist_vp) const
            {
                int idx = threadIdx.x  + blockIdx.x * CTA_SIZE;

                if (idx > indices.size)
                    return;
            }

            __device__ void operator()() const
            {
                __shared__ float shist[FSize];
                __shared__ bool lastBlock;

                Block::fill(shist, shist + FSize, 0.f);
                __syncthreads();

                float *const shist_b1 = shist;
                float *const shist_b2 = shist_b1 + bins1;
                float *const shist_b3 = shist_b2 + bins2;
                float *const shist_b4 = shist_b3 + bins3;
                float *const shist_vp = shist_b4 + bins4;

                int STRIDE = gridDim.x * CTA_SIZE;
                int idx = blockIdx.x * CTA_SIZE + threadIdx.x;

                for (; idx < indices.size; idx += STRIDE) 
                {
                    int index = indices.data[idx];

                    float3 p = fetch(points, index);
                    float3 n = fetch(normals, index);

                    // spfh
                    int h_index;
                    float f1, f2, f3, f4;
                    
                    if (computePairFeatures(centroid_p, centroid_n, p, n, f1, f2, f3, f4))
                    {
                        // Normalize the f1, f2, f3, f4 features and push them in the histogram
                        h_index = floor (bins1 * ((f1 + M_PI) * (1.f / (2.f * M_PI))));
                        h_index = min(bins1 - 1, max(0, h_index));  
                        atomicAdd(shist_b1 + h_index, hist_incr);    

                        h_index = floor (bins2 * ((f2 + 1.f) * 0.5f));
                        h_index = min(bins2 - 1, max (0, h_index));                                            
                        atomicAdd(shist_b2 + h_index, hist_incr);  

                        h_index = floor (bins3 * ((f3 + 1.f) * 0.5f));
                        h_index = min(bins3 - 1, max (0, h_index));
                        atomicAdd(shist_b3 + h_index, hist_incr);

                        if (normalize_distances)
                            h_index = floor (bins4 * (f4 * distance_normalization_factor_inv));
                        else
                            h_index = __float2int_rn (f4 * 100);

                        h_index = min(bins4 - 1, max (0, h_index));                    
                        atomicAdd(shist_b4 + h_index, hist_incr_size_component);
                    }

                    // viewpoint component
                    float alfa = ((dot(n, d_vp_p) + 1.f) * 0.5f);                    
                    h_index = floor (bins_vp * alfa);
                    h_index = min(bins_vp - 1, max (0, h_index));
                    atomicAdd(shist_vp + h_index, hist_incr_vp);
                
                } /* for (; idx < indices.size; idx += STRIDE)  */

                __syncthreads();

                Block::copy(shist, shist + FSize, global_buffer.ptr(blockIdx.x));

                __threadfence();
                
                if (threadIdx.x == 0)
                {
                    unsigned int value = atomicInc(&count, gridDim.x);    
                    lastBlock = (value == (gridDim.x - 1));
                }
                __syncthreads();
                
                if (lastBlock)
                {
                    int total_rows = gridDim.x;

                    for(int x = threadIdx.x; x < FSize; x += CTA_SIZE)
                    {
                        float sum = 0.f;
                        for(int y = 0; y < total_rows; ++y)
                            sum += global_buffer.ptr(y)[x];

                        output[x] = sum;
                    }

                    if (threadIdx.x == 0)
                        count = 0;
                }
            }
        };

        __global__ void estimateVfhKernel(const VfhDevice vfh) { vfh(); }
    }
}


void pcl::device::VFHEstimationImpl::compute(DeviceArray<VFHSignature308>& feature)
{
    feature.create(1);
    VfhDevice vfh;

    vfh.centroid_p = xyz_centroid;
    vfh.centroid_n = normal_centroid;
    vfh.indices = indices;
    vfh.points = points;
    vfh.normals = normals;
    vfh.normalize_distances = normalize_distances;

    // Compute the direction of view from the viewpoint to the centroid        
    vfh.d_vp_p = normalized (viewpoint - xyz_centroid);   

    vfh.distance_normalization_factor_inv = 1.f;
    if ( normalize_distances ) 
    {
        float3 max_pt = getMaxDistance (points, indices, xyz_centroid);
        vfh.distance_normalization_factor_inv = 1.f / norm(xyz_centroid - max_pt);
    }

    // Factorization constant
    vfh.hist_incr = normalize_bins ? (100.f / (indices.size() - 1)) : 1.f;
    vfh.hist_incr_size_component = size_component ? vfh.hist_incr : 0.f;

    vfh.hist_incr_vp = normalize_bins ? (100.f / indices.size()) : 1.f;


    int device;
    cudaSafeCall( cudaGetDevice(&device) );
    
    cudaDeviceProp prop;    
    cudaSafeCall( cudaGetDeviceProperties(&prop, device) );
    
    int total = static_cast<int> (indices.empty() ? points.size() : indices.size());
    int total_lenght_in_blocks = (total + VfhDevice::CTA_SIZE - 1) / VfhDevice::CTA_SIZE;

    int block = VfhDevice::CTA_SIZE;
    int grid =  min(total_lenght_in_blocks, prop.multiProcessorCount * prop.maxThreadsPerMultiProcessor / VfhDevice::CTA_SIZE);

    DeviceArray2D<float> global_buffer(grid, VfhDevice::FSize);
    
    vfh.global_buffer = global_buffer;
    vfh.output = (float*)feature.ptr();   

    estimateVfhKernel<<<grid, block>>>(vfh);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );    
}

