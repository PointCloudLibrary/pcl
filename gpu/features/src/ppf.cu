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
#include "utils/vector_operations.hpp"
#include "pcl/gpu/utils/device/funcattrib.hpp"
#include "pcl/gpu/utils/device/warp.hpp"

#include "pcl/gpu/features/device/rodrigues.hpp"
#include "pcl/gpu/features/device/pair_features.hpp"

namespace pcl
{
    namespace device
    {
        struct PpfImpl
        {
            enum
            {
                CTA_SIZE = 256
            };

            PtrSz<PointType> points;
            const NormalType *normals;
            PtrSz<int> indices;

            mutable PPFSignature* output;

            __device__ __forceinline__ void operator()() const
            {
                int total = points.size * indices.size;
                int idx = blockIdx.x * CTA_SIZE + threadIdx.x;

                if (idx > total)
                    return;

                int index_i = idx / points.size; // indices

                int j = idx % points.size; // points            
                int i = indices.data[index_i];

                PPFSignature out;

                if (i != j)
                {
                    float3 pi = fetch(points.data, i);
                    float3 ni = fetch(normals, i);

                    float3 pj = fetch(points.data, j);
                    float3 nj = fetch(normals, j);

                    //if (computePPFPairFeature(pi, ni, pj, nj, out.f1, out.f2, out.f3, out.f4))
                    if (computePairFeatures(pi, ni, pj, nj, out.f1, out.f2, out.f3, out.f4))
                        computeAlfaM(pi, ni, pj, out.alpha_m);
                    else
                        out.f1 = out.f2 = out.f3 = out.f4 = out.alpha_m = 0.f;
                }
                else
                    out.f1 = out.f2 = out.f3 = out.f4 = out.alpha_m = 0.f;

                output[idx] = out;           
            }

            template<class T> __forceinline__ __device__ float3 fetch(const T* data, int index) const
            {
                //return *(float3*)&data[index];
                T t = data[index];
                return make_float3(t.x, t.y, t.z);
            }
        };

        __global__ void estimatePpfKernel(const PpfImpl ppf) { ppf(); }



        struct PpfRgbImpl
        {
            enum
            {
                CTA_SIZE = 256
            };

            PtrSz<PointXYZRGB> points;
            const NormalType *normals;
            PtrSz<int> indices;

            mutable PPFRGBSignature* output;

            __device__ __forceinline__ void operator()() const
            {
                int total = points.size * indices.size;
                int idx = blockIdx.x * CTA_SIZE + threadIdx.x;

                if (idx > total)
                    return;

                int index_i = idx / points.size; // indices

                int j = idx % points.size; // points            
                int i = indices.data[index_i];

                PPFRGBSignature out;

                if (i != j)
                {
                    int ci;
                    float3 pi = fetchXYZRGB(points.data, i, ci);
                    float3 ni = fetch(normals, i);

                    int cj;
                    float3 pj = fetchXYZRGB(points.data, j, cj);
                    float3 nj = fetch(normals, j);

                    if (computeRGBPairFeatures(pi, ni, ci, pj, nj, cj, out.f1, out.f2, out.f3, out.f4, out.r_ratio, out.g_ratio, out.b_ratio))
                    //if (computePairFeatures(pi, ni, pj, nj, out.f1, out.f2, out.f3, out.f4))
                    {
                        computeAlfaM(pi, ni, pj, out.alpha_m);
                        //computeRGBPairFeatures_RGBOnly(ci, cj, out.r_ratio, out.g_ratio, out.b_ratio);
                    }
                    else
                        out.f1 = out.f2 = out.f3 = out.f4 = out.r_ratio =  out.g_ratio = out.b_ratio = out.alpha_m = 0.f;
                }
                else
                    out.f1 = out.f2 = out.f3 = out.f4 = out.r_ratio = out.g_ratio = out.b_ratio = out.alpha_m = 0.f;


                
                output[idx] = out;           
            }

            template<class T> __forceinline__ __device__ float3 fetch(const T* data, int index) const
            {
                //return *(float3*)&data[index];
                T t = data[index];
                return make_float3(t.x, t.y, t.z);
            }

            __forceinline__ __device__ float3 fetchXYZRGB(const PointXYZRGB* data, int index, int& color) const
            {
                float4 xyzrgb = data[index];
                color = __float_as_int(xyzrgb.w);
                return make_float3(xyzrgb.x, xyzrgb.y, xyzrgb.z);
            }
        };

        __global__ void estimatePpfRgbKernel(const PpfRgbImpl ppfrgb) { ppfrgb(); }

    }
}

void pcl::device::computePPF(const PointCloud& input, const Normals& normals, const Indices& indices, DeviceArray<PPFSignature>& output)
{
    int total = (int)input.size() * (int)indices.size();
    output.create(total);

    PpfImpl ppf;
    ppf.points = input;
    ppf.normals = normals;
    ppf.indices = indices;
    ppf.output = output;

    int block = PpfImpl::CTA_SIZE;
    int grid = divUp(total, block);
    estimatePpfKernel<<<grid, block>>>(ppf);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );

    //printFuncAttrib(estimatePpfKernel);
}


void pcl::device::computePPFRGB(const PointXYZRGBCloud& input, const Normals& normals, const Indices& indices, DeviceArray<PPFRGBSignature>& output)
{
    int total = (int)input.size() * (int)indices.size();
    output.create(total);

    PpfRgbImpl ppfrgb;
    ppfrgb.points = input;
    ppfrgb.normals = normals;
    ppfrgb.indices = indices;
    ppfrgb.output = output;

    int block = PpfRgbImpl::CTA_SIZE;
    int grid = divUp(total, block);
    estimatePpfRgbKernel<<<grid, block>>>(ppfrgb);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );

    //printFuncAttrib(estimatePpfRgbKernel);
}


namespace pcl
{
    namespace device
    {
        struct PpfRgbRegionImpl
        {
            enum
            {
                CTA_SIZE = 256,
                WARPS = CTA_SIZE / Warp::WARP_SIZE,
                FSize = sizeof(PPFRGBSignature)/sizeof(float),
                FSizeWithoutAlfaM = FSize - 1
            };

            struct plus 
            {              
                __forceinline__ __device__ float operator()(const float &lhs, const volatile float& rhs) const { return lhs + rhs; }
            }; 

            const PointXYZRGB* points;
            const NormalType* normals;
            PtrSz<int> indices;
            
            PtrStep<int> gindices;            
            const int *sizes;         

            mutable PPFRGBSignature* output;

            __device__ __forceinline__ void operator()() const
            {        
                int tid = threadIdx.x;
                int warpid = Warp::id();
                int index_i = blockIdx.x * WARPS + warpid;

                if (index_i >= indices.size)
                    return;

                int i = indices[index_i];
                int size = sizes[index_i];
                const int* ginds = gindices.ptr(index_i);
                
                int lane = Warp::laneId();
                
                __shared__ float3 points_buf[WARPS];
                __shared__ float3 normasl_buf[WARPS];
                __shared__ int colors_buf[WARPS];

                if (lane == 0)
                {                    
                    points_buf[warpid]  = fetchXYZRGB(points, i, colors_buf[warpid]);                    
                    normasl_buf[warpid] = fetch(normals, i);
                }

                __shared__ float cta_buf[7][CTA_SIZE + 1];
                cta_buf[0][tid] = cta_buf[1][tid] = cta_buf[2][tid] = cta_buf[3][tid] = 0.f;
                cta_buf[4][tid] = cta_buf[5][tid] = cta_buf[6][tid] = 0.f;
  
                for(int c = lane; c < size; c+= Warp::STRIDE)
                {
                    int j = ginds[c];
                                                             
                    if (i != j)
                    {                        
                        int cj;
                        float3 pj = fetchXYZRGB(points, j, cj);
                        float3 nj = fetch(normals, j);
                    
                        float f1, f2, f3, f4, r_ratio, g_ratio, b_ratio;
                        if (computeRGBPairFeatures(points_buf[warpid], normasl_buf[warpid], colors_buf[warpid], pj, nj, cj, f1, f2, f3, f4, r_ratio, g_ratio, b_ratio))
                        //computeRGBPairFeatures(points_buf[warpid], normasl_buf[warpid], colors_buf[warpid], pj, nj, cj, f1, f2, f3, f4, r_ratio, g_ratio, b_ratio);
                        {
                            cta_buf[0][tid] += f1;
                            cta_buf[1][tid] += f2;
                            cta_buf[2][tid] += f3;
                            cta_buf[3][tid] += f4;
                            cta_buf[4][tid] += r_ratio;
                            cta_buf[5][tid] += g_ratio;
                            cta_buf[6][tid] += b_ratio;    
                        }
                    }
                }
                
                Warp::reduce(&cta_buf[0][tid - lane], plus());
                Warp::reduce(&cta_buf[1][tid - lane], plus());
                Warp::reduce(&cta_buf[2][tid - lane], plus());
                Warp::reduce(&cta_buf[3][tid - lane], plus());

                Warp::reduce(&cta_buf[4][tid - lane], plus());
                Warp::reduce(&cta_buf[5][tid - lane], plus());
                Warp::reduce(&cta_buf[6][tid - lane], plus());
                
                float val = 0.f;
                if (lane < FSizeWithoutAlfaM)
                    val = cta_buf[lane][tid - lane]/size;

                float *ptr = (float*)&output[index_i];
                if (lane < FSize)
                    ptr[lane] = val;                
            }

            __forceinline__ __device__ float3 fetchXYZRGB(const PointXYZRGB* data, int index, int& color) const
            {
                float4 xyzrgb = data[index];
                color = __float_as_int(xyzrgb.w);
                return make_float3(xyzrgb.x, xyzrgb.y, xyzrgb.z);
            }
            template<class T> __forceinline__ __device__ float3 fetch(const T* data, int index) const
            {
                //return *(float3*)&data[index];
                T t = data[index];
                return make_float3(t.x, t.y, t.z);
            }
        };
        __global__ void estiamtePpfRgbRegionKernel(const PpfRgbRegionImpl impl) { impl(); }
    }
}

void pcl::device::computePPFRGBRegion(const PointXYZRGBCloud& cloud, const Normals& normals, const Indices& indices, const NeighborIndices& nn_indices, DeviceArray<PPFRGBSignature>& output)
{
    output.create(nn_indices.sizes.size());

    PpfRgbRegionImpl impl;
    impl.points = cloud;
    impl.normals = normals;
    impl.indices = indices;
    impl.gindices = nn_indices;
    impl.sizes = nn_indices.sizes;
    impl.output = output;

    int block = PpfRgbRegionImpl::CTA_SIZE;
    int grid  = divUp((int)impl.indices.size, PpfRgbRegionImpl::WARPS);

    estiamtePpfRgbRegionKernel<<<grid, block>>>(impl);

    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());    

    //printFuncAttrib(estiamtePpfRgbRegionKernel);
}