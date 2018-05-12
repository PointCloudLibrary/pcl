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
#include "pcl/gpu/utils/device/functional.hpp"
#include "pcl/gpu/utils/device/funcattrib.hpp"
#include "pcl/gpu/utils/timers_cuda.hpp"
#include "pcl/gpu/features/device/pair_features.hpp"

#include <iostream>

using namespace pcl::gpu;
using namespace pcl::device;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace pcl
{
    namespace device
    {        
        struct KernelBase
        {
            enum 
            { 
                CTA_SIZE = 256,                
                WAPRS = CTA_SIZE/Warp::WARP_SIZE,

                bins1 = 11,
                bins2 = 11,
                bins3 = 11, 

                FSize = bins1 + bins2 + bins3
            };

            const PointType* point_cloud;                           
            size_t work_size;

            PtrStep<int> gindices;            
            const int* sizes;            

            template<class It> 
            __device__ __forceinline__ float3 fetch(It ptr, int index) const
            {
                //return tr(ptr[index]);
                return *(float3*)&ptr[index];
            }
        };

        struct Spfh : public KernelBase
        {                    
            const NormalType* normals;
            mutable PtrStep<float> output;
            const int *indices;

            __device__ __forceinline__ void operator()() const
            {
                __shared__ float shists[WAPRS * FSize];

                __shared__ float3 current_point[WAPRS];                            
                __shared__ float3 current_nomal[WAPRS];

                int lane = Warp::laneId();
                int warp_idx = Warp::id();    
                int idx = WAPRS * blockIdx.x + warp_idx;

                if (idx >= work_size)
                    return;
                
                int point_idx = indices ? indices[idx] : idx;

                float* shist = shists + warp_idx * FSize;
                float* shist_b1 = shist;
                float* shist_b2 = shist_b1 + bins1;
                float* shist_b3 = shist_b2 + bins2;

                Warp::fill(shist, shist + FSize, 0.f);

                if (lane == 0)
                {
                    current_point[warp_idx] = fetch(point_cloud, point_idx);
                    current_nomal[warp_idx] = fetch(normals, point_idx);
                }

                const int *ginds = gindices.ptr(idx);
                int size = sizes[idx];

                float hist_incr = 100.f / (float)(size - 1); // or 100/(size - 1) ???

                //now [inds, inds + size) contains indices of neighb points for idx-th point in cloud
                //this list also contains idx itseelf.               

                for(int i = lane; i < size; i += Warp::STRIDE)
                {
                    int point_index = ginds[i];
                    if (point_index != point_idx) // skip itself
                    {                        
                        float3 p = fetch(point_cloud, point_index);
                        float3 n = fetch(normals, point_index);

                        int h_index;
                        float f1, f2, f3, f4;
                        if (computePairFeatures (current_point[warp_idx], current_nomal[warp_idx], p, n, f1, f2, f3, f4))
                        {                                                          
                            // Normalize the f1, f2, f3 features and push them in the histogram
                            h_index = floorf (bins1 * ((f1 + M_PI) * (1.0f / (2.0f * M_PI))));
                            h_index = min(bins1 - 1, max(0, h_index));                                                       
                            atomicAdd(shist_b1 + h_index, hist_incr);                            

                            h_index = floorf (bins2 * ((f2 + 1.0f) * 0.5f));
                            h_index = min(bins2 - 1, max (0, h_index));
                            atomicAdd(shist_b2 + h_index, hist_incr);                          

                            h_index = floorf (bins3 * ((f3 + 1.0f) * 0.5f));
                            h_index = min(bins3 - 1, max (0, h_index));

                            atomicAdd(shist_b3 + h_index, hist_incr); 
                        }
                    }
                }                
                float *out = output.ptr(idx);
                Warp::copy(shist, shist + FSize, out);
            }
        };   

        __global__ void SpfhKernel(const Spfh spfh33) { spfh33(); }
    }
}

void pcl::device::computeSPFH(const PointCloud& surface, const Normals& normals, const Indices& indices, const NeighborIndices& neighbours, DeviceArray2D<FPFHSignature33>& spfh33)
{
    spfh33.create(indices.empty() ? (int)surface.size() : (int)indices.size(), 1);

    std::vector<int> inds;
    indices.download(inds);

    Spfh spfh;
    spfh.point_cloud = surface;
    spfh.normals = normals;
    spfh.indices = indices;
    spfh.work_size = spfh33.rows();

    spfh.sizes    = neighbours.sizes;
    spfh.gindices = neighbours;    
    spfh.output = spfh33;

    int block = KernelBase::CTA_SIZE;
    int grid  = divUp((int)spfh.work_size, KernelBase::WAPRS);

    SpfhKernel<<<grid, block>>>(spfh);

    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());
}

namespace pcl
{
    namespace device
    {
        struct Fpfh : public KernelBase
        {   
            const   PtrStep<float> spfh;
            const PointType* surface;
            const int* indices;
            const int* lookup;
            mutable PtrStep<float> fpfh;            
            
            Fpfh(PtrStep<float> spfh_arg) : spfh(spfh_arg) {}

            __device__ __forceinline__ void operator()() const
            {                                
                int lane = Warp::laneId();                
                int warp_idx = Warp::id();    
                int idx = WAPRS * blockIdx.x + warp_idx; // "index in neighbours" == "index in output" == "index in indices".

                if (idx >= work_size)
                    return;

                __shared__ float3 current_point[WAPRS];                            
                __shared__ float features[WAPRS * FSize];
                __shared__ int sindices[CTA_SIZE];

                int point_idx = indices ? indices[idx] : idx; //index in cloud

                if (lane == 0)
                    current_point[warp_idx] = fetch(point_cloud, point_idx);

                volatile float *feature_beg = features    + FSize * warp_idx ;
                volatile float *feature_end = feature_beg + FSize;

                Warp::fill(feature_beg, feature_end, 0.f);

                const int *ginds = gindices.ptr(idx);
                int *sinds = sindices + Warp::WARP_SIZE * warp_idx;
                int size = sizes[idx];

                for(int i = lane; __any(i < size); i += Warp::STRIDE)                
                {
                    if (i < size)
                        sinds[lane] = ginds[i];

                    int inds_num = __popc(__ballot(i < size));

                    for(int j = 0; j < inds_num; ++j)
                    {
                        int point_index = sinds[j]; // index in surface
                        
                        if (surface == point_cloud)
                        {
                            if(point_index != point_idx) //surface == cloud -> point_index and idx are indeces both for the same array.
                            {                            
                                float3 p = fetch(point_cloud, point_index);                        
                                //float dist = norm(p, current_point[warp_idx]);

                                float dx = p.x - current_point[warp_idx].x;
                                float dy = p.y - current_point[warp_idx].y;
                                float dz = p.z - current_point[warp_idx].z;

                                float dist = dx * dx + dy * dy + dz * dz;
                                float weight = 1.f / dist;

                                const float *spfh_ptr = spfh.ptr( lookup ? lookup[point_index] : point_index );

                                Warp::transform(feature_beg, feature_end, spfh_ptr, feature_beg, plusWeighted<volatile float, float>(weight));
                            }
                        }
                        else
                        {
                            float3 p = fetch(surface, point_index);

                            float dx = p.x - current_point[warp_idx].x;
                            float dy = p.y - current_point[warp_idx].y;
                            float dz = p.z - current_point[warp_idx].z;

                            float dist = dx * dx + dy * dy + dz * dz;

                            if (dist == 0)
                                continue;

                            float weight = 1.f / dist;       

                            const float *spfh_ptr = spfh.ptr( lookup[point_index] );

                            Warp::transform(feature_beg, feature_end, spfh_ptr, feature_beg, plusWeighted<volatile float, float>(weight));
                        }
                    }                    
                }

                float *buffer = (float*)&sindices[threadIdx.x - lane];

                normalizeFeature<bins1>(feature_beg,                 buffer, lane);
                normalizeFeature<bins2>(feature_beg + bins1,         buffer, lane);
                normalizeFeature<bins3>(feature_beg + bins1 + bins2, buffer, lane);

                Warp::copy(feature_beg, feature_end, fpfh.ptr(idx));
            }            

            template<int bins>
            __device__ __forceinline__ void normalizeFeature(volatile float *feature, volatile float *buffer, int lane) const
            {                                
                //nomalize buns
                float value = (lane < bins) ? feature[lane] : 0.f;                
                float sum = Warp::reduce(buffer, value, plus<volatile float>());

                if (sum != 0)
                    sum = 100.0 / sum;

                if (lane < bins)
                    feature[lane] *= sum;
            }
        };

        __global__ void FpfhKernel(const Fpfh fpfh33) { fpfh33(); }        
    }
}


void pcl::device::computeFPFH(const PointCloud& cloud, const NeighborIndices& neighbours, const DeviceArray2D<FPFHSignature33>& spfh, DeviceArray2D<FPFHSignature33>& features)
{   
    Fpfh fpfh(spfh);
    fpfh.point_cloud = cloud;    
    fpfh.surface = cloud;
    fpfh.work_size = neighbours.sizes.size();
    fpfh.lookup = 0;
    fpfh.indices = 0;

    fpfh.sizes    = neighbours.sizes;
    fpfh.gindices = neighbours;
    fpfh.fpfh = features;    

    int block = KernelBase::CTA_SIZE;
    int grid  = divUp((int)fpfh.work_size, KernelBase::WAPRS);

    FpfhKernel<<<grid, block>>>(fpfh);

    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());    
}

void pcl::device::computeFPFH(const PointCloud& cloud, const Indices& indices, const PointCloud& surface, 
                              const NeighborIndices& neighbours, DeviceArray<int>& lookup, const DeviceArray2D<FPFHSignature33>& spfh, DeviceArray2D<FPFHSignature33>& features)
{
    Fpfh fpfh(spfh);
    fpfh.point_cloud = cloud;    
    fpfh.surface = surface;
    fpfh.work_size = neighbours.sizes.size();
    fpfh.indices = indices;
    fpfh.lookup = lookup;

    fpfh.sizes    = neighbours.sizes;
    fpfh.gindices = neighbours;
    fpfh.fpfh = features;    

    int block = KernelBase::CTA_SIZE;
    int grid  = divUp((int)fpfh.work_size, KernelBase::WAPRS);

    FpfhKernel<<<grid, block>>>(fpfh);

    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());    

}


