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
#include "pcl/gpu/utils/safe_call.hpp"
#include "pcl/gpu/utils/device/block.hpp"
#include "pcl/gpu/features/device/eigen.hpp"

//#define TWO_STEP_ALG

namespace pcl
{
    namespace device
    {

        __device__ __forceinline__ float3 operator*(const Eigen33::Mat33& m, float3 v)
        {
            float3 r;
            r.x = m[0].x * v.x + m[0].y * v.y + m[0].z * v.z;
            r.y = m[1].x * v.x + m[1].y * v.y + m[1].z * v.z;
            r.z = m[2].x * v.x + m[2].y * v.y + m[2].z * v.z;
            return r;
        }

        struct PrincipalCurvaturesImpl
        {
            enum
            {
                CTA_SIZE = 256,
                WARPS = CTA_SIZE/Warp::WARP_SIZE
            };

            struct plus 
            {              
                __forceinline__ __device__ float operator()(const float &lhs, const volatile float& rhs) const { return lhs + rhs; }
            }; 

            const NormalType *normals;
            const int *indices;

            PtrStep<int> neighbours;            
            const int *sizes;  

            int work_size;

            mutable PtrStep<float> proj_normals;
            int max_elems;

            mutable PrincipalCurvatures* output;

            __device__ __forceinline__ void operator()() const
            {
                __shared__ float cov_buffer[6][CTA_SIZE + 1];

                int warpid = Warp::id();
                int f = blockIdx.x * WARPS + warpid;

                if (f >= work_size)
                    return;
                
                int lane = Warp::laneId();
                int size = sizes[f];

                int p_idx = indices ? indices[f] : f;

                float3 n_idx_f3 = fetch(normals, p_idx);
                float n_idx[] = { n_idx_f3.x, n_idx_f3.y, n_idx_f3.z};

                __shared__ Eigen33::Mat33 mats[WARPS];                

                Eigen33::Mat33& M = mats[warpid];

                /*if (lane == 0)
                {
                    M[0].x = -n_idx.x * n_idx.x; M[0].y = -n_idx.x * n_idx.y; M[0].z = -n_idx.x * n_idx.z;
                    M[1].x = -n_idx.y * n_idx.x; M[1].y = -n_idx.y * n_idx.y; M[1].z = -n_idx.y * n_idx.z;
                    M[2].x = -n_idx.z * n_idx.x; M[2].y = -n_idx.z * n_idx.y; M[2].z = -n_idx.z * n_idx.z;

                    M[0].x += 1.f; M[1].y += 1.f; M[3].z += 1.f;
                }*/

                if (lane < 9)
                {
                    float unit = ((lane & 3) == 0) ? 1.f : 0.f;
                    
                    int y = lane / 3;
                    int x = lane - y * 3;
                                        
                    float* mat = (float*)&M;
                    mat[lane] = unit - n_idx[y] * n_idx[x];
                }                

                const int* neighbs = neighbours.ptr(p_idx);

                int idx_shift = max_elems * f;

                float3 centroid = make_float3(0.f, 0.f, 0.f);                
                for(int i = lane; i < size; i += Warp::STRIDE)
                {
                    int neighb_idx = neighbs[i];

                    float3 normal = fetch(normals, neighb_idx);
                    
                    float3 proj_normal = M * normal;

                    centroid.x += proj_normal.x;
                    centroid.y += proj_normal.y;
                    centroid.z += proj_normal.z;

                    proj_normals.ptr(0)[i + idx_shift] = proj_normal.x;
                    proj_normals.ptr(1)[i + idx_shift] = proj_normal.y;
                    proj_normals.ptr(2)[i + idx_shift] = proj_normal.z;
                }

                volatile float *buffer = &cov_buffer[0][threadIdx.x - lane];
                centroid.x = Warp::reduce(buffer, centroid.x, plus());
                centroid.y = Warp::reduce(buffer, centroid.y, plus());
                centroid.z = Warp::reduce(buffer, centroid.z, plus());                                
                centroid *= 1.f/size;  

                //nvcc bug work workaround.
                __threadfence_block();

                //compute covariance matrix        
                int tid = threadIdx.x;

                for(int i = 0; i < 6; ++i)
                    cov_buffer[i][tid] = 0.f; 

                for(int i = lane; i < size; i += Warp::STRIDE)
                {
                    float3 proj_normal;
                    proj_normal.x = proj_normals.ptr(0)[i + idx_shift];
                    proj_normal.y = proj_normals.ptr(1)[i + idx_shift];
                    proj_normal.z = proj_normals.ptr(2)[i + idx_shift];

                    float3 d = proj_normal - centroid;                                  

                    cov_buffer[0][tid] += d.x * d.x; //cov (0, 0) 
                    cov_buffer[1][tid] += d.x * d.y; //cov (0, 1) 
                    cov_buffer[2][tid] += d.x * d.z; //cov (0, 2) 
                    cov_buffer[3][tid] += d.y * d.y; //cov (1, 1) 
                    cov_buffer[4][tid] += d.y * d.z; //cov (1, 2) 
                    cov_buffer[5][tid] += d.z * d.z; //cov (2, 2)       
                }

                Warp::reduce(&cov_buffer[0][tid - lane], plus());
                Warp::reduce(&cov_buffer[1][tid - lane], plus());
                Warp::reduce(&cov_buffer[2][tid - lane], plus());
                Warp::reduce(&cov_buffer[3][tid - lane], plus());
                Warp::reduce(&cov_buffer[4][tid - lane], plus());
                Warp::reduce(&cov_buffer[5][tid - lane], plus());

                volatile float *cov = &cov_buffer[0][tid-lane];
                if (lane < 6)
                    cov[lane] = cov_buffer[lane][tid-lane];

                if (lane == 0)
                {
                    // Extract the eigenvalues and eigenvectors                    
                    Eigen33 eigen33(cov);

                    Eigen33::Mat33&     tmp = (Eigen33::Mat33&)cov_buffer[1][tid - lane];
                    Eigen33::Mat33& vec_tmp = (Eigen33::Mat33&)cov_buffer[2][tid - lane];
                    Eigen33::Mat33& evecs   = (Eigen33::Mat33&)cov_buffer[3][tid - lane];
                    
                    float3 evals;
                    eigen33.compute(tmp, vec_tmp, evecs, evals);

                    PrincipalCurvatures out;                    
                    out.principal_curvature_x = evecs[2].x;
                    out.principal_curvature_y = evecs[2].y;
                    out.principal_curvature_z = evecs[2].z;
                    float indices_size_inv = 1.f / size;
                    out.pc1 = evals.z * indices_size_inv;
                    out.pc2 = evals.y * indices_size_inv;

                    output[f] = out;
                 }
            }
            
            template<class T> __forceinline__ __device__ float3 fetch(const T* data, int index) const
            {
                //return *(float3*)&data[index];
                T t = data[index];
                return make_float3(t.x, t.y, t.z);
            }
        };

        __global__ void principalCurvaturesKernel(const PrincipalCurvaturesImpl impl) { impl(); }
    }    
}

void pcl::device::computePointPrincipalCurvatures(const Normals& normals, const Indices& indices, const NeighborIndices& neighbours, DeviceArray<PrincipalCurvatures>& output, DeviceArray2D<float>& proj_normals)
{             
    proj_normals.create(3, (int)neighbours.data.size());

    PrincipalCurvaturesImpl impl;
    impl.normals = normals;    
    impl.indices = indices;

    impl.neighbours = neighbours;    
    impl.sizes = neighbours.sizes;

    impl.proj_normals = proj_normals;
    impl.max_elems = neighbours.max_elems;

    impl.output = output;

    impl.work_size = indices.empty() ? (int)normals.size() : (int)indices.size();

    int block = PrincipalCurvaturesImpl::CTA_SIZE;
    int grid = divUp(impl.work_size, PrincipalCurvaturesImpl::WARPS);
    principalCurvaturesKernel<<<grid, block>>>(impl);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );

    //printFuncAttrib(principalCurvaturesKernel);
    //printFuncAttrib(principalCurvaturesStep2<256>);
}
