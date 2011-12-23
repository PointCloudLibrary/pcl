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
#include "pcl/gpu/utils/device/funcattrib.hpp"

#include "pcl/gpu/features/device/eigen.hpp"

using namespace pcl::gpu;

namespace pcl
{
    namespace device
    {                 
        struct NormalsEstimator
        {            
            enum
            {
                CTA_SIZE = 256,
                WAPRS = CTA_SIZE / Warp::WARP_SIZE,

                MIN_NEIGHBOORS = 1
            };

            struct plus 
            {              
                __forceinline__ __device__ float operator()(const float &lhs, const volatile float& rhs) const { return lhs + rhs; }
            }; 

            PtrStep<int> indices;
            const int *sizes;
            const PointType *points;

            PtrSz<NormalType> normals;            

            __device__ __forceinline__ void operator()() const
            {                
                __shared__ float cov_buffer[6][CTA_SIZE + 1];

                int warp_idx = Warp::id();
                int idx = blockIdx.x * WAPRS + warp_idx;
                
                if (idx >= normals.size)
                    return;               

                int size = sizes[idx];
                int lane = Warp::laneId();

                if (size < MIN_NEIGHBOORS)
                {
                    const float NaN = numeric_limits<float>::quiet_NaN();
                    if (lane == 0)
                        normals.data[idx] = make_float4(NaN, NaN, NaN, NaN);
                }

                const int *ibeg = indices.ptr(idx);
                const int *iend = ibeg + size;

                //copmpute centroid
                float3 c = make_float3(0.f, 0.f, 0.f);
                for(const int *t = ibeg + lane; t < iend; t += Warp::STRIDE)                
                    c += fetch(*t);

                volatile float *buffer = &cov_buffer[0][threadIdx.x - lane];

                c.x = Warp::reduce(buffer, c.x, plus());
                c.y = Warp::reduce(buffer, c.y, plus());
                c.z = Warp::reduce(buffer, c.z, plus());                                
                c *= 1.f/size;                                                  

                //nvcc bug workaround. if comment this => c.z == 0 at line: float3 d = fetch(*t) - c;
                __threadfence_block();

                //compute covariance matrix        
                int tid = threadIdx.x;

                for(int i = 0; i < 6; ++i)
                    cov_buffer[i][tid] = 0.f;                

                for(const int *t = ibeg + lane; t < iend; t += Warp::STRIDE)   
                {
                    //float3 d = fetch(*t) - c;

                    float3 p = fetch(*t);
                    float3 d = p - c;

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

                //solvePlaneParameters
                if (lane == 0)
                {
                    // Extract the eigenvalues and eigenvectors
                    typedef Eigen33::Mat33 Mat33;
                    Eigen33 eigen33(&cov[lane]);

                    Mat33&     tmp = (Mat33&)cov_buffer[1][tid - lane];
                    Mat33& vec_tmp = (Mat33&)cov_buffer[2][tid - lane];
                    Mat33& evecs   = (Mat33&)cov_buffer[3][tid - lane];
                    float3 evals;

                    eigen33.compute(tmp, vec_tmp, evecs, evals);
                    //evecs[0] - eigenvector with the lowerst eigenvalue

                    // Compute the curvature surface change
                    float eig_sum = evals.x + evals.y + evals.z;
                    float curvature = (eig_sum == 0) ? 0 : fabsf( evals.x / eig_sum );

                    NormalType output;
                    output.w = curvature;

                    // The normalization is not necessary, since the eigenvectors from Eigen33 are already normalized
                    output.x = evecs[0].x;
                    output.y = evecs[0].y;
                    output.z = evecs[0].z;                    

                    normals.data[idx] = output;
                }
            }                  

            __device__ __forceinline__ float3 fetch(int idx) const 
            {
                /*PointType p = points[idx];
                return make_float3(p.x, p.y, p.z);*/
                return *(float3*)&points[idx];                
            }

        };

        __global__ void EstimateNormaslKernel(const NormalsEstimator est) { est(); }


        struct FlipNormal
        {
            const PointType* cloud;
            float3 vp;
            mutable PtrSz<NormalType> normals;

            __device__ __forceinline__ void operator()(int idx, const float3& p) const
            {
                NormalType n = normals[idx];

                float vp_x = vp.x - p.x;
                float vp_y = vp.y - p.y;
                float vp_z = vp.z - p.z;

                // Dot product between the (viewpoint - point) and the plane normal
                float cos_theta = vp_x * n.x + vp_y * n.y + vp_z * n.z;

                // Flip the plane normal
                if (cos_theta < 0)
                {
                    n.x *= -1;
                    n.y *= -1;
                    n.z *= -1;
                    normals[idx] = n;
                }    
            }         
        };

        __global__ void flipNormalTowardsViewpointKernel(const FlipNormal flip) 
        { 
            int idx = threadIdx.x + blockIdx.x * blockDim.x;

            if (idx < flip.normals.size)
            {    
                float3 p = *(float3*)&flip.cloud[idx];
                flip(idx, p);                   
            }
        }
        __global__ void flipNormalTowardsViewpointKernel(const FlipNormal flip, const int* indices) 
        { 
            int idx = threadIdx.x + blockIdx.x * blockDim.x;

            if (idx < flip.normals.size)
            {    
                float3 p = *(float3*)&flip.cloud[indices[idx]];
                flip(idx, p);                   
            }
        }
    }
}

void pcl::device::computeNormals(const PointCloud& cloud, const NeighborIndices& nn_indices, Normals& normals)
{
    NormalsEstimator est;
    est.indices = nn_indices;    
    est.sizes = nn_indices.sizes;
    est.points = cloud;
    est.normals = normals;

    //printFuncAttrib(EstimateNormaslKernel);

    int block = NormalsEstimator::CTA_SIZE;
    int grid = divUp((int)normals.size(), NormalsEstimator::WAPRS);
    EstimateNormaslKernel<<<grid, block>>>(est);

    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());
}

void pcl::device::flipNormalTowardsViewpoint(const PointCloud& cloud, const float3& vp, Normals& normals)
{
    int block = 256;
    int grid = divUp((int)normals.size(), block);

    FlipNormal flip;
    flip.cloud = cloud;
    flip.vp = vp;
    flip.normals = normals;

    flipNormalTowardsViewpointKernel<<<grid, block>>>(flip);
    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());
}

void pcl::device::flipNormalTowardsViewpoint(const PointCloud& cloud, const Indices& indices, const float3& vp, Normals& normals)
{
    int block = 256;
    int grid = divUp((int)normals.size(), block);

    FlipNormal flip;
    flip.cloud = cloud;
    flip.vp = vp;
    flip.normals = normals;

    flipNormalTowardsViewpointKernel<<<grid, block>>>(flip, indices.ptr());
    cudaSafeCall( cudaGetLastError() );        
    cudaSafeCall(cudaDeviceSynchronize());
}