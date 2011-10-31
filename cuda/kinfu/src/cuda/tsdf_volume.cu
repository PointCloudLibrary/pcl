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

using namespace pcl::device;

namespace pcl
{
	namespace device
	{
		__global__ void initializeVolume(PtrStepSz<short2> array)
		{
			int x = threadIdx.x + blockIdx.x * blockDim.x;
			int y = threadIdx.y + blockIdx.y * blockDim.y;

			if (x < array.cols && y < array.rows)
				array.ptr(y)[x] = pack_tsdf(0.f, 0);
		}
			
		struct Tsdf
		{	
			enum
			{
				CTA_SIZE_X = 32, CTA_SIZE_Y = 8,
				MAX_WEIGHT = 1 << 9
			};

			mutable PtrStep<short2> volume;
			float3 volume_size; //in mm

			Intr intr;

			Mat33 Rcurr_inv;
			float3 tcurr;

			PtrStepSz<ushort> depth_raw;

			float tranc_dist;
			
			__device__ __forceinline__ float3 getVoxelGCoo(int x, int y, int z) const
			{
				float3 coo = make_float3(x, y, z);
				coo += 0.5f; //shift to cell center;

				coo.x *= volume_size.x / VOLUME_X;
				coo.y *= volume_size.y / VOLUME_Y;
				coo.z *= volume_size.z / VOLUME_Z;

				return coo;
			}

			__device__ __forceinline__ void operator()() const
			{
				int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
				int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;
                
				if (x >= VOLUME_X || y >= VOLUME_Y)
					return;

				short2 *pos = volume.ptr(y) + x;
				int elem_step = volume.step * VOLUME_Y / sizeof(short2);

				for(int z = 0; z < VOLUME_Z; ++z, pos += elem_step)
				{
					float3 v_g = getVoxelGCoo(x, y, z);   //3 // p                    
					
					//tranform to curr cam coo space 
					float3 v = Rcurr_inv * (v_g - tcurr); //4 

					int2 coo; //project to current cam
					coo.x = __float2int_rn(v.x * intr.fx/v.z + intr.cx); 
					coo.y = __float2int_rn(v.y * intr.fy/v.z + intr.cy); 
                    
					if (coo.x >= 0 && coo.y >= 0 && coo.x < depth_raw.cols && coo.y < depth_raw.rows) //6
					{
						int Dp = depth_raw.ptr(coo.y)[coo.x];

						if (Dp != 0)
						{                                           
                            float xl = (coo.x - intr.cx)/intr.fx;
                            float yl = (coo.y - intr.cy)/intr.fy;                            
                            float lambda_inv = rsqrtf(xl * xl + yl * yl + 1);

							float sdf = norm(tcurr - v_g) * lambda_inv - Dp;

                            sdf *= (-1);
                                                       
                            if (sdf >= -tranc_dist) 
                            {
                                float tsdf = fmin(1, sdf/tranc_dist);

                                //printf("(%f, %f, %f) (%d, %d) -> Dp: %d - Dist: %f - sdf: %f >> tsdf: %f\n", v_g.x, v_g.y, v_g.z, coo.x, coo.y, Dp, norm(tcurr - v_g)*lambda_inv, sdf, tsdf);
                                
                                int weight_prev;
							    float tsdf_prev;					
							
							    //read and unpack
							    unpack_tsdf(*pos, tsdf_prev, weight_prev);

							    const int Wrk = 1;

                                float tsdf_new = (tsdf_prev*weight_prev + Wrk*tsdf)/(weight_prev+Wrk);
                                int weight_new = min(weight_prev + Wrk, MAX_WEIGHT);
														    
							    *pos = pack_tsdf(tsdf_new, weight_new);                            
                            }   
                            else
                            {

                                //printf("(%f, %f, %f) (%d, %d) -> Dp: %d - Dist: %f - sdf: %f >> tsdf: null\n", v_g.x, v_g.y, v_g.z, coo.x, coo.y, Dp, norm(tcurr - v_g)*lambda_inv, sdf);

                            }
						}
					}
				}				
			}
		};

		__global__ void integrateTsdfKernel(const Tsdf tsdf) { tsdf(); }
	}
}

void pcl::device::initVolume(PtrStepSz<short2> array)
{
	dim3 block(32, 16);
	dim3 grid(1,1,1);
	grid.x = divUp(array.cols, block.x);
	grid.y = divUp(array.rows, block.y);

	initializeVolume<<<grid, block>>>(array);
	cudaSafeCall( cudaGetLastError() );
	cudaSafeCall(cudaDeviceSynchronize());
}

void pcl::device::integrateTsdfVolume(const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume)
{
	Tsdf tsdf;
    
	tsdf.volume = volume;
	tsdf.volume_size = volume_size;

	tsdf.intr = intr;

	tsdf.Rcurr_inv = Rcurr_inv;
	tsdf.tcurr = tcurr;
	tsdf.depth_raw = depth_raw;

	tsdf.tranc_dist = tranc_dist;
		
	dim3 block(Tsdf::CTA_SIZE_X, Tsdf::CTA_SIZE_Y);
	dim3 grid(divUp(VOLUME_X, block.x), divUp(VOLUME_Y, block.y));
	
	integrateTsdfKernel<<<grid, block>>>(tsdf);    
	cudaSafeCall( cudaGetLastError() );
	cudaSafeCall(cudaDeviceSynchronize());
}