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
#include "pcl/gpu/utils/device/warp.hpp"
#include "pcl/gpu/utils/device/block.hpp"
#include "pcl/gpu/utils/device/limits.hpp"
#include "pcl/gpu/utils/device/vector_math.hpp"
#include "pcl/gpu/utils/device/functional.hpp"

#include "pcl/gpu/utils/safe_call.hpp"

#include "thrust/transform.h"
#include "thrust/device_ptr.h"

namespace pcl
{
	namespace device
	{		
		//[spinimage][angles] = [0..FSize][..FSize]
		extern __shared__ float simage_angles[];
		
		template<class It> __device__ __forceinline__ float3 fetch(It ptr, int index) { return *(float3*)&ptr[index]; } 
		//template<class It> __device__ __forceinline__ float3 fetch(It ptr, int index) { return tr(ptr[index]); }

		struct UseCustomAxis
		{
			float3 rotation_axis;
			__device__ __forceinline__ float3 getRotationAxes(int /*index*/, const float3& /*normal*/) const { return rotation_axis; }
		};

		struct UseCustomAxesCloud
		{
			const NormalType* rotation_axes_cloud;
			__device__ __forceinline__ float3 getRotationAxes(int index, const float3& /*normal*/) const { return fetch(rotation_axes_cloud, index); }
		};

		struct UseOriginNormal
		{			
			__device__ __forceinline__ float3 getRotationAxes(int /*index*/, const float3& normal) const { return normal; }
		};

		struct Div12eps
		{
            __device__ __forceinline__ float operator()(float v1, float v2) const { return (float)(v1 / ( v2 + numeric_limits<double>::epsilon() )); }
		};

		struct DivValIfNonZero
		{
			float val;
			__device__ __forceinline__ DivValIfNonZero(float value) : val(value) {}
            __device__ __forceinline__ float operator()(float v) const { return val == 0 ? v : v/val; }
		};

		template<bool radial, bool angular, typename AxesStrategy>
		struct SpinImpl : public AxesStrategy
		{	
			enum
			{
                CTA_SIZE = 192
			};
			
			int work_size;
			const int* indices;

			const PointType*  input_cloud;
			const NormalType* input_normals;

			const PointType*  surface;
			const NormalType* normals;

			PtrStep<int> neighbor_indices;
			const int* neighbor_indices_sizes;
			float support_angle_cos;

			int min_neighb;
			int image_width;
			float bin_size;
			int FSize;

			mutable PtrStep<float> output;

			static __device__ __host__ __forceinline__ int computeFSize(int image_width)
			{
				int cols = 1 + image_width * 2;
				int rows = 1 + image_width;
				return cols * rows;
			}
			
			__device__ __forceinline__ void operator()() const				
			{													
				int i_input = blockIdx.x + gridDim.x * blockIdx.y;

				int index = indices[i_input];

				int neighb_count = neighbor_indices_sizes[i_input];
				const int *ginds = neighbor_indices.ptr  (i_input);

				if (neighb_count < min_neighb)
					return;

				//set zeros to spin image
				Block::fill(simage_angles, simage_angles + FSize, 0.f); 
				if (angular) //set zeros to angles
					Block::fill(simage_angles + FSize, simage_angles + FSize + FSize, 0.f);

				__syncthreads();

				float3 origin_point  = fetch(input_cloud, index);
				float3 origin_normal = input_normals ? fetch(input_normals, index) : make_float3(0.f, 0.f, 0.f); 				
				origin_normal = normalized_safe(origin_normal); //normalize if non-zero

                float3 rotation_axis = AxesStrategy::getRotationAxes(index, origin_normal);
				rotation_axis = normalized_safe(rotation_axis); //normalize if non-zero

				const float eps = numeric_limits<float>::epsilon ();

				for(int i_neighb = threadIdx.x; i_neighb < neighb_count; i_neighb += CTA_SIZE)
				{
					int neighb_index = ginds[i_neighb];

					// first, skip the points with distant normals
					float cos_between_normals = -2.f; 
					if (angular || support_angle_cos > 0.f) // not bogus
					{
						float3 normal = normalized(fetch(normals, neighb_index));
						cos_between_normals = dot(origin_normal, normal);						
						cos_between_normals = fmax (-1.f, fmin (1.f, cos_between_normals));

						if (fabs(cos_between_normals) < support_angle_cos)    // allow counter-directed normals
							continue;

						cos_between_normals = fabs(cos_between_normals); // the normal is not used explicitly from now						
					}

					// now compute the coordinate in cylindric coordinate system associated with the origin point
					float3 direction = fetch(surface, neighb_index) - origin_point;											
					float direction_norm = norm (direction);

					// ignore the point itself; it does not contribute really
					if (direction_norm < 10 * eps)  
						continue;				

					// the angle between the normal vector and the direction to the point
					float cos_dir_axis = dot(direction, rotation_axis) / direction_norm;					
					cos_dir_axis = fmax(-1.f, fmin(1.f, cos_dir_axis));

					// compute coordinates w.r.t. the reference frame
					float beta  = numeric_limits<float>::quiet_NaN();
					float alpha = numeric_limits<float>::quiet_NaN();
					
					if (radial) // radial spin image structure
					{
						beta  = asinf(cos_dir_axis);  // yes, arc sine! to get the angle against tangent, not normal!
						alpha = direction_norm;
					}
					else // rectangular spin-image structure
					{
						beta  = direction_norm * cos_dir_axis;
						alpha = direction_norm * sqrt (1.0 - cos_dir_axis*cos_dir_axis);

						if (fabs (beta) >= bin_size * image_width || alpha >= bin_size * image_width)
							continue;  // outside the cylinder
					}

					// bilinear interpolation
					float beta_bin_size = radial ? (PI*0.5f/image_width) : bin_size;
					int beta_bin  = floorf(beta  / beta_bin_size) + image_width;
					int alpha_bin = floorf(alpha / bin_size);

					//alpha_bin = min(simage_cols, max(0, alpha_bin));
					//beta_bin  = min(simage_rows, max(0,  beta_bin));					

					if (alpha_bin == image_width)  // border points
					{
						alpha_bin--;
						// HACK: to prevent a > 1
						alpha = bin_size * (alpha_bin + 1) - eps;
					}
					if (beta_bin == 2*image_width )  // border points
					{
						beta_bin--;
						// HACK: to prevent b > 1
						beta = beta_bin_size * (beta_bin - image_width + 1) - eps;
					}
					
					float a = alpha/bin_size     - alpha_bin;
					float b = beta/beta_bin_size - float(beta_bin-image_width); 

					incSpinI(alpha_bin,   beta_bin,   (1-a) * (1-b));
					incSpinI(alpha_bin+1, beta_bin,      a  * (1-b));
					incSpinI(alpha_bin,   beta_bin+1, (1-a) *    b );
					incSpinI(alpha_bin+1, beta_bin+1,    a  *    b );

					if (angular)
					{
						float anlge_betwwn_normals = acos(cos_between_normals);						
						incAngle(alpha_bin,   beta_bin,   anlge_betwwn_normals * (1-a) * (1-b)); 
						incAngle(alpha_bin+1, beta_bin,   anlge_betwwn_normals *    a  * (1-b));
						incAngle(alpha_bin,   beta_bin+1, anlge_betwwn_normals * (1-a) *    b );
						incAngle(alpha_bin+1, beta_bin+1, anlge_betwwn_normals *    a  *    b );
					}
				}  /* for(int i_neighb = threadIdx.x; i_neighb < neighb_count; i_neighb += CTA_SIZE) */

				__syncthreads();
				
				if (angular)
				{					
					//transform sum to average dividing angle/spinimage element-wize.
					const float *amgles_beg = simage_angles + FSize;
					const float *amgles_end = amgles_beg + FSize;
					const float *images_beg = simage_angles;

					Block::transfrom(amgles_beg, amgles_end, images_beg, output.ptr(i_input), Div12eps());
					////Block::copy(amgles_beg, amgles_end, output.ptr(i_input));
					//Block::copy(images_beg, images_beg + FSize, output.ptr(i_input));
				}
				else
				{
					// copy to compute sum
					Block::copy(simage_angles, simage_angles + FSize, simage_angles + FSize);					
					__syncthreads();

					//compute sum
					Block::reduce_n(simage_angles + FSize, FSize, pcl::device::plus<float>());
					__syncthreads();

					float sum = simage_angles[FSize];
					Block::transfrom(simage_angles, simage_angles + FSize, output.ptr(i_input), DivValIfNonZero(sum));
				}		
			}

			__device__ __forceinline__ void incSpinI(int y, int x, float value) const { atomicAdd(simage_angles       + y * (2*image_width + 1) + x, value); }
			__device__ __forceinline__ void incAngle(int y, int x, float value) const { atomicAdd(simage_angles+FSize + y * (2*image_width + 1) + x, value); }
		};	

		template<typename Impl>
		__global__ void computeSpinKernel(const Impl impl) { impl(); }

		template<typename Impl>
		inline void computeSpinImages_caller(Impl& impl, float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
			const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, int min_neighb, int image_width, float bin_size, PtrStep<float> output)
		{
			impl.work_size = (int)indices.size();
			impl.indices = indices;
			impl.input_cloud = input_cloud;
			impl.input_normals = input_normals;

			impl.surface = surface;
			impl.normals = normals;

			impl.neighbor_indices = neighbours;
			impl.neighbor_indices_sizes = neighbours.sizes;

			impl.min_neighb = min_neighb;
			impl.image_width = image_width;
			impl.bin_size = bin_size;
			impl.support_angle_cos = support_angle_cos;
			impl.FSize = Impl::computeFSize(image_width);

			impl.output = output;

			const int total = (int)indices.size();
			const int max_grid_dim = 65535;			
			const int smem_size = 2 * Impl::computeFSize(image_width) * sizeof(float);

			dim3 block(Impl::CTA_SIZE);			
			dim3 grid(min(total, max_grid_dim), divUp(total, max_grid_dim));

			computeSpinKernel<Impl><<<grid, block, smem_size>>>(impl);
			cudaSafeCall( cudaGetLastError() );
			cudaSafeCall( cudaDeviceSynchronize() );
		}

		template<bool radial, bool angular>
		void computeSpinImagesOriginNormalEx(float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
			const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, 
			int min_neighb, int image_width, float bin_size, PtrStep<float> output)
		{
			SpinImpl<radial, angular, UseOriginNormal> si;
			computeSpinImages_caller(si, support_angle_cos, indices, input_cloud, input_normals, surface, normals, neighbours, min_neighb, image_width, bin_size, output);
		}

		template<bool radial, bool angular>
		void computeSpinImagesCustomAxesEx(float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
			const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, 
			int min_neighb, int image_width, float bin_size, const float3& rotation_axis, PtrStep<float> output)
		{
			SpinImpl<radial, angular, UseCustomAxis> si;
			si.rotation_axis = rotation_axis;
			computeSpinImages_caller(si, support_angle_cos, indices, input_cloud, input_normals, surface, normals, neighbours, min_neighb, image_width, bin_size, output);
		}

		template<bool radial, bool angular>
		void computeSpinImagesCustomAxesCloudEx(float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals, 
			const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, 
			int min_neighb, int image_width, float bin_size, const Normals& rotation_axes_cloud, PtrStep<float> output)
		{

			SpinImpl<radial, angular, UseCustomAxesCloud> si;
			si.rotation_axes_cloud = rotation_axes_cloud;
			computeSpinImages_caller(si, support_angle_cos, indices, input_cloud, input_normals, surface, normals, neighbours, min_neighb, image_width, bin_size, output);		
        }
	}
}

void pcl::device::computeSpinImagesOrigigNormal(bool radial, bool angular, float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
		const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, int min_neighb, int image_width, float bin_size, PtrStep<float> output)
{	
	typedef void (*originNormal)(float, const Indices&, const PointCloud&, const Normals&, const PointCloud&, const Normals&, const NeighborIndices&, int , int , float, PtrStep<float>);

	const originNormal table[2][2] = 
	{
		{ computeSpinImagesOriginNormalEx<false, false>, computeSpinImagesOriginNormalEx<false, true> },
		{ computeSpinImagesOriginNormalEx<true, false>,  computeSpinImagesOriginNormalEx<true, true> }
	};

	table[(int)radial][(int)angular](support_angle_cos, indices, input_cloud, input_normals, surface, normals, neighbours, min_neighb, image_width, bin_size, output);	
}

void pcl::device::computeSpinImagesCustomAxes(bool radial, bool angular, float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
		const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, int min_neighb, int image_width, float bin_size, const float3& rotation_axis, PtrStep<float> output)
{
	typedef void (*customAxes)(float, const Indices&, const PointCloud&, const Normals&, const PointCloud&, const Normals&, const NeighborIndices&, int, int, float, const float3&, PtrStep<float>);

	const customAxes table[2][2] = 
	{
		{ computeSpinImagesCustomAxesEx<false, false>, computeSpinImagesCustomAxesEx<false, true> },
		{ computeSpinImagesCustomAxesEx<true, false>,  computeSpinImagesCustomAxesEx<true, true> }
	};

	table[(int)radial][(int)angular](support_angle_cos, indices, input_cloud, input_normals, surface, normals, neighbours, min_neighb, image_width, bin_size, rotation_axis, output);	
}


void pcl::device::computeSpinImagesCustomAxesCloud(bool radial, bool angular, float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
		const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, int min_neighb, int image_width, float bin_size, const Normals& rotation_axes_cloud, PtrStep<float> output)
{
	typedef void (*customAxesCloud)(float, const Indices&, const PointCloud&, const Normals&, const PointCloud&, const Normals&, const NeighborIndices&, int, int, float, const Normals&, PtrStep<float>);

	const customAxesCloud table[2][2] = 
	{
		{ computeSpinImagesCustomAxesCloudEx<false, false>, computeSpinImagesCustomAxesCloudEx<false, true> },
		{ computeSpinImagesCustomAxesCloudEx<true, false>,  computeSpinImagesCustomAxesCloudEx<true, true> }
	};

	table[(int)radial][(int)angular](support_angle_cos, indices, input_cloud, input_normals, surface, normals, neighbours, min_neighb, image_width, bin_size, rotation_axes_cloud, output);	
};

namespace pcl
{
	namespace device
	{
		struct GtThan
		{
			int val;
			GtThan(int value) : val(value) {}
			__device__ __forceinline__ unsigned char operator()(int size) const { return size > val ? 1 : 0; }
		};
	}
}

void pcl::device::computeMask(const NeighborIndices& neighbours, int min_neighb, DeviceArray<unsigned char>& mask)
{
	thrust::device_ptr<int> beg((int*)neighbours.sizes.ptr());
	thrust::device_ptr<int> end = beg + neighbours.sizes.size();
	thrust::device_ptr<unsigned char> out(mask.ptr());
	
	thrust::transform(beg, end, out, GtThan(min_neighb));
	cudaSafeCall( cudaGetLastError() );
	cudaSafeCall( cudaDeviceSynchronize() );	
}
