#include "device.hpp"

using namespace pcl::device;

namespace pcl
{
	namespace device
	{
		__global__ void initializeVolume(PtrStepSz<float> array)
		{
			int x = threadIdx.x + blockIdx.x * blockDim.x;
			int y = threadIdx.y + blockIdx.y * blockDim.y;

			if (x < array.cols && y < array.rows)
				array.ptr(y)[x] = 0.f;
		}
	}
}

void pcl::device::initVolume(PtrStepSz<float> array)
{
	dim3 block(32, 16);
	dim3 grid(1,1,1);
	grid.x = divUp(array.cols, block.x);
	grid.y = divUp(array.rows, block.y);

	initializeVolume<<<grid, block>>>(array);
	cudaSafeCall( cudaGetLastError() );
	cudaSafeCall(cudaDeviceSynchronize());
}

namespace pcl
{
	namespace device
	{
		const float max_trancation =  30.f; // in mm
		const float min_trancation = -30.f; // in mm

		__device__ __forceinline__ float pack(float tsdf, int weight)
		{
			//assumming that fabs(tsdf) <= 1 and weight is interger

			float tmp = fabsf(tsdf) + weight * 2;
			return tsdf < 0 ? -tmp : tmp;
		}

		__device__ __forceinline__ void unpack(float value, float& tsdf, int& weight)
		{	
			float tmp = fabsf(value);
			weight = __float2int_rz(tmp/2);
			tmp = tmp - weight * 2;
			tsdf = value < 0 ? -tmp : tmp;
		}

		struct Tsdf
		{	
			enum
			{
				CTA_SIZE_X = 32, CTA_SIZE_Y = 8,
				MAX_WEIGHT = 1 << 20
			};

			mutable PtrStep<float> volume;
			float3 volume_size; //in mm

			Intr intr;

			Mat33 Rcurr_inv;
			float3 tcurr;

			PtrStepSz<ushort> depth_curr;

			float max_trancation_inv;
			float min_trancation_inv;


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

				float *pos = volume.ptr(y) + x;
				int elem_step = volume.step * VOLUME_Y / sizeof(float);

				for(int z = 0; z < VOLUME_Z; ++z, pos += elem_step)
				{
					float3 v_g = getVoxelGCoo(x, y, z);   //3
					
					//tranform to curr cam coo space 
					float3 v = Rcurr_inv * (v_g - tcurr); //4 

					int2 p; //project to current cam
					p.x = __float2int_rn(v.x * intr.fx/v.z + intr.cx); 
					p.y = __float2int_rn(v.y * intr.fy/v.z + intr.cy); 

					if (p.x >= 0 && p.y >= 0 && p.x < depth_curr.cols && p.y < depth_curr.rows) //6
					{
						ushort Dp = depth_curr.ptr(p.y)[p.x];

						if (Dp != 0)
						{
							float sdf = Dp - norm(tcurr - v_g);

							float tsdf;
							if (sdf > 0)
								tsdf = fmax(1e-5f, fmin( 1.f, sdf * max_trancation_inv));
							else
								tsdf = fmin(-1e-5f, fmax(-1.f, sdf * min_trancation_inv));

							int w_prev;
							float tsdf_prev;					
							
							//read and unpack
							unpack(*pos, tsdf_prev, w_prev);

							const int w_inc = 1;
							
							int w = min(MAX_WEIGHT, w_prev + w_inc);
							float tsdf_avg = (tsdf_prev * w_prev + w * tsdf)/w;

							*pos = pack(tsdf_avg, w);
						}
					}
				}				
			}
		};

		__global__ void integrateTsdfKernel(const Tsdf tsdf) { tsdf(); }
	}
}

void pcl::device::integrateTsdfVolume(const PtrStepSz<ushort>& depth_curr, const Intr& intr, const float3& volume_size, const Mat33& Rcurr_inv, const float3& tcurr, PtrStep<float> volume)
{
	Tsdf tsdf;

	tsdf.volume = volume;
	tsdf.volume_size = volume_size;

	tsdf.intr = intr;

	tsdf.Rcurr_inv = Rcurr_inv;
	tsdf.tcurr = tcurr;
	tsdf.depth_curr = depth_curr;

	tsdf.max_trancation_inv = 1.f/max_trancation;
	tsdf.min_trancation_inv = 1.f/min_trancation;
	
	dim3 block(Tsdf::CTA_SIZE_X, Tsdf::CTA_SIZE_Y);
	dim3 grid(divUp(VOLUME_X, block.x), divUp(VOLUME_Y, block.y));
	
	integrateTsdfKernel<<<grid, block>>>(tsdf);
	cudaSafeCall( cudaGetLastError() );
	cudaSafeCall(cudaDeviceSynchronize());
}