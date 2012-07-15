#include "device.hpp"
#include <ctime>

namespace pcl
{
	namespace device
	{
		struct KernelPolicy
		{
			enum 
			{
				CTA_SIZE = 256,
				WARP_SIZE = 32,
				WARPS_COUNT = CTA_SIZE/WARP_SIZE,
			};
		};

		struct ParticleInitializer
		{
			PtrSz<float> mean_;
			PtrSz<float> cov_;
			
			float8 representative_state_;
			
			mutable PtrSz<StateType> particles_;
			int num_particles_;

			mutable PtrSz<curandState>	rng_states_;
			unsigned long int						rng_seed;
			
			__device__ __forceinline__ void
				operator () () const
      {
				unsigned int tid = threadIdx.x + blockDim.x;
								
				curandState* rng_state = &rng_states_[tid];
				curand_init ( rng_seed, tid, 0, rng_state );
								
				StateType* p = &particles_[tid];
				p->x	= getSampleNormal(mean_[0], cov_[0], rng_state);
				p->y	= getSampleNormal(mean_[1], cov_[1], rng_state);
				p->z	= getSampleNormal(mean_[2], cov_[2], rng_state);

				p->roll	= getSampleNormal(mean_[3], cov_[3], rng_state);
				p->pitch= getSampleNormal(mean_[4], cov_[4], rng_state);
				p->yaw	= getSampleNormal(mean_[5], cov_[5], rng_state);
				
				p->x	+= representative_state_.x;
				p->y	+= representative_state_.y;
				p->z	+= representative_state_.z;

				p->roll		+= representative_state_.roll;
				p->pitch	+= representative_state_.pitch;
				p->yaw		+= representative_state_.yaw;
								
				p->weight= 1.0f / static_cast<float>(num_particles_);							
			}		
		};
		
		struct ParticleFilter
		{
			PtrStepSz<float4> ref_;
			PtrStepSz<uchar4> ref_color_;

			PtrStepSz<float4> input_;
			PtrStepSz<uchar4> input_color_;

			PtrSz<curandState> rng_states_;
			
			PtrSz<float> step_noise_covariance_;

			PtrSz<float8> particles_;
			
			float8 representative_state_;

			float8 motion_;

			float motion_ratio_;
			

			// previous implementation
			// 1. resampling
			// 2. weight
			// 3. weight normalization
			// 4. update
			// now we want
			// 1. sampling
			// 2. weight
			// 3. weight normalization
			// 4. resampling
			// 5. update
			//
			// consideration
			// 1. weight
			//	- find nearest point
			// 2. resampling  
			//  - build CDF

			__device__ __forceinline__ void
      operator () () const
      {
				// 1. resampling

				__syncthreads();

				// 2. weight
				// transform ref
				// find nearest correspondences
				// coherence				 

				// 3. weight normalization
				// 3.1. find min, max - reduction
				__syncthreads();
				// 3.2. normalize weight using min, max
				__syncthreads();
				// 3.3. sum - reduction
				__syncthreads();
				// 3.4 normalize consider sum
				__syncthreads();

				// 4. update
				// reduction

			}
		};  
		
		__global__ void
			ParticleInitializerKernel(ParticleInitializer pi)
		{
			pi();
		}

	}
}

void 
	pcl::device::initParticles ( PtrSz<curandState> rng_states,
		DeviceArray<float>& initial_noise_mean, DeviceArray<float>& initial_noise_covariance,
		const StateType& representative_state,
		DeviceArray<StateType>& particles )
{
	const int num_particles = particles.size();
		
	ParticleInitializer pi;

	pi.rng_seed = time(NULL);
	pi.rng_states_ = rng_states;

	pi.mean_ = initial_noise_mean;
	pi.cov_ = initial_noise_covariance;

	pi.representative_state_ = representative_state;

	pi.particles_ = particles;
	pi.num_particles_ = particles.size();
		
	int block = pcl::device::KernelPolicy::WARP_SIZE;
	int grid = divUp (particles.size(), block);
	
	ParticleInitializerKernel<<<grid, block>>>(pi);

	cudaSafeCall( cudaGetLastError() );
	cudaSafeCall( cudaDeviceSynchronize() );
}

void 
	pcl::device::computeTracking ( const DeviceArray2D<PointType>& ref, const DeviceArray2D<PixelRGB>& ref_color,
		const DeviceArray2D<PointType>& input, const DeviceArray2D<PixelRGB>& input_color,
		PtrSz<curandState> rng_states, const DeviceArray<float>& step_noise_covariance,
		DeviceArray<StateType>& particles,
		StateType& representative_state, StateType& motion, float motion_ratio )
{

}
