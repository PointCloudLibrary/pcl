#pragma once

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/utils/safe_call.hpp>

#include <curand.h>
#include <curand_kernel.h>

namespace pcl
{
	namespace device
	{
		
		struct float8 {	float	x, y, z, w, roll, pitch, yaw, weight;	};	
		using StateType = float8;		

		using PointType = float4;
		using PixelRGB = uchar4;
		using NormalType = float4;
		
		
		void 
			initParticles (PtrSz<curandState> rng_states,
			DeviceArray<float>& initial_noise_mean, DeviceArray<float>& initial_noise_covariance,
			const StateType& representative_state,
			DeviceArray<StateType>& particles);

		void 
			computeTracking ( const DeviceArray2D<PointType>& ref, const DeviceArray2D<PixelRGB>& ref_color,
			const DeviceArray2D<PointType>& input, const DeviceArray2D<PixelRGB>& input_color,
			PtrSz<curandState> rng_states, const DeviceArray<float>& step_noise_covariance,
			DeviceArray<StateType>& particles,
			StateType& representative_state, StateType& motion, float motion_ratio );
		
		/*
		void
			resample (StateXYZ& motion_xyz, StateRPY& motion_rpy, float motion_ratio,
			int num_particles, PtrSz<StateXYZ>& particles_xyz_, PtrSz<StateRPY>& particles_rpy_, PtrSz<float>& particles_weight_);
		void
			weight (const PtrSz<PointType>& input, const PtrSz<PixelRGB>& input_color, 
			const PtrSz<PointType>& ref, const PtrSz<PixelRGB>& ref_color,
			int num_particles, PtrSz<StateXYZ>& particles_xyz_, PtrSz<StateRPY>& particles_rpy_, PtrSz<float>& particles_weight_);
		void
			update (int num_particles, PtrSz<StateXYZ>& particles_xyz_, PtrSz<StateRPY>& particles_rpy_, PtrSz<float>& particles_weight_, 
			StateXYZ& representative_state_xyz, StateRPY& representative_state_rpy,
			StateXYZ& motion_xyz, StateRPY& motion_rpy);
			*/			
	}
}
