#include "device.hpp"

namespace pcl
{
  namespace device
  {
  	struct ParticleInitializer
  	{
  	  PtrSz<float> initial_noise_mean_;
  	  PtrSz<float> initial_noise_covariance_;
  	  
	  float8 representative_state_;
	  
	  PtrSz<float8> particles_;
  	
  	};
  	
  	struct ParticleFilter
  	{
  	  PtrStepSz<float4> ref_;
  	  
  	  PtrStepSz<uchar4> ref_color_;
  	  
  	  PtrStepSz<float4> input_;
  	  
  	  PtrStepSz<uchar4> input_color_;
  	  
	  PtrSz<float> step_noise_covariance_;
	  
	  PtrSz<float8> particles_;
	  
	  float8 representative_state_;
	  
	  float8 motion_;
	  
	  float motion_ratio_;
  	
  	};  
  
  }
}

void 
  pcl::device::initParticles ( DeviceArray<float>& initial_noise_mean, DeviceArray<float>& initial_noise_covariance,
					StateType& representative_state,
					DeviceArray<StateType>& particles)
{

}

void 
  pcl::device::computeTracking ( const DeviceArray2D<PointType>& ref, const DeviceArray2D<PixelRGB>& ref_color,
					const DeviceArray2D<PointType>& input, const DeviceArray2D<PixelRGB>& input_color,
					const DeviceArray<float>& step_noise_covariance,
					DeviceArray<StateType>& particles,
					StateType& representative_state, StateType& motion, float motion_ratio )
{

}
