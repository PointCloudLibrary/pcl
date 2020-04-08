#pragma once

#include "internal.h"

namespace pcl
{
	namespace device
	{
		__device__ __forceinline__ float
			getSampleNormal (const float mean, const float cov, curandState* rng_state)
		{
			float rn = curand_normal(rng_state);
			return (rn*cov + mean);
		}
	}
}
