#pragma once 

#include "pcl/gpu/utils/device/limits.hpp"
#include "pcl/gpu/utils/device/vector_math.hpp"

#include "internal.hpp"

namespace pcl
{
    namespace device
    {       
        __device__ __forceinline__ float3 operator*(const Mat33& m, const float3& vec)
        {
            return make_float3(dot(m.data[0], vec), dot(m.data[1], vec), dot(m.data[2], vec));			
        }		
    }
}