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

#ifndef PCL_GPU_FEATURS_DEVICE_VECTOR_OPERATIONS_HPP_
#define PCL_GPU_FEATURS_DEVICE_VECTOR_OPERATIONS_HPP_

#include <pcl/gpu/features/device/rodrigues.hpp>
#include <pcl/gpu/utils/device/vector_math.hpp>

namespace pcl
{
    namespace device
    {       
         __forceinline__ __device__ __host__ float3 operator/(const float3& vec, float val)
        {
            return make_float3(vec.x/val, vec.y/val, vec.z/val);
        }

        __device__ __host__ __forceinline__ float3& operator/=(float3& v, const float& value)
        {
            v.x /= value;
            v.y /= value;
            v.z /= value;
            return v;
        }
             
        __device__ __host__ __forceinline__ float norm(const float3& v1, const float3& v2)
        {
            float dx = v1.x - v2.x;
            float dy = v1.y - v2.y;
            float dz = v1.z - v2.z;
            return sqrtf(dx*dx + dy*dy + dz*dz);
        }
        
        template<typename T> __device__ __forceinline__ float3 tr(const T& v)
        {
            return make_float3(v.x, v.y, v.z);
        }

        inline __host__ __device__ float3 normalize(const float3& v)
        {
            return v * inverse_norm(v);
        }
    }
}

#endif /* PCL_GPU_FEATURS_DEVICE_VECTOR_OPERATIONS_HPP_ */