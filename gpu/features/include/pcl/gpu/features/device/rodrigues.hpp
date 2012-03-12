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


#ifndef PCL_GPU_DEVICE_RODRIGUES_HPP_
#define PCL_GPU_DEVICE_RODRIGUES_HPP_

#include <pcl/gpu/utils/device/vector_math.hpp>

namespace pcl
{
    namespace device
    {
        __device__ __host__ __forceinline__ void AngleAxisf(float angle, const float3& r, float3& row1, float3& row2, float3& row3)
        {
            float cosA, sinA;
            sincosf(angle, &sinA, &cosA);

            row1.x = cosA;  row1.y =  0.f; row1.z =  0.f; 
            row2.x =  0.f;  row2.y = cosA; row2.z =  0.f; 
            row3.x =  0.f;  row3.y =  0.f; row3.z = cosA; 

            /*                 */  row1.y += -r.z * sinA; row1.z +=  r.y * sinA; 
            row2.x +=  r.z * sinA; /*                 */  row2.z += -r.x * sinA; 
            row3.x += -r.y * sinA; row3.y +=  r.x * sinA; /*                 */

            row1.x += r.x * r.x * (1 - cosA);  row1.y += r.x * r.y * (1 - cosA); row1.z += r.x * r.z * (1 - cosA); 
            row2.x += r.y * r.x * (1 - cosA);  row2.y += r.y * r.y * (1 - cosA); row2.z += r.y * r.z * (1 - cosA); 
            row3.x += r.z * r.x * (1 - cosA);  row3.y += r.z * r.y * (1 - cosA); row3.z += r.z * r.z * (1 - cosA);                             
        }

        __device__ __host__ __forceinline__ void Rodrigues(const float3& rvec, float3& row1, float3& row2, float3& row3)
        {
            float angle = norm(rvec);
            float3 unit_axis = make_float3(rvec.x/angle, rvec.y/angle, rvec.z/angle);
            AngleAxisf(angle, unit_axis, row1, row2, row3);
        }
    }
}

#endif /* PCL_GPU_DEVICE_RODRIGUES_HPP_ */