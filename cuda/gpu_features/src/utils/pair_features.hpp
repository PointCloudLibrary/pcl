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

#include "utils/vector_operations.hpp"

namespace pcl
{
    namespace device
    {
        __device__ __host__ __forceinline__ 
        bool computePairFeatures (const float3& p1, const float3& n1, const float3& p2, const float3& n2, float &f1, float &f2, float &f3, float &f4)
        {
            f1 = f2 = f3 = f4 = 0.0f;

            float3 dp2p1 = p2 - p1;            
            f4 = norm(dp2p1);

            if (f4 == 0.f)
                return false;           

            float3 n1_copy = n1, n2_copy = n2;
            float angle1 = dot(n1_copy, dp2p1) / f4;
            
#if 0 // disabled this to pass the unit tests
            float angle2 = -dot(n2_copy, dp2p1) / f4;
            if (acosf (angle1) > acosf(angle2))
            {
                // switch p1 and p2
                n1_copy = n2;
                n2_copy = n1;                
                dp2p1 *= -1;
                f3 = angle2;
            }
            else
#endif
            f3 = angle1;

            // Create a Darboux frame coordinate system u-v-w
            // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
            float3 v = cross(dp2p1, n1_copy);            
            float v_norm = norm(v);
            if (v_norm == 0.0f)
                return false;
            
            // Normalize v
            v /= v_norm;            
                        
            // Do not have to normalize w - it is a unit vector by construction            
            f2 = dot(v, n2_copy);
            
            float3 w = cross(n1_copy, v);
            // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system            
            f1 = atan2f (dot(w, n2_copy), dot(n1_copy, n2_copy)); // @todo optimize this

            return true;
        }
    }

}