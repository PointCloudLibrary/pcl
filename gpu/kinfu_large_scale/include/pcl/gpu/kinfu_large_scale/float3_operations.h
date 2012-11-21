/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
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
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT> tue.nl)
 * 
 */

#include <iostream>

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      inline float
      dot(const float3& v1, const float3& v2)
      {
        return v1.x * v2.x + v1.y*v2.y + v1.z*v2.z;
      }

      inline float3&
      operator+=(float3& vec, const float& v)
      {
        vec.x += v;  vec.y += v;  vec.z += v; return vec;
      }

      inline float3&
      operator+=(float3& vec, const float3& v)
      {
        vec.x += v.x;  vec.y += v.y;  vec.z += v.z; return vec;
      }
      
      inline float3
      operator+(const float3& v1, const float3& v2)
      {
        return make_float3 (v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
      }
      
      inline float3&
      operator*=(float3& vec, const float& v)
      {
        vec.x *= v;  vec.y *= v;  vec.z *= v; return vec;
      }

      inline float3&
      operator-=(float3& vec, const float& v)
      {
        vec.x -= v;  vec.y -= v;  vec.z -= v; return vec;
      }
      
      inline float3&
      operator-=(float3& vec, const float3& v)
      {
        vec.x -= v.x;  vec.y -= v.y;  vec.z -= v.z; return vec;
      }
      
      inline float3
      operator-(const float3& v1, const float3& v2)
      {
        return make_float3 (v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
      }
      
      inline float3
      operator-(const float3& v1)
      {
        return make_float3 (-v1.x, -v1.y, -v1.z);
      }

      inline float3
      operator-(float3& v1)
      {
        v1.x = -v1.x; v1.y = -v1.y; v1.z = -v1.z; return v1;
      }

      inline float3
      operator*(const float3& v1, const float& v)
      {
        return make_float3 (v1.x * v, v1.y * v, v1.z * v);
      }

      inline float
      norm(const float3& v)
      {
        return sqrt (dot (v, v));
      }

      inline std::ostream&
      operator << (std::ostream& os, const float3& v1)
      {
        os << "[" << v1.x << ", " << v1.y <<  ", " << v1.z<< "]";
        return (os);
      }
      
      /*inline float3
      normalized(const float3& v)
      {
        return v * rsqrt(dot(v, v));
      }*/

      inline float3 
      cross(const float3& v1, const float3& v2)
      {
        return make_float3 (v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
      }
    }
  }
}