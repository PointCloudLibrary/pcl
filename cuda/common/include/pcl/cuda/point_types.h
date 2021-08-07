/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#pragma once

#include <pcl/cuda/common/point_type_rgb.h>
#include <cuda.h>

namespace pcl
{
namespace cuda
{
  /** \brief Default point xyz-rgb structure. */
  struct /*__align__(16)*/ PointXYZRGB
  {
    inline __host__ __device__ PointXYZRGB () {}
    inline __host__ __device__ PointXYZRGB (float _x, float _y, float _z, int _rgb) : 
                                     x(_x), y(_y), z(_z), rgb(_rgb) {}

    // Declare a union for XYZ
    union
    {
      float3 xyz;
      struct
      {
        float x;
        float y;
        float z;
      };
    };
    RGB rgb;
    
    inline __host__ __device__ bool operator == (const PointXYZRGB &rhs)
    {
      return (x == rhs.x && y == rhs.y && z == rhs.z && rgb == rhs.rgb);
    }

    // this allows direct assignment of a PointXYZRGB to float3...
    inline __host__ __device__ operator float3 () const
    {
      return xyz;
    }

    const inline __host__ __device__ PointXYZRGB operator - (const PointXYZRGB &rhs) const
    {
      PointXYZRGB res = *this;
      res -= rhs;
      return (res);
//      xyz = -rhs.xyz;
//      rgb = -rhs.rgb;
//      return (*this -= rhs);
    }

    inline __host__ __device__ PointXYZRGB& operator += (const PointXYZRGB &rhs)
    {
      xyz += rhs.xyz;
      rgb += rhs.rgb;
      return (*this);
    }

    inline __host__ __device__ PointXYZRGB& operator -= (const PointXYZRGB &rhs)
    {
      xyz -= rhs.xyz;
      rgb -= rhs.rgb;
      return (*this);
    }

    inline __host__ __device__ PointXYZRGB& operator *= (const PointXYZRGB &rhs)
    {
      xyz *= rhs.xyz;
      rgb *= rhs.rgb;
      return (*this);
    }

    inline __host__ __device__ PointXYZRGB& operator /= (const PointXYZRGB &rhs)
    {
      xyz /= rhs.xyz;
      rgb /= rhs.rgb;
      return (*this);
    }
  };

  /** \brief Default point xyz-rgb structure. */
  struct __align__(16) PointXYZRGBNormal
  {
    inline __host__ __device__ PointXYZRGBNormal () {}
    inline __host__ __device__ PointXYZRGBNormal (float _x, float _y, float _z, int _rgb) : 
                                     x(_x), y(_y), z(_z), rgb(_rgb) {}

    // Declare a union for XYZ
    union
    {
      float3 xyz;
      struct
      {
        float x;
        float y;
        float z;
      };
    };
    RGB rgb;
    union
    {
      float4 normal;
      struct
      {
        float normal_x;
        float normal_y;
        float normal_z;
        float curvature;
      };
    };
    
    inline __host__ __device__ bool operator == (const PointXYZRGBNormal &rhs)
    {
      return (x == rhs.x && y == rhs.y && z == rhs.z && rgb == rhs.rgb && normal_x == rhs.normal_x && normal_y == rhs.normal_y && normal_z == rhs.normal_z);
    }

    // this allows direct assignment of a PointXYZRGBNormal to float3...
    inline __host__ __device__ operator float3 () const
    {
      return xyz;
    }

    const inline __host__ __device__ PointXYZRGBNormal operator - (const PointXYZRGBNormal &rhs) const
    {
      PointXYZRGBNormal res = *this;
      res -= rhs;
      return (res);
//      xyz = -rhs.xyz;
//      rgb = -rhs.rgb;
//      return (*this -= rhs);
    }

    inline __host__ __device__ PointXYZRGBNormal& operator += (const PointXYZRGBNormal &rhs)
    {
      xyz += rhs.xyz;
      rgb += rhs.rgb;
      normal += rhs.normal;
      return (*this);
    }

    inline __host__ __device__ PointXYZRGBNormal& operator -= (const PointXYZRGBNormal &rhs)
    {
      xyz -= rhs.xyz;
      rgb -= rhs.rgb;
      normal -= rhs.normal;
      return (*this);
    }

    inline __host__ __device__ PointXYZRGBNormal& operator *= (const PointXYZRGBNormal &rhs)
    {
      xyz *= rhs.xyz;
      rgb *= rhs.rgb;
      normal *= rhs.normal;
      return (*this);
    }

    inline __host__ __device__ PointXYZRGBNormal& operator /= (const PointXYZRGBNormal &rhs)
    {
      xyz /= rhs.xyz;
      rgb /= rhs.rgb;
      normal /= rhs.normal;
      return (*this);
    }
  };
} // namespace
} // namespace
