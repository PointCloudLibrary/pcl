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
 */

#ifndef PCL_GPU_SURFACE_DEVICE_H_
#define PCL_GPU_SURFACE_DEVICE_H_

#include "internal.h"
#include <pcl/gpu/utils/device/vector_math.hpp>

namespace pcl
{
  namespace device
  {
    /** \brief Computers plane from 3 points (v, v1, v2), ensures that point P lies positive subspace.
      * \param[in] v 3D point volume tsdf volume container
      * \param[in] v1 3D point volume tsdf volume container
      * \param[in] v2 3D point volume tsdf volume container
      * \param[in] p point for sign check of plane coefs (should lie in positive subspace)
      * \return a,b,c,d coefs vector
      */ 
    __device__ __host__ __forceinline__
    float4 compute_plane(const float3& v, const float3& v1, const float3& v2, const float3& p)
    {
      float3 n = cross(v1 - v, v2 - v);

      float d = -dot(n, v);

      if (dot(n, p) + d < 0)
      {
        n*=-1.f;
        d*=-1.f;
      }
      return make_float4(n.x, n.y, n.z, d);
    }

    __device__ __host__ __forceinline__ float3 tr(const PointType& p) { return make_float3(p.x, p.y, p.z); }

    struct LessThanByFacet
    {
      __device__ __forceinline__
      bool operator()(const uint64_type& e1, const int& e2) const
      {
        int i1 = (int)(e1 >> 32);
        return i1 < e2;
      }
    };

    __device__ __host__ __forceinline__ float compue_inv_normal_norm(const float4& p) { return 1.f/sqrt(p.x*p.x + p.y*p.y + p.z*p.z); }


    __device__ __host__ __forceinline__ float4& operator*=(float4& p, float v) { p.x*=v; p.y*=v; p.z*=v; p.w*=v; return p; }    

  }
};

#endif /* PCL_GPU_SURFACE_DEVICE_H_ */
