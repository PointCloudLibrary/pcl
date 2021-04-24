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

#ifndef PCL_GPU_KINFU_DEVICE_HPP_
#define PCL_GPU_KINFU_DEVICE_HPP_

#include "utils.hpp" //temporary reimplementing to release kinfu without pcl_gpu_utils

#include "internal.h"

namespace pcl
{
  namespace device
  {   
    #define INV_DIV 3.051850947599719e-5f

    __device__ __forceinline__ void
    pack_tsdf (float tsdf, int weight, short2& value)
    {
      int fixedp = max (-DIVISOR, min (DIVISOR, __float2int_rz (tsdf * DIVISOR)));
      //int fixedp = __float2int_rz(tsdf * DIVISOR);
      value = make_short2 (fixedp, weight);
    }

    __device__ __forceinline__ void
    unpack_tsdf (short2 value, float& tsdf, int& weight)
    {
      weight = value.y;
      tsdf = __int2float_rn (value.x) / DIVISOR;   //*/ * INV_DIV;
    }

    __device__ __forceinline__ float
    unpack_tsdf (short2 value)
    {
      return static_cast<float>(value.x) / DIVISOR;    //*/ * INV_DIV;
    }


    __device__ __forceinline__ float3
    operator* (const Mat33& m, const float3& vec)
    {
      return make_float3 (dot (m.data[0], vec), dot (m.data[1], vec), dot (m.data[2], vec));
    }


    ////////////////////////////////////////////////////////////////////////////////////////
    ///// Prefix Scan utility

    enum ScanKind { exclusive, inclusive };

    template<ScanKind Kind, class T>
    __device__ __forceinline__ T
    scan_warp ( volatile T *ptr, const unsigned int idx = threadIdx.x )
    {
      const unsigned int lane = idx & 31;       // index of thread in warp (0..31) 

      if (lane >=  1) ptr[idx] = ptr[idx -  1] + ptr[idx];
      if (lane >=  2) ptr[idx] = ptr[idx -  2] + ptr[idx];
      if (lane >=  4) ptr[idx] = ptr[idx -  4] + ptr[idx];
      if (lane >=  8) ptr[idx] = ptr[idx -  8] + ptr[idx];
      if (lane >= 16) ptr[idx] = ptr[idx - 16] + ptr[idx];

      if (Kind == inclusive)
        return ptr[idx];
      else
        return (lane > 0) ? ptr[idx - 1] : 0;
    }
  }
}

#endif /* PCL_GPU_KINFU_DEVICE_HPP_ */
