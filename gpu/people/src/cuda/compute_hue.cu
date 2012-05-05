/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 * @authors: Koen Buys, Anatoly Baksheev
 *
 */

#include "internal.h"
#include <pcl/gpu/utils/device/limits.hpp>


namespace pcl
{
  namespace device
  {
    __device__ __host__ __forceinline__ float computeHueFunc (int rgba)
    {
      int r = (rgba      ) & 0xFF;
      int g = (rgba >>  8) & 0xFF;
      int b = (rgba >> 16) & 0xFF;

      int v = max (r, max (g, b));
      float h;

      float div_inv = 1.f / (v - min (r, min (g, b)) );

      if (v == 0)
          return -1;

      if (r == v)
          h = (    (g - b)) * div_inv;
      else if (g == v)
          h = (2 + (b - r)) * div_inv;
      else 
          h = (4 + (r - g)) * div_inv;

      h *= 60;    
      if (h < 0)
          h += 360;

      return h;
    }

  }
}

float pcl::device::computeHue(int rgba) 
{ 
  return computeHueFunc(rgba); 
}

namespace pcl
{
  namespace device
  {
    __global__ void computeHueKernel(const PtrStepSz<int> rgba, const PtrStep<unsigned short> depth, PtrStep<float> hue)
    {
      int x = blockIdx.x * blockDim.x + threadIdx.x;
      int y = blockIdx.y * blockDim.y + threadIdx.y;

      if (x < rgba.cols && y < rgba.rows)
      {
        const float qnan = numeric_limits<float>::quiet_NaN();

        unsigned short d = depth.ptr(y)[x];            
        hue.ptr(y)[x] = (d == 0) ? qnan : computeHueFunc(rgba.ptr(y)[x]);           
      }
    }
  }
}

void pcl::device::computeHueWithNans(const Image& rgba, const Depth& depth, Hue& hue)
{
  hue.create(rgba.rows(), rgba.cols());

  dim3 block(32, 8);
  dim3 grid;

  grid.x = divUp(rgba.cols(), block.x);
  grid.y = divUp(rgba.rows(), block.y);

  computeHueKernel<<<grid, block>>>(rgba, depth, hue);

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}

namespace pcl
{
  namespace device
  {
    __global__ void reprojectDepthKenrel(const PtrStepSz<unsigned short> depth, const Intr intr, PtrStep<float4> cloud)
    {
      int x = blockIdx.x * blockDim.x + threadIdx.x;
      int y = blockIdx.y * blockDim.y + threadIdx.y;

      const float qnan = numeric_limits<float>::quiet_NaN();

      if (x < depth.cols && y < depth.rows)
      {
        float4 p = make_float4(qnan, qnan, qnan, qnan);

        int d = depth.ptr(y)[x];
        float z = d * 0.001f; // mm -> meters

        p.x = z * (x - intr.cx) / intr.fx;
        p.y = z * (y - intr.cy) / intr.fy;
        p.z = z;

        cloud.ptr(y)[x] = p;
      }      
    }
  }
}

void pcl::device::computeCloud(const Depth& depth, const Intr& intr, Cloud& cloud)
{
  cloud.create(depth.rows(), depth.cols());

  dim3 block(32, 8);
  dim3 grid;
  grid.x = divUp(depth.cols(), block.x);
  grid.y = divUp(depth.rows(), block.y);

  reprojectDepthKenrel<<<grid, block>>>(depth, intr, cloud);

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}