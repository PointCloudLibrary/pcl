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

#include "device.hpp"

namespace pcl
{
  namespace device
  {
    const float sigma_color = 30;     //in mm
    const float sigma_space = 4.5;     // in pixels

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    __global__ void
    bilateralKernel (const PtrStepSz<ushort> src, 
                     PtrStep<ushort> dst, 
                     float sigma_space2_inv_half, float sigma_color2_inv_half)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x >= src.cols || y >= src.rows)
        return;

      const int R = 6;       //static_cast<int>(sigma_space * 1.5);
      const int D = R * 2 + 1;

      int value = src.ptr (y)[x];

      int tx = min (x - D / 2 + D, src.cols - 1);
      int ty = min (y - D / 2 + D, src.rows - 1);

      float sum1 = 0;
      float sum2 = 0;

      for (int cy = max (y - D / 2, 0); cy < ty; ++cy)
      {
        for (int cx = max (x - D / 2, 0); cx < tx; ++cx)
        {
          int tmp = src.ptr (cy)[cx];

          float space2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
          float color2 = (value - tmp) * (value - tmp);

          float weight = __expf (-(space2 * sigma_space2_inv_half + color2 * sigma_color2_inv_half));

          sum1 += tmp * weight;
          sum2 += weight;
        }
      }

      int res = __float2int_rn (sum1 / sum2);
      dst.ptr (y)[x] = max (0, min (res, std::numeric_limits<short>::max ()));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    __global__ void
    pyrDownGaussKernel (const PtrStepSz<ushort> src, PtrStepSz<ushort> dst, float sigma_color)
    {
      int x = blockIdx.x * blockDim.x + threadIdx.x;
      int y = blockIdx.y * blockDim.y + threadIdx.y;

      if (x >= dst.cols || y >= dst.rows)
        return;

      const int D = 5;

      int center = src.ptr (2 * y)[2 * x];

      int x_mi = max(0, 2*x - D/2) - 2*x;
      int y_mi = max(0, 2*y - D/2) - 2*y;

      int x_ma = min(src.cols, 2*x -D/2+D) - 2*x;
      int y_ma = min(src.rows, 2*y -D/2+D) - 2*y;
            
      float sum = 0;
      float wall = 0;
      
      float weights[] = {0.375f, 0.25f, 0.0625f} ;

      for(int yi = y_mi; yi < y_ma; ++yi)
          for(int xi = x_mi; xi < x_ma; ++xi)
          {
              int val = src.ptr (2*y + yi)[2*x + xi];

              if (std::abs (val - center) < 3 * sigma_color)
              {                                 
                sum += val * weights[std::abs(xi)] * weights[std::abs(yi)];
                wall += weights[std::abs(xi)] * weights[std::abs(yi)];
              }
          }


      dst.ptr (y)[x] = static_cast<int>(sum /wall);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    __global__ void
    pyrDownKernel (const PtrStepSz<ushort> src, PtrStepSz<ushort> dst, float sigma_color)
    {
      int x = blockIdx.x * blockDim.x + threadIdx.x;
      int y = blockIdx.y * blockDim.y + threadIdx.y;

      if (x >= dst.cols || y >= dst.rows)
        return;

      const int D = 5;

      int center = src.ptr (2 * y)[2 * x];

      int tx = min (2 * x - D / 2 + D, src.cols - 1);
      int ty = min (2 * y - D / 2 + D, src.rows - 1);
      int cy = max (0, 2 * y - D / 2);

      int sum = 0;
      int count = 0;

      for (; cy < ty; ++cy)
        for (int cx = max (0, 2 * x - D / 2); cx < tx; ++cx)
        {
          int val = src.ptr (cy)[cx];
          if (std::abs (val - center) < 3 * sigma_color)
          {
            sum += val;
            ++count;
          }
        }
      dst.ptr (y)[x] = sum / count;
    }

	__global__ void
    truncateDepthKernel(PtrStepSz<ushort> depth, ushort max_distance_mm)
	{
		int x = blockIdx.x * blockDim.x + threadIdx.x;
		int y = blockIdx.y * blockDim.y + threadIdx.y;

		if (x < depth.cols && y < depth.rows)		
			if(depth.ptr(y)[x] > max_distance_mm)
				depth.ptr(y)[x] = 0;
	}
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::device::bilateralFilter (const DepthMap& src, DepthMap& dst)
{
  dim3 block (32, 8);
  dim3 grid (divUp (src.cols (), block.x), divUp (src.rows (), block.y));

  cudaFuncSetCacheConfig (bilateralKernel, cudaFuncCachePreferL1);
  bilateralKernel<<<grid, block>>>(src, dst, 0.5f / (sigma_space * sigma_space), 0.5f / (sigma_color * sigma_color));

  cudaSafeCall ( cudaGetLastError () );
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::device::pyrDown (const DepthMap& src, DepthMap& dst)
{
  dst.create (src.rows () / 2, src.cols () / 2);

  dim3 block (32, 8);
  dim3 grid (divUp (dst.cols (), block.x), divUp (dst.rows (), block.y));

  //pyrDownGaussKernel<<<grid, block>>>(src, dst, sigma_color);
  pyrDownKernel<<<grid, block>>>(src, dst, sigma_color);
  cudaSafeCall ( cudaGetLastError () );
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::device::truncateDepth(DepthMap& depth, float max_distance)
{
  dim3 block (32, 8);
  dim3 grid (divUp (depth.cols (), block.x), divUp (depth.rows (), block.y));

  truncateDepthKernel<<<grid, block>>>(depth, static_cast<ushort>(max_distance * 1000.f));

  cudaSafeCall ( cudaGetLastError () );
}
