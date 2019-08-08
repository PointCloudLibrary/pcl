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
 * @authors: Anatoly Baskheev
 *
 */

#include "internal.h"
#include <pcl/gpu/utils/device/funcattrib.hpp>
#include <exception>

namespace pcl
{
  namespace device
  {
    template<int N>
    __global__ void smoothKernel(const PtrStep<unsigned char> src, const PtrStep<unsigned short> depth, PtrStepSz<unsigned char> dst,
                                 int patch, int depthThres, int num_parts)
    {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        int patch2 = patch/2;
        if (x >= (dst.cols - patch2*2) || y >= (dst.rows - patch2*2))
            return;

        int depth_center = depth.ptr(y + patch2)[x+patch2];

        unsigned int hist4[(N + 4 - 1) / 4];

        #pragma unroll
        for(int i = 0; i < sizeof(hist4)/sizeof(hist4[0]); ++i)
            hist4[i] = 0;

        for (int dy = 0; dy < patch; ++dy)
        {
            const unsigned short* d = depth.ptr(dy + y);
            const unsigned char * s = src.ptr(dy + y);

            for (int dx = 0; dx < patch; ++dx)
            {
                if (std::abs(d[dx+x] - depth_center) < depthThres)
                {
                    int l = s[dx+x];
                    if (l < num_parts)
                    {
                        int bin = l / sizeof(int);
                        int off = l - bin * sizeof(int);

                        hist4[bin] += (unsigned)(1 << 8*off);
                    }
                }
            }
        }

        int max_label = src.ptr(y + patch2)[x+patch2];
        int max_value = 0;

        #pragma unroll
        for(int i = 0; i < sizeof(hist4)/sizeof(hist4[0]); ++i)
        {
            int bin = hist4[i];
            int val;

            val = (bin >> 0) & 0xFF;
            if (max_value < val)
            {
                max_value = val;
                max_label = sizeof(int)*i+0;
            }

            val = (bin >> 8) & 0xFF;
            if (max_value < val)
            {
                max_value = val;
                max_label = sizeof(int)*i+1;
            }

            val = (bin >> 16) & 0xFF;
            if (max_value < val)
            {
                max_value = val;
                max_label = sizeof(int)*i+2;
            }

            val = (bin >> 24) & 0xFF;
            if (max_value < val)
            {
                max_value = val;
                max_label = sizeof(int)*i+3;
            }
        }

        dst.ptr(y + patch2)[x+patch2] = max_label;
    }
  }
}

void pcl::device::smoothLabelImage(const Labels& src, const Depth& depth, Labels& dst, int num_parts, int  patch_size, int  depthThres)
{
  dst.create(src.rows(), src.cols());

  dim3 block(32, 8);
  dim3 grid(divUp(src.cols(), block.x), divUp(src.rows(), block.y));

  if (num_parts <= 28)
    pcl::device::smoothKernel<28><<<grid, block>>>(src, depth, dst, patch_size, depthThres, num_parts);
  else
  if (num_parts <= 32)
    pcl::device::smoothKernel<32><<<grid, block>>>(src, depth, dst, patch_size, depthThres, num_parts);
  else
    throw std::exception(); //should instantiate another smoothKernel<N>

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}



