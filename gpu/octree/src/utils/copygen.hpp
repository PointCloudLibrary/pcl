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

#ifndef PCL_GPU_OCTREE_COPYGE_HPP
#define PCL_GPU_OCTREE_COPYGE_HPP

#include <pcl/gpu/utils/device/warp.hpp>

namespace pcl
{
    namespace device
    {

        template <typename T>
        __device__ void CopyKernel(const T* in, T *out, int length)
        {
            int STRIDE = gridDim.x * blockDim.x;
            for (int idx = (blockIdx.x * blockDim.x) + threadIdx.x; idx < length; idx += STRIDE) 
            {
                out[idx] = in[idx];
            }
        }

        template <typename T>
        __device__ void GenerateKernel(T* out, int beg, int end)
        {
            int length = end - beg;
            int pos = beg;

            int STRIDE = blockDim.x;
            for (int idx = threadIdx.x; idx < length; idx += STRIDE, pos += STRIDE) 
            {
                out[idx] = pos + threadIdx.x;
            }
        }

        template <typename T>
        __device__ void GenerateTasksKernel(T* out, int beg, int end, int level)
        {
            int length = end - beg;
            int pos = beg;

            int STRIDE = blockDim.x;
            for (int idx = threadIdx.x; idx < length; idx += STRIDE, pos += STRIDE) 
            {
                out[idx] = ((pos + threadIdx.x) << 8) + level;
            }
        }
    }
}

#endif /* PCL_GPU_OCTREE_COPYGE_HPP */