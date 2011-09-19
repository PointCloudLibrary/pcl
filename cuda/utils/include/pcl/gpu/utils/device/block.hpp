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

#ifndef PCL_DEVICE_UTILS_BLOCK_HPP_
#define PCL_DEVICE_UTILS_BLOCK_HPP_

namespace pcl
{
    namespace device
    {
        struct Block
        {            
            static __device__ __forceinline__ unsigned int id()
            {
                return blockIdx.x;
            }

            static __device__ __forceinline__ unsigned int stride()
            {
                return blockDim.x * blockDim.y * blockDim.z;
            }

            static __device__ __forceinline__ void sync()
            {
                __syncthreads();                
            }

            static __device__ __forceinline__ int straightenedThreadId()
            {
                return threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
            }

            template<typename It, typename T>
            static __device__ __forceinline__ void fill(It beg, It end, const T& value)
            {
                int STRIDE = stride();
                It t = beg + straightenedThreadId(); 

                for(; t < end; t += STRIDE)
                    *t = value;
            }

            template<typename InIt, typename OutIt>
            static __device__ __forceinline__ void copy(InIt beg, InIt end, OutIt out)
            {
                int STRIDE = stride();
                InIt  t = beg + straightenedThreadId();
                OutIt o = out + (t - beg);

                for(; t < end; t += STRIDE, o += STRIDE)
                    *o = *t;
            }        
        };
    }
}

#endif /* PCL_DEVICE_UTILS_BLOCK_HPP_ */