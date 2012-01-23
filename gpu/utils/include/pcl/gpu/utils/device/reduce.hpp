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

#ifndef PCL_GPU_DEVUCE_REDUCE_HPP_
#define PCL_GPU_DEVUCE_REDUCE_HPP_

namespace pcl
{
    namespace device
    {        
        template <unsigned int CTA_SIZE, typename T, typename BinaryFunction>
        __device__ __forceinline__ void reduce_block(volatile T* data, BinaryFunction op, unsigned int tid = threadIdx.x)
        {
            T val = data[tid];

            //if (CTA_SIZE >= 1024) { if (tid < 512) { data[tid] = val = op(val, data[tdi + 512]); } __syncthreads(); }
            if (CTA_SIZE >=  512) { if (tid < 256) { data[tid] = val = op(val, data[tid + 256]); } __syncthreads(); }
            if (CTA_SIZE >=  256) { if (tid < 128) { data[tid] = val = op(val, data[tid + 128]); } __syncthreads(); }
            if (CTA_SIZE >=  128) { if (tid <  64) { data[tid] = val = op(val, data[tid +  64]); } __syncthreads(); }

            if (tid < 32)
            {
                if (CTA_SIZE >= 64) data[tid] = val = op(val, data[tid + 32]);
                if (CTA_SIZE >= 32) data[tid] = val = op(val, data[tid + 16]);
                if (CTA_SIZE >= 16) data[tid] = val = op(val, data[tid +  8]);
                if (CTA_SIZE >=  8) data[tid] = val = op(val, data[tid +  4]);
                if (CTA_SIZE >=  4) data[tid] = val = op(val, data[tid +  2]);
                if (CTA_SIZE >=  2) data[tid] = val = op(val, data[tid +  1]);
            }            
        };
    }
}


#endif