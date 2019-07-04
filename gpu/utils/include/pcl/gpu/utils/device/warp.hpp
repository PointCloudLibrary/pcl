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

#ifndef PCL_DEVICE_UTILS_WARP_HPP_
#define PCL_DEVICE_UTILS_WARP_HPP_

namespace pcl
{
    namespace device
    {
        struct Warp
        {
            enum
            {
                LOG_WARP_SIZE = 5,
                WARP_SIZE     = 1 << LOG_WARP_SIZE,
                STRIDE        = WARP_SIZE
            };

            /** \brief Returns the warp lane ID of the calling thread. */
            static __device__ __forceinline__ unsigned int laneId()
            {
	            unsigned int ret;
	            asm("mov.u32 %0, %laneid;" : "=r"(ret) );
	            return ret;
            }

            static __device__ __forceinline__ int laneMaskLe()
            {
                unsigned int ret;
	            asm("mov.u32 %0, %lanemask_le;" : "=r"(ret) );
	            return ret;
            }

            static __device__ __forceinline__ int laneMaskLt()
            {
                unsigned int ret;
	            asm("mov.u32 %0, %lanemask_lt;" : "=r"(ret) );
	            return ret;
            }
            static __device__ __forceinline__ unsigned int id()
            {
                int tid = threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
                return tid >> LOG_WARP_SIZE;
            }

            static __device__ __forceinline__ int binaryInclScan(int ballot_mask)
            {
                 return __popc(Warp::laneMaskLe() & ballot_mask);
            }

            static __device__ __forceinline__ int binaryExclScan(int ballot_mask)
            {
                 return __popc(Warp::laneMaskLt() & ballot_mask);
            }            

            template<typename It, typename T>
            static __device__ __forceinline__ void fill(It beg, It end, const T& value)
            {                
                for(It t = beg + laneId(); t < end; t += STRIDE)
                    *t = value;
            }            

            template<typename InIt, typename OutIt>
            static __device__ __forceinline__ OutIt copy(InIt beg, InIt end, OutIt out)
            {      
                unsigned int lane = laneId();               
                InIt  t = beg + lane;
                OutIt o = out + lane;

                for(; t < end; t += STRIDE, o += STRIDE)
                    *o = *t;
                return o;
            }            
           
            template<typename InIt, typename OutIt, class UnOp>
            static __device__ __forceinline__ OutIt transform(InIt beg, InIt end, OutIt out, UnOp op)
            {
                unsigned int lane = laneId();
                InIt  t = beg + lane;
                OutIt o = out + lane;

                for(InIt t = beg + laneId(); t < end; t += STRIDE, o += STRIDE)
                    *o = op(*t);
                return o;
            }

            template<typename InIt1, typename InIt2, typename OutIt, class BinOp>
            static __device__ __forceinline__ OutIt transform(InIt1 beg1, InIt1 end1, InIt2 beg2, OutIt out, BinOp op)
            {
                unsigned int lane = laneId();                
                InIt1 t1 = beg1 + lane; 
                InIt2 t2 = beg2 + lane;
                OutIt o  = out  + lane;
            
                for(; t1 < end1; t1 += STRIDE, t2 += STRIDE, o += STRIDE)
                    *o = op(*t1, *t2);
                return o;
            }

            template<typename OutIt, typename T>
            static __device__ __forceinline__ void yota(OutIt beg, OutIt end, T value)
            {
                unsigned int lane = laneId();                
                value += lane;

                for(OutIt t = beg + lane; t < end; t += STRIDE, value += STRIDE)
                    *t = value;
            }

            template<typename T, class BinOp>
            static __device__ __forceinline__ void reduce(volatile T* buffer, BinOp op)
            {
                unsigned int lane = laneId();
                T val =  buffer[lane];                

                if (lane < 16) 
                {
                    buffer[lane] = val = op(val, buffer[lane + 16]);
                    buffer[lane] = val = op(val, buffer[lane +  8]);
                    buffer[lane] = val = op(val, buffer[lane +  4]);
                    buffer[lane] = val = op(val, buffer[lane +  2]);
                    buffer[lane] = val = op(val, buffer[lane +  1]);
                }
            }

            template<typename T, class BinOp>
            static __device__ __forceinline__ T reduce(volatile T* buffer, T init, BinOp op)
            {
                unsigned int lane = laneId();                
                T val = buffer[lane] = init;
                
                if (lane < 16) 
                {
                    buffer[lane] = val = op(val, buffer[lane + 16]);
                    buffer[lane] = val = op(val, buffer[lane +  8]);
                    buffer[lane] = val = op(val, buffer[lane +  4]);
                    buffer[lane] = val = op(val, buffer[lane +  2]);
                    buffer[lane] = val = op(val, buffer[lane +  1]);
                }
                return buffer[0];
            }       
        };
    }
}

#endif /* PCL_DEVICE_UTILS_WARP_HPP_ */