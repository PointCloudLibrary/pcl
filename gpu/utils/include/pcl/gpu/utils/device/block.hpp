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

			static __device__ __forceinline__ int flattenedThreadId()
			{
				return threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
			}

			template<typename It, typename T>
			static __device__ __forceinline__ void fill(It beg, It end, const T& value)
			{
				int STRIDE = stride();
				It t = beg + flattenedThreadId(); 

				for(; t < end; t += STRIDE)
					*t = value;
			}

			template<typename OutIt, typename T>
			static __device__ __forceinline__ void yota(OutIt beg, OutIt end, T value)
			{
				int STRIDE = stride();
				int tid = flattenedThreadId();                                
				value += tid;

				for(OutIt t = beg + tid; t < end; t += STRIDE, value += STRIDE)
					*t = value;                
			}

			template<typename InIt, typename OutIt>
			static __device__ __forceinline__ void copy(InIt beg, InIt end, OutIt out)
			{
				int STRIDE = stride();
				InIt  t = beg + flattenedThreadId();
				OutIt o = out + (t - beg);

				for(; t < end; t += STRIDE, o += STRIDE)
					*o = *t;
			}

			template<typename InIt, typename OutIt, class UnOp>
			static __device__ __forceinline__ void transform(InIt beg, InIt end, OutIt out, UnOp op)
			{
				int STRIDE = stride();
				InIt  t = beg + flattenedThreadId();
				OutIt o = out + (t - beg);

				for(; t < end; t += STRIDE, o += STRIDE)
					*o = op(*t);
			}

			template<typename InIt1, typename InIt2, typename OutIt, class BinOp>
			static __device__ __forceinline__ void transform(InIt1 beg1, InIt1 end1, InIt2 beg2, OutIt out, BinOp op)
			{
				int STRIDE = stride();
				InIt1 t1 = beg1 + flattenedThreadId();
				InIt2 t2 = beg2 + flattenedThreadId();
				OutIt o  = out + (t1 - beg1);

				for(; t1 < end1; t1 += STRIDE, t2 += STRIDE, o += STRIDE)
					*o = op(*t1, *t2);
			}

			template<int CTA_SIZE, typename T, class BinOp>
			static __device__ __forceinline__ void reduce(volatile T* buffer, BinOp op)
			{
				int tid = flattenedThreadId();
				T val =  buffer[tid];

				if (CTA_SIZE >= 1024) { if (tid < 512) buffer[tid] = val = op(val, buffer[tid + 512]); __syncthreads(); }
				if (CTA_SIZE >=  512) { if (tid < 256) buffer[tid] = val = op(val, buffer[tid + 256]); __syncthreads(); }
				if (CTA_SIZE >=  256) { if (tid < 128) buffer[tid] = val = op(val, buffer[tid + 128]); __syncthreads(); }
				if (CTA_SIZE >=  128) { if (tid <  64) buffer[tid] = val = op(val, buffer[tid +  64]); __syncthreads(); }

				if (tid < 32)
				{
					if (CTA_SIZE >=   64) { buffer[tid] = val = op(val, buffer[tid +  32]); }
					if (CTA_SIZE >=   32) { buffer[tid] = val = op(val, buffer[tid +  16]); }
					if (CTA_SIZE >=   16) { buffer[tid] = val = op(val, buffer[tid +   8]); }
					if (CTA_SIZE >=    8) { buffer[tid] = val = op(val, buffer[tid +   4]); }
					if (CTA_SIZE >=    4) { buffer[tid] = val = op(val, buffer[tid +   2]); }
					if (CTA_SIZE >=    2) { buffer[tid] = val = op(val, buffer[tid +   1]); }
				}
			}

			template<int CTA_SIZE, typename T, class BinOp>
			static __device__ __forceinline__ T reduce(volatile T* buffer, T init, BinOp op)
			{
				int tid = flattenedThreadId();
				T val =  buffer[tid] = init;
				__syncthreads();

				if (CTA_SIZE >= 1024) { if (tid < 512) buffer[tid] = val = op(val, buffer[tid + 512]); __syncthreads(); }
				if (CTA_SIZE >=  512) { if (tid < 256) buffer[tid] = val = op(val, buffer[tid + 256]); __syncthreads(); }
				if (CTA_SIZE >=  256) { if (tid < 128) buffer[tid] = val = op(val, buffer[tid + 128]); __syncthreads(); }
				if (CTA_SIZE >=  128) { if (tid <  64) buffer[tid] = val = op(val, buffer[tid +  64]); __syncthreads(); }

				if (tid < 32)
				{
					if (CTA_SIZE >=   64) { buffer[tid] = val = op(val, buffer[tid +  32]); }
					if (CTA_SIZE >=   32) { buffer[tid] = val = op(val, buffer[tid +  16]); }
					if (CTA_SIZE >=   16) { buffer[tid] = val = op(val, buffer[tid +   8]); }
					if (CTA_SIZE >=    8) { buffer[tid] = val = op(val, buffer[tid +   4]); }
					if (CTA_SIZE >=    4) { buffer[tid] = val = op(val, buffer[tid +   2]); }
					if (CTA_SIZE >=    2) { buffer[tid] = val = op(val, buffer[tid +   1]); }
				}
				__syncthreads();				
				return buffer[0];
			}

			template <typename T, class BinOp>
			static __device__ __forceinline__ void reduce_n(T* data, unsigned int n, BinOp op)
			{
				int ftid = flattenedThreadId();
				int sft = stride();

				if (sft < n)
				{
					for (unsigned int i = sft + ftid; i < n; i += sft)
						data[ftid] = op(data[ftid], data[i]);

					__syncthreads();

					n = sft;
				}

				while (n > 1)
				{
					unsigned int half = n/2;

					if (ftid < half)
						data[ftid] = op(data[ftid], data[n - ftid - 1]);

					__syncthreads();

					n = n - half;
				}
			}
		};
	}
}

#endif /* PCL_DEVICE_UTILS_BLOCK_HPP_ */

