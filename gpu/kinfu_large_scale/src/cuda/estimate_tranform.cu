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
//#include <boost/graph/buffer_concepts.hpp>
//#include <pcl/gpu/utils/device/block.hpp>
//#include <pcl/gpu/utils/device/funcattrib.hpp>
//#include <pcl/gpu/utils/timers_cuda.hpp>

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      template<typename T>
      struct TransformEstimator
      {
        enum
        {
          CTA_SIZE_X = 32,
          CTA_SIZE_Y = 8,
          CTA_SIZE = CTA_SIZE_X * CTA_SIZE_Y
        };

        struct plus
        {
          __forceinline__ __device__ T
          operator () (const T &lhs, const volatile T &rhs) const {
            return lhs + rhs;
          }
        };

        PtrStep<float> v_dst;
        PtrStep<float> n_dst;
        PtrStep<float> v_src;
        PtrStepSz<short2> coresp;

        mutable PtrStep<T> gbuf;

        __device__ __forceinline__ void
        operator () () const
        {
          int x = threadIdx.x + blockIdx.x * blockDim.x;
          int y = threadIdx.y + blockIdx.y * blockDim.y;

          float row[7];
          row[0] = row[1] = row[2] = row[3] = row[4] = row[5] = row[6] = 0.f;

          if (x < coresp.cols || y < coresp.rows)
          {
            short2 ukr = coresp.ptr (y)[x];

            if (ukr.x != -1)
            {
              float3 n;
              n.x = n_dst.ptr (ukr.y                  )[ukr.x];
              n.y = n_dst.ptr (ukr.y +     coresp.rows)[ukr.x];
              n.z = n_dst.ptr (ukr.y + 2 * coresp.rows)[ukr.x];

              float3 d;
              d.x = v_dst.ptr (ukr.y                  )[ukr.x];
              d.y = v_dst.ptr (ukr.y +     coresp.rows)[ukr.x];
              d.z = v_dst.ptr (ukr.y + 2 * coresp.rows)[ukr.x];

              float3 s;
              s.x = v_src.ptr (y                  )[x];
              s.y = v_src.ptr (y +     coresp.rows)[x];
              s.z = v_src.ptr (y + 2 * coresp.rows)[x];

              float b = dot (n, d - s);

              *(float3*)&row[0] = cross (s, n);
              *(float3*)&row[3] = n;
              row[6] = b;
            }
          }

          __shared__ T smem[CTA_SIZE];
          int tid = Block::flattenedThreadId ();

          int shift = 0;
          for (int i = 0; i < 6; ++i)        //rows
          {
            #pragma unroll
            for (int j = i; j < 7; ++j)          // cols + b
            {
              __syncthreads ();
              smem[tid] = row[i] * row[j];
              __syncthreads ();

              Block::reduce<CTA_SIZE>(smem, plus ());

              if (tid == 0)
                gbuf.ptr (shift++)[blockIdx.x + gridDim.x * blockIdx.y] = smem[0];
            }
          }
        }
      };

      template<typename T>
      struct TranformReduction
      {
        enum
        {
          CTA_SIZE = 512,
          STRIDE = CTA_SIZE,

          B = 6, COLS = 6, ROWS = 6, DIAG = 6,
          UPPER_DIAG_MAT = (COLS * ROWS - DIAG) / 2 + DIAG,
          TOTAL = UPPER_DIAG_MAT + B,

          GRID_X = TOTAL
        };

        PtrStep<T> gbuf;
        int length;
        mutable T* output;

        __device__ __forceinline__ void
        operator () () const
        {
          const T *beg = gbuf.ptr (blockIdx.x);
          const T *end = beg + length;

          int tid = threadIdx.x;

          T sum = 0.f;
          for (const T *t = beg + tid; t < end; t += STRIDE)
            sum += *t;

          __shared__ T smem[CTA_SIZE];

          smem[tid] = sum;
          __syncthreads ();

          Block::reduce<CTA_SIZE>(smem, TransformEstimator<T>::plus ());

          if (tid == 0)
            output[blockIdx.x] = smem[0];
        }
      };

      __global__ void
      TransformEstimatorKernel1 (const TransformEstimator<float> te) {
        te ();
      }
      __global__ void
      TransformEstimatorKernel2 (const TranformReduction<float> tr) {
        tr ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      estimateTransform (const MapArr& v_dst, const MapArr& n_dst, 
                                      const MapArr& v_src, const PtrStepSz<short2>& coresp,
                                      DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, 
                                      float* matrixA_host, float* vectorB_host)
      {
        typedef TransformEstimator<float> TEst;
        typedef TranformReduction<float> TRed;

        dim3 block (TEst::CTA_SIZE_X, TEst::CTA_SIZE_Y);
        dim3 grid (1, 1, 1);
        grid.x = divUp (coresp.cols, block.x);
        grid.y = divUp (coresp.rows, block.y);

        mbuf.create (TRed::TOTAL);
        if (gbuf.rows () != TRed::TOTAL || gbuf.cols () < (int)(grid.x * grid.y))
          gbuf.create (TRed::TOTAL, grid.x * grid.y);

        TEst te;
        te.n_dst = n_dst;
        te.v_dst = v_dst;
        te.v_src = v_src;
        te.coresp = coresp;
        te.gbuf = gbuf;

        TransformEstimatorKernel1<<<grid, block>>>(te);
        cudaSafeCall ( cudaGetLastError () );
        //cudaSafeCall(cudaDeviceSynchronize());

        TRed tr;
        tr.gbuf = gbuf;
        tr.length = grid.x * grid.y;
        tr.output = mbuf;

        TransformEstimatorKernel2<<<TRed::TOTAL, TRed::CTA_SIZE>>>(tr);

        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());

        float host_data[TRed::TOTAL];
        mbuf.download (host_data);

        int shift = 0;
        for (int i = 0; i < 6; ++i)  //rows
          for (int j = i; j < 7; ++j)    // cols + b
          {
            float value = host_data[shift++];
            if (j == 6)       // vector b
              vectorB_host[i] = value;
            else
              matrixA_host[j * 6 + i] = matrixA_host[i * 6 + j] = value;
          }
      }
    }
  }
}
