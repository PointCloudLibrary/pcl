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

namespace pcl
{
  namespace device
  {
    namespace kinfuLS
    {
      __device__ unsigned int count = 0;

      struct CorespSearch
      {
        enum { CTA_SIZE_X = 32, CTA_SIZE_Y = 8, CTA_SIZE = CTA_SIZE_X * CTA_SIZE_Y };

        struct plus
        {
          __forceinline__ __device__ int
          operator () (const int &lhs, const volatile int& rhs) const {
            return lhs + rhs;
          }
        };

        PtrStep<float> vmap_g_curr;
        PtrStep<float> nmap_g_curr;

        Mat33 Rprev_inv;
        float3 tprev;

        Intr intr;

        PtrStep<float> vmap_g_prev;
        PtrStep<float> nmap_g_prev;

        float distThres;
        float angleThres;

        mutable PtrStepSz<short2> coresp;

        mutable int* gbuf;

        __device__ __forceinline__ int
        search () const
        {
          int x = threadIdx.x + blockIdx.x * blockDim.x;
          int y = threadIdx.y + blockIdx.y * blockDim.y;

          if (x >= coresp.cols || y >= coresp.rows)
            return 0;

          coresp.ptr (y)[x] = make_short2 (-1, -1);

          float3 ncurr_g;
          ncurr_g.x = nmap_g_curr.ptr (y)[x];

          if (isnan (ncurr_g.x))
            return 0;

          float3 vcurr_g;
          vcurr_g.x = vmap_g_curr.ptr (y              )[x];
          vcurr_g.y = vmap_g_curr.ptr (y + coresp.rows)[x];
          vcurr_g.z = vmap_g_curr.ptr (y + 2 * coresp.rows)[x];

          float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev);         // prev camera coo space

          int2 ukr;         //projection
          ukr.x = __float2int_rn (vcurr_cp.x * intr.fx / vcurr_cp.z + intr.cx);      //4
          ukr.y = __float2int_rn (vcurr_cp.y * intr.fy / vcurr_cp.z + intr.cy);                      //4

          if (ukr.x < 0 || ukr.y < 0 || ukr.x >= coresp.cols || ukr.y >= coresp.rows)
            return 0;

          float3 nprev_g;
          nprev_g.x = nmap_g_prev.ptr (ukr.y)[ukr.x];

          if (isnan (nprev_g.x))
            return 0;

          float3 vprev_g;
          vprev_g.x = vmap_g_prev.ptr (ukr.y              )[ukr.x];
          vprev_g.y = vmap_g_prev.ptr (ukr.y + coresp.rows)[ukr.x];
          vprev_g.z = vmap_g_prev.ptr (ukr.y + 2 * coresp.rows)[ukr.x];

          float dist = norm (vcurr_g - vprev_g);
          if (dist > distThres)
            return 0;

          ncurr_g.y = nmap_g_curr.ptr (y + coresp.rows)[x];
          ncurr_g.z = nmap_g_curr.ptr (y + 2 * coresp.rows)[x];

          nprev_g.y = nmap_g_prev.ptr (ukr.y + coresp.rows)[ukr.x];
          nprev_g.z = nmap_g_prev.ptr (ukr.y + 2 * coresp.rows)[ukr.x];

          float sine = norm (cross (ncurr_g, nprev_g));

          /*if (sine >= 1 || asinf(sine) >= angleThres)
              return 0;*/

          if (/*sine >= 1 || */ sine >= angleThres)
            return 0;

          coresp.ptr (y)[x] = make_short2 (ukr.x, ukr.y);
          return 1;
        }

        __device__ __forceinline__ void
        reduce (int i) const
        {
          __shared__ volatile int smem[CTA_SIZE];

          int tid = Block::flattenedThreadId ();

          smem[tid] = i;
          __syncthreads ();

          Block::reduce<CTA_SIZE>(smem, plus ());

          __shared__ bool isLastBlockDone;

          if (tid == 0)
          {
            gbuf[blockIdx.x + gridDim.x * blockIdx.y] = smem[0];
            __threadfence ();

            unsigned int value = atomicInc (&count, gridDim.x * gridDim.y);

            isLastBlockDone = (value == (gridDim.x * gridDim.y - 1));
          }
          __syncthreads ();

          if (isLastBlockDone)
          {
            int sum = 0;
            int stride = Block::stride ();
            for (int pos = tid; pos < gridDim.x * gridDim.y; pos += stride)
              sum += gbuf[pos];

            smem[tid] = sum;
            __syncthreads ();
            Block::reduce<CTA_SIZE>(smem, plus ());

            if (tid == 0)
            {
              gbuf[0] = smem[0];
              count = 0;
            }
          }
        }

        __device__ __forceinline__ void
        operator () () const
        {
          int mask = search ();
          //reduce(mask); if uncomment -> need to allocate and set gbuf
        }
      };

      __global__ void
      corespKernel (const CorespSearch cs) {
        cs ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
      findCoresp (const MapArr& vmap_g_curr, const MapArr& nmap_g_curr, 
                              const Mat33& Rprev_inv, const float3& tprev, const Intr& intr,
                              const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, 
                              float distThres, float angleThres, PtrStepSz<short2> coresp)
      {
        CorespSearch cs;

        cs.vmap_g_curr = vmap_g_curr;
        cs.nmap_g_curr = nmap_g_curr;

        cs.Rprev_inv = Rprev_inv;
        cs.tprev = tprev;

        cs.intr = intr;

        cs.vmap_g_prev = vmap_g_prev;
        cs.nmap_g_prev = nmap_g_prev;

        cs.distThres = distThres;
        cs.angleThres = angleThres;

        cs.coresp = coresp;

        dim3 block (CorespSearch::CTA_SIZE_X, CorespSearch::CTA_SIZE_Y);
        dim3 grid (divUp (coresp.cols, block.x), divUp (coresp.rows, block.y));

        corespKernel<<<grid, block>>>(cs);

        cudaSafeCall ( cudaGetLastError () );
        cudaSafeCall (cudaDeviceSynchronize ());
      }
    }
  }
}