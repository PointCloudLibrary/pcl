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
//#include <pcl/gpu/utils/device/funcattrib.hpp>
//include <pcl/gpu/utils/device/block.hpp>
//#include <pcl/gpu/utils/device/warp.hpp>

namespace pcl
{
  namespace device
  {

    ////////////////////////////////////////////////////////////////////////////////////////
    ///// Full Volume Scan6

    enum
    {
      CTA_SIZE_X = 32,
      CTA_SIZE_Y = 6,
      CTA_SIZE = CTA_SIZE_X * CTA_SIZE_Y,

      MAX_LOCAL_POINTS = 3
    };

    __device__ int global_count = 0;
    __device__ int output_count;
    __device__ unsigned int blocks_done = 0;

    __shared__ float storage_X[CTA_SIZE * MAX_LOCAL_POINTS];
    __shared__ float storage_Y[CTA_SIZE * MAX_LOCAL_POINTS];
    __shared__ float storage_Z[CTA_SIZE * MAX_LOCAL_POINTS];

    struct FullScan6
    {
      PtrStep<short2> volume;
      float3 cell_size;

      mutable PtrSz<PointType> output;

      __device__ __forceinline__ float
      fetch (int x, int y, int z, int& weight) const
      {
        float tsdf;
        unpack_tsdf (volume.ptr (VOLUME_Y * z + y)[x], tsdf, weight);
        return tsdf;
      }

      __device__ __forceinline__ void
      operator () () const
      {
        int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
        int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;
       
#if CUDART_VERSION >= 9000
        if (__all_sync (__activemask (), x >= VOLUME_X)
            || __all_sync (__activemask (), y >= VOLUME_Y))
          return;
#else 
        if (__all (x >= VOLUME_X) || __all (y >= VOLUME_Y))
          return;
#endif

        float3 V;
        V.x = (x + 0.5f) * cell_size.x;
        V.y = (y + 0.5f) * cell_size.y;

        int ftid = Block::flattenedThreadId ();

        for (int z = 0; z < VOLUME_Z - 1; ++z)
        {
          float3 points[MAX_LOCAL_POINTS];
          int local_count = 0;

          if (x < VOLUME_X && y < VOLUME_Y)
          {
            int W;
            float F = fetch (x, y, z, W);

            if (W != 0 && F != 1.f)
            {
              V.z = (z + 0.5f) * cell_size.z;

              //process dx
              if (x + 1 < VOLUME_X)
              {
                int Wn;
                float Fn = fetch (x + 1, y, z, Wn);

                if (Wn != 0 && Fn != 1.f)
                  if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                  {
                    float3 p;
                    p.y = V.y;
                    p.z = V.z;

                    float Vnx = V.x + cell_size.x;

                    float d_inv = 1.f / (std::abs (F) + std::abs (Fn));
                    p.x = (V.x * std::abs (Fn) + Vnx * std::abs (F)) * d_inv;

                    points[local_count++] = p;
                  }
              }               /* if (x + 1 < VOLUME_X) */

              //process dy
              if (y + 1 < VOLUME_Y)
              {
                int Wn;
                float Fn = fetch (x, y + 1, z, Wn);

                if (Wn != 0 && Fn != 1.f)
                  if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                  {
                    float3 p;
                    p.x = V.x;
                    p.z = V.z;

                    float Vny = V.y + cell_size.y;

                    float d_inv = 1.f / (std::abs (F) + std::abs (Fn));
                    p.y = (V.y * std::abs (Fn) + Vny * std::abs (F)) * d_inv;

                    points[local_count++] = p;
                  }
              }                /*  if (y + 1 < VOLUME_Y) */

              //process dz
              //if (z + 1 < VOLUME_Z) // guaranteed by loop
              {
                int Wn;
                float Fn = fetch (x, y, z + 1, Wn);

                if (Wn != 0 && Fn != 1.f)
                  if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                  {
                    float3 p;
                    p.x = V.x;
                    p.y = V.y;

                    float Vnz = V.z + cell_size.z;

                    float d_inv = 1.f / (std::abs (F) + std::abs (Fn));
                    p.z = (V.z * std::abs (Fn) + Vnz * std::abs (F)) * d_inv;

                    points[local_count++] = p;
                  }
              }               /* if (z + 1 < VOLUME_Z) */
            }              /* if (W != 0 && F != 1.f) */
          }            /* if (x < VOLUME_X && y < VOLUME_Y) */

#if CUDART_VERSION >= 9000
          int total_warp = __popc (__ballot_sync (__activemask (), local_count > 0))
                         + __popc (__ballot_sync (__activemask (), local_count > 1))
                         + __popc (__ballot_sync (__activemask (), local_count > 2));
#else
          ///not we fulfilled points array at current iteration
          int total_warp = __popc (__ballot (local_count > 0)) + __popc (__ballot (local_count > 1)) + __popc (__ballot (local_count > 2));
#endif

          if (total_warp > 0)
          {
            int lane = Warp::laneId ();
            int storage_index = (ftid >> Warp::LOG_WARP_SIZE) * Warp::WARP_SIZE * MAX_LOCAL_POINTS;

            volatile int* cta_buffer = (int*)(storage_X + storage_index);

            cta_buffer[lane] = local_count;
            int offset = scan_warp<exclusive>(cta_buffer, lane);

            if (lane == 0)
            {
              int old_global_count = atomicAdd (&global_count, total_warp);
              cta_buffer[0] = old_global_count;
            }
            int old_global_count = cta_buffer[0];

            for (int l = 0; l < local_count; ++l)
            {
              storage_X[storage_index + offset + l] = points[l].x;
              storage_Y[storage_index + offset + l] = points[l].y;
              storage_Z[storage_index + offset + l] = points[l].z;
            }

            PointType *pos = output.data + old_global_count + lane;
            for (int idx = lane; idx < total_warp; idx += Warp::STRIDE, pos += Warp::STRIDE)
            {
              float x = storage_X[storage_index + idx];
              float y = storage_Y[storage_index + idx];
              float z = storage_Z[storage_index + idx];
              store_point_type (x, y, z, pos);
            }

            bool full = (old_global_count + total_warp) >= output.size;

            if (full)
              break;
          }

        }         /* for(int z = 0; z < VOLUME_Z - 1; ++z) */


        ///////////////////////////
        // prepare for future scans
        if (ftid == 0)
        {
          unsigned int total_blocks = gridDim.x * gridDim.y * gridDim.z;
          unsigned int value = atomicInc (&blocks_done, total_blocks);

          //last block
          if (value == total_blocks - 1)
          {
            output_count = min ((int)output.size, global_count);
            blocks_done = 0;
            global_count = 0;
          }
        }
      }       /* operator() */

      __device__ __forceinline__ void
      store_point_type (float x, float y, float z, float4* ptr) const {
        *ptr = make_float4 (x, y, z, 0);
      }
      __device__ __forceinline__ void
      store_point_type (float x, float y, float z, float3* ptr) const {
        *ptr = make_float3 (x, y, z);
      }
    };

    __global__ void
    extractKernel (const FullScan6 fs) {
      fs ();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t
pcl::device::extractCloud (const PtrStep<short2>& volume, const float3& volume_size, 
                           PtrSz<PointType> output)
{
  FullScan6 fs;
  fs.volume = volume;
  fs.cell_size.x = volume_size.x / VOLUME_X;
  fs.cell_size.y = volume_size.y / VOLUME_Y;
  fs.cell_size.z = volume_size.z / VOLUME_Z;
  fs.output = output;

  dim3 block (CTA_SIZE_X, CTA_SIZE_Y);
  dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

  //cudaFuncSetCacheConfig(extractKernel, cudaFuncCachePreferL1);
  //printFuncAttrib(extractKernel);

  extractKernel<<<grid, block>>>(fs);
  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());

  int size;
  cudaSafeCall ( cudaMemcpyFromSymbol (&size, output_count, sizeof(size)) );
  return (std::size_t)size;
}


namespace pcl
{
  namespace device
  {
    template<typename NormalType>
    struct ExtractNormals
    {
      float3 cell_size;
      PtrStep<short2> volume;
      PtrSz<PointType> points;

      mutable NormalType* output;

      __device__ __forceinline__ float
      readTsdf (int x, int y, int z) const
      {
        return unpack_tsdf (volume.ptr (VOLUME_Y * z + y)[x]);
      }

      __device__ __forceinline__ float3
      fetchPoint (int idx) const
      {
        PointType p = points.data[idx];
        return make_float3 (p.x, p.y, p.z);
      }
      __device__ __forceinline__ void
      storeNormal (int idx, float3 normal) const
      {
        NormalType n;
        n.x = normal.x; n.y = normal.y; n.z = normal.z;
        output[idx] = n;
      }

      __device__ __forceinline__ int3
      getVoxel (const float3& point) const
      {
        int vx = __float2int_rd (point.x / cell_size.x);        // round to negative infinity
        int vy = __float2int_rd (point.y / cell_size.y);
        int vz = __float2int_rd (point.z / cell_size.z);

        return make_int3 (vx, vy, vz);
      }

      __device__ __forceinline__ void
      operator () () const
      {
        int idx = threadIdx.x + blockIdx.x * blockDim.x;

        if (idx >= points.size)
          return;
        constexpr float qnan = std::numeric_limits<float>::quiet_NaN ();
        float3 n = make_float3 (qnan, qnan, qnan);

        float3 point = fetchPoint (idx);
        int3 g = getVoxel (point);

        if (g.x > 1 && g.y > 1 && g.z > 1 && g.x < VOLUME_X - 2 && g.y < VOLUME_Y - 2 && g.z < VOLUME_Z - 2)
        {
          float3 t;

          t = point;
          t.x += cell_size.x;
          float Fx1 = interpolateTrilineary (t);

          t = point;
          t.x -= cell_size.x;
          float Fx2 = interpolateTrilineary (t);

          n.x = (Fx1 - Fx2);

          t = point;
          t.y += cell_size.y;
          float Fy1 = interpolateTrilineary (t);

          t = point;
          t.y -= cell_size.y;
          float Fy2 = interpolateTrilineary (t);

          n.y = (Fy1 - Fy2);

          t = point;
          t.z += cell_size.z;
          float Fz1 = interpolateTrilineary (t);

          t = point;
          t.z -= cell_size.z;
          float Fz2 = interpolateTrilineary (t);

          n.z = (Fz1 - Fz2);

          n = normalized (n);
        }
        storeNormal (idx, n);
      }

      __device__ __forceinline__ float
      interpolateTrilineary (const float3& point) const
      {
        int3 g = getVoxel (point);

        float vx = (g.x + 0.5f) * cell_size.x;
        float vy = (g.y + 0.5f) * cell_size.y;
        float vz = (g.z + 0.5f) * cell_size.z;

        g.x = (point.x < vx) ? (g.x - 1) : g.x;
        g.y = (point.y < vy) ? (g.y - 1) : g.y;
        g.z = (point.z < vz) ? (g.z - 1) : g.z;

        float a = (point.x - (g.x + 0.5f) * cell_size.x) / cell_size.x;
        float b = (point.y - (g.y + 0.5f) * cell_size.y) / cell_size.y;
        float c = (point.z - (g.z + 0.5f) * cell_size.z) / cell_size.z;

        float res = readTsdf (g.x + 0, g.y + 0, g.z + 0) * (1 - a) * (1 - b) * (1 - c) +
                    readTsdf (g.x + 0, g.y + 0, g.z + 1) * (1 - a) * (1 - b) * c +
                    readTsdf (g.x + 0, g.y + 1, g.z + 0) * (1 - a) * b * (1 - c) +
                    readTsdf (g.x + 0, g.y + 1, g.z + 1) * (1 - a) * b * c +
                    readTsdf (g.x + 1, g.y + 0, g.z + 0) * a * (1 - b) * (1 - c) +
                    readTsdf (g.x + 1, g.y + 0, g.z + 1) * a * (1 - b) * c +
                    readTsdf (g.x + 1, g.y + 1, g.z + 0) * a * b * (1 - c) +
                    readTsdf (g.x + 1, g.y + 1, g.z + 1) * a * b * c;
        return res;
      }
    };

    template<typename NormalType>
    __global__ void
    extractNormalsKernel (const ExtractNormals<NormalType> en) {
      en ();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename NormalType> void
pcl::device::extractNormals (const PtrStep<short2>& volume, const float3& volume_size, 
                             const PtrSz<PointType>& points, NormalType* output)
{
  ExtractNormals<NormalType> en;
  en.volume = volume;
  en.cell_size.x = volume_size.x / VOLUME_X;
  en.cell_size.y = volume_size.y / VOLUME_Y;
  en.cell_size.z = volume_size.z / VOLUME_Z;
  en.points = points;
  en.output = output;

  dim3 block (256);
  dim3 grid (divUp (points.size, block.x));

  extractNormalsKernel<<<grid, block>>>(en);
  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());
}

using namespace pcl::device;

template void pcl::device::extractNormals<PointType>(const PtrStep<short2>&volume, const float3 &volume_size, const PtrSz<PointType>&input, PointType * output);
template void pcl::device::extractNormals<float8>(const PtrStep<short2>&volume, const float3 &volume_size, const PtrSz<PointType>&input, float8 * output);

