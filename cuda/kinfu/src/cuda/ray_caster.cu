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
#include "device.hpp"

namespace pcl
{
    namespace device
    {	
        __device__ __forceinline__ float getMinTime(const float3& volume_min, const float3& volume_max, const float3& origin, const float3& dir)
        {						
            float txmin = ( (dir.x > 0 ? volume_min.x : volume_max.x) - origin.x) / dir.x;
            float tymin = ( (dir.y > 0 ? volume_min.y : volume_max.y) - origin.y) / dir.y;
            float tzmin = ( (dir.z > 0 ? volume_min.z : volume_max.z) - origin.z) / dir.z;

            return fmax( fmax(txmin, tymin), tzmin);
        }

        __device__ __forceinline__ float getMaxTime(const float3& volume_min, const float3& volume_max, const float3& origin, const float3& dir)
        {						
            float txmax = ( (dir.x > 0 ? volume_max.x : volume_min.x) - origin.x) / dir.x;
            float tymax = ( (dir.y > 0 ? volume_max.y : volume_min.y) - origin.y) / dir.y;
            float tzmax = ( (dir.z > 0 ? volume_max.z : volume_min.z) - origin.z) / dir.z;

            return fmin(fmin(txmax, tymax), tzmax);			
        }

        __device__ __forceinline__ float unpack(float value)
        {	
            float tmp = fabsf(value);
            int weight = __float2int_rz(tmp/2);
            tmp = tmp - weight * 2;
            return value < 0 ? -tmp : tmp;
        }

        struct RayCaster
        {
            enum
            {
                CTA_SIZE_X = 32,  CTA_SIZE_Y = 8
            };

            float fx_inv, fy_inv, cx, cy;
            Mat33  Rcurr;
            float3 tcurr;

            const float3 volume_min; // !!! must be set to {0, 0, 0}
            float3 volume_max;						

            float3 cell_size;
            int cols, rows;

            PtrStep<short2> volume;

            mutable PtrStep<float> nmap;
            mutable PtrStep<float> vmap;

            RayCaster() : volume_min(make_float3(0.f, 0.f, 0.f)) {}

            __device__ __forceinline__ float3 get_ray_start(int x, int y) const
            {
                return make_float3(0.f, 0.f, 0.f);
            }
            __device__ __forceinline__ float3 get_ray_next(int x, int y) const
            {
                float3 ray_next;
                ray_next.x = (x - cx) * fx_inv;
                ray_next.y = (y - cy) * fy_inv;
                ray_next.z = 1;
                return ray_next;		
            }

            __device__ __forceinline__ float3 cvtToGlobalCoo(const float3& v) const
            {
                return Rcurr * v + tcurr;
            }

            __device__ __forceinline__ int3 truncateInds(const int3& g) const 
            {
                int x = max(0, min(g.x, VOLUME_X-1));
                int y = max(0, min(g.y, VOLUME_Y-1));
                int z = max(0, min(g.z, VOLUME_Z-1));
                return make_int3(x, y, z);
            }

            __device__ __forceinline__ bool checkInds(const int3& g) const
            {
                return (g.x >= 0 && g.y >=0 && g.z >= 0 && g.x < VOLUME_X && g.y < VOLUME_Y && g.z < VOLUME_X);
            }

            __device__ __forceinline__ float readTsdf(int x, int y, int z) const
            {
                return unpack(volume.ptr(VOLUME_Y * z + y)[x]);
            }

            __device__ __forceinline__ int3 getVoxelFromTime(const float3& origin, const float3& dir, float time) const
            {
                float3 intersect = origin + dir * time;				
                int vx = __float2int_rz(intersect.x / cell_size.x);
                int vy = __float2int_rz(intersect.y / cell_size.y);
                int vz = __float2int_rz(intersect.z / cell_size.z);

                return make_int3(vx, vy, vz);				
            }	

            __device__ __forceinline__ int3 computeNextVoxel(const float3& origin, const float3& dir, const int3& voxel) const
            {
                float exit_time = computeVoxelExitTime(origin, dir, voxel);
                float xmax = fabs( voxel.x * cell_size.x + (dir.x > 0 ? cell_size.x : 0.f) - origin.x - exit_time * dir.x);
                float ymax = fabs( voxel.y * cell_size.y + (dir.y > 0 ? cell_size.y : 0.f) - origin.y - exit_time * dir.y);
                float zmax = fabs( voxel.z * cell_size.z + (dir.z > 0 ? cell_size.z : 0.f) - origin.z - exit_time * dir.z);

                float min_facet_dist = fmin( fmin(xmax, ymax), zmax);

                int3 res;
                res.x = xmax == min_facet_dist ? voxel.x + (dir.x > 0 ? 1 : - 1) : voxel.x;
                res.y = ymax == min_facet_dist ? voxel.y + (dir.y > 0 ? 1 : - 1) : voxel.y;
                res.z = zmax == min_facet_dist ? voxel.z + (dir.z > 0 ? 1 : - 1) : voxel.z;                
                return res;
            }

            __device__ __forceinline__ float computeVoxelExitTime(const float3& origin, const float3& dir, const int3& voxel) const
            {	            
                float txmax = ( voxel.x * cell_size.x + (dir.x > 0 ? cell_size.x : 0.f) - origin.x) / dir.x;
                float tymax = ( voxel.y * cell_size.y + (dir.y > 0 ? cell_size.y : 0.f) - origin.y) / dir.y;
                float tzmax = ( voxel.z * cell_size.z + (dir.z > 0 ? cell_size.z : 0.f) - origin.z) / dir.z;

                return fmin( fmin(txmax, tymax), tzmax);
            }

            __device__ __forceinline__ float computeVoxelEntryTime(const float3& origin, const float3& dir, const int3& voxel) const
            {	            
                float txmin = ( voxel.x * cell_size.x + (dir.x > 0 ? 0.f : cell_size.x) - origin.x) / dir.x;
                float tymin = ( voxel.y * cell_size.y + (dir.y > 0 ? 0.f : cell_size.y) - origin.y) / dir.y;
                float tzmin = ( voxel.z * cell_size.z + (dir.z > 0 ? 0.f : cell_size.z) - origin.z) / dir.z;

                return fmax( fmax(txmin, tymin), tzmin);
            }

            __device__ __forceinline__ void operator()() const
            {
                const float step = fmin(cell_size.x, fmin(cell_size.y, cell_size.z)) / 333;
                const float min_dist = 5.f; //in mm

                int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
                int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

                if (x >= cols || y >= rows)
                    return;

                vmap.ptr(y)[x] = numeric_limits<float>::quiet_NaN();
                nmap.ptr(y)[x] = numeric_limits<float>::quiet_NaN();

                float3 ray_start = cvtToGlobalCoo(get_ray_start(x, y)); // 2
                float3 ray_next  = cvtToGlobalCoo(get_ray_next (x, y)); // 3		

                float3 ray_dir = normalized(ray_next - ray_start); //4

                //ensure that it isn't a degenerate case				
                ray_dir.x  = (ray_dir.x == 0.f) ? 1e-15 : ray_dir.x;
                ray_dir.y  = (ray_dir.y == 0.f) ? 1e-15 : ray_dir.y;
                ray_dir.z  = (ray_dir.z == 0.f) ? 1e-15 : ray_dir.z;								

                // computer time when entry and exit volume
                float time_start_volume = getMinTime(volume_min, volume_max, ray_start, ray_dir);
                float time_exit_volume  = getMaxTime(volume_min, volume_max, ray_start, ray_dir);								

                time_start_volume = fmax(time_start_volume, min_dist);
                if (time_start_volume >= time_exit_volume)
                    return;

                int3 g = truncateInds(getVoxelFromTime(ray_start, ray_dir, time_start_volume)); //6						
                float tsdf = readTsdf(g.x, g.y, g.z);

                //ray_start = ray_start + ray_dir * time_start_volume;
                
                for(;;) // while voxel g withing volume bounds
                {			                    
                    int3  g_prev = g;
                    float tsdf_prev = tsdf;

                    g = computeNextVoxel(ray_start, ray_dir, g_prev);
                    if(!checkInds(g))
                        break;                   

                    tsdf = readTsdf(g.x, g.y, g.z);

                    if (tsdf_prev > 0.f && tsdf < 0.f) //13 zero crossing
                    {												
                        //my guess how to do                        
                        float time_start = computeVoxelEntryTime(ray_start, ray_dir, g_prev);
                        float time_exit = computeVoxelExitTime(ray_start, ray_dir, g);

                        float3 point1 = ray_start + ray_dir * time_start;
                        float3 point2 = ray_start + ray_dir * time_exit;

                        float3 v = (point2 * tsdf_prev + point1 * fabs(tsdf)) * (1.f/(tsdf_prev + fabs(tsdf)));
                        
                        vmap.ptr(y       )[x] = v.x;
                        vmap.ptr(y+  rows)[x] = v.y;
                        vmap.ptr(y+2*rows)[x] = v.z;	

                        //printf("(%d,%d) %d %d %d - %f %f - - - %d %d %d\n",x, y, g.x, g.y, g.z, tsdf_prev, tsdf, g_prev.x, g_prev.y, g_prev.z);

                        if (g.x != 0 && g.y != 0 && g.z != 0 && g.x != VOLUME_X - 1 && g.y != VOLUME_Y - 1 && g.z != VOLUME_Z - 1)
                        {
                            //extract grad(tsdf_volume)
                            float3 normal;
                            normal.x = readTsdf(g.x+1, g.y, g.z) - readTsdf(g.x-1, g.y, g.z);
                            normal.y = readTsdf(g.x, g.y+1, g.z) - readTsdf(g.x, g.y-1, g.z);
                            normal.z = readTsdf(g.x, g.y, g.z+1) - readTsdf(g.x, g.y, g.z-1);								

                            normal = normalized(normal);

                            nmap.ptr(y       )[x] = normal.x;
                            nmap.ptr(y+  rows)[x] = normal.y;
                            nmap.ptr(y+2*rows)[x] = normal.z;
                        }

                        break;
                    }
                }
            }
        };

        __global__ void rayCastKernel(const RayCaster rc) { rc(); }
    }
}



void pcl::device::raycast(const Mat33& Rcurr, const float3& tcurr, const Intr& intr, const float3& volume_size, 
                            const PtrStep<short2>& volume, MapArr vmap, MapArr nmap)
{
    RayCaster rc;

    rc.fx_inv = 1.f/intr.fx;  rc.cx = intr.cx;
    rc.fy_inv = 1.f/intr.fy;  rc.cy = intr.cy;		

    rc.Rcurr = Rcurr;
    rc.tcurr = tcurr;
    rc.volume_max = volume_size;		

    rc.cell_size.x = volume_size.x / VOLUME_X;
    rc.cell_size.y = volume_size.y / VOLUME_Y;
    rc.cell_size.z = volume_size.z / VOLUME_Z;

    rc.cols = vmap.cols();
    rc.rows = vmap.rows()/3;

    rc.volume = volume;		
    rc.nmap = nmap;
    rc.vmap = vmap;

    dim3 block(RayCaster::CTA_SIZE_X, RayCaster::CTA_SIZE_Y);
    dim3 grid(divUp(rc.cols, block.x), divUp(rc.rows, block.y));

    rayCastKernel<<<grid, block>>>(rc);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall(cudaDeviceSynchronize());
}

