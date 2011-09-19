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

#ifndef _PCL_GPU_OCTREE_BOXUTILS_HPP_
#define _PCL_GPU_OCTREE_BOXUTILS_HPP_

#include "utils/morton.hpp"

namespace pcl
{
    namespace device
    {
        __device__ __host__ __forceinline__
        static bool checkIfNodeInsideSphere(const float3& minp, const float3& maxp, const float3& c, float r)
        {
            r *= r;

            float d2_xmin = (minp.x - c.x) * (minp.x - c.x);
            float d2_ymin = (minp.y - c.y) * (minp.y - c.y);
            float d2_zmin = (minp.z - c.z) * (minp.z - c.z);

            if (d2_xmin + d2_ymin + d2_zmin > r)
                return false;

            float d2_zmax = (maxp.z - c.z) * (maxp.z - c.z);

            if (d2_xmin + d2_ymin + d2_zmax > r)
                return false;

            float d2_ymax = (maxp.y - c.y) * (maxp.y - c.y);

            if (d2_xmin + d2_ymax + d2_zmin > r)
                return false;

            if (d2_xmin + d2_ymax + d2_zmax > r)
                return false;

            float d2_xmax = (maxp.x - c.x) * (maxp.x - c.x);

            if (d2_xmax + d2_ymin + d2_zmin > r)
                return false;

            if (d2_xmax + d2_ymin + d2_zmax > r)
                return false;

            if (d2_xmax + d2_ymax + d2_zmin > r)
                return false;

            if (d2_xmax + d2_ymax + d2_zmax > r)
                return false;

            return true;
        }

        __device__ __host__ __forceinline__
        static bool checkIfNodeOutsideSphere(const float3& minp, const float3& maxp, const float3& c, float r)
        {
            if (maxp.x < (c.x - r) ||  maxp.y < (c.y - r) || maxp.z < (c.z - r))
                return true;

            if ((c.x + r) < minp.x || (c.y + r) < minp.y || (c.z + r) < minp.z)
                return true;

            return false;
        }

        __device__ __host__ __forceinline__
        static void calcBoundingBox(int level, int code, float3& res_minp, float3& res_maxp)
        {        
            int cell_x, cell_y, cell_z;
            Morton::decomposeCode(code, cell_x, cell_y, cell_z);   

            float cell_size_x = (res_maxp.x - res_minp.x) / (1 << level);
            float cell_size_y = (res_maxp.y - res_minp.y) / (1 << level);
            float cell_size_z = (res_maxp.z - res_minp.z) / (1 << level);

            res_minp.x += cell_x * cell_size_x;
            res_minp.y += cell_y * cell_size_y;
            res_minp.z += cell_z * cell_size_z;

            res_maxp.x = res_minp.x + cell_size_x;
            res_maxp.y = res_minp.y + cell_size_y;
            res_maxp.z = res_minp.z + cell_size_z;       
        }
    }
}

#endif /* _PCL_GPU_OCTREE_BOXUTILS_HPP_ */
