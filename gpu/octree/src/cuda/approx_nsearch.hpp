/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#ifndef PCL_GPU_APPROX_NSEARCH
#define PCL_GPU_APPROX_NSEARCH

namespace pcl
{
    namespace device
    {
        namespace appnearest_search
        {
            std::pair<uint3, std::uint8_t> nearestVoxel(const float3 query, const unsigned& level, const std::uint8_t& mask, const float3& minp, const float3& maxp, const uint3& index);
        }
    }
}

#endif /* PCL_GPU_APPROX_NSEARCH */
