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


#include <thrust/copy.h>
#include <thrust/device_ptr.h>
#include <thrust/iterator/counting_iterator.h>

#include "internal.hpp"

#include "cuda.h"

using namespace thrust;

namespace pcl
{    
    namespace device
    {
        struct InSphere
        {    
            float x_, y_, z_, radius2_;
            InSphere(float x, float y, float z, float radius) : x_(x), y_(y), z_(z), radius2_(radius * radius) {}

            __device__ __host__ __forceinline__ bool operator()(const float3& point) const
            {
                float dx = point.x - x_;
                float dy = point.y - y_;
                float dz = point.z - z_;

                return (dx * dx + dy * dy + dz * dz) < radius2_;
            }

            __device__ __host__ __forceinline__ bool operator()(const float4& point) const
            {
                return (*this)(make_float3(point.x, point.y, point.z));                
            }
        };
    }
}

void pcl::device::bruteForceRadiusSearch(const OctreeImpl::PointCloud& cloud, const OctreeImpl::PointType& query, float radius, DeviceArray<int>& result, DeviceArray<int>& buffer)
{   
    using PointType = OctreeImpl::PointType;

    if (buffer.size() < cloud.size())
        buffer.create(cloud.size());

    InSphere cond(query.x, query.y, query.z, radius);

    device_ptr<const PointType> cloud_ptr((const PointType*)cloud.ptr());
    device_ptr<int> res_ptr(buffer.ptr());
    
    counting_iterator<int> first(0);
    counting_iterator<int> last = first + cloud.size();
    
    //main bottle neck is a kernel call overhead/allocs
    //work time for 871k points ~0.8ms
    int count = (int)(thrust::copy_if(first, last, cloud_ptr, res_ptr, cond) - res_ptr);
    result = DeviceArray<int>(buffer.ptr(), count);
}
