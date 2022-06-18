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

#include "internal.hpp"

#include <thrust/device_ptr.h>
#include <thrust/transform_reduce.h>
#include <thrust/iterator/permutation_iterator.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/zip_iterator.h>

#include "pcl/gpu/utils/device/vector_math.hpp"

namespace pcl
{
    namespace device
    {
        template<class PointT>
        struct PointT2float3
        {
            __device__ __forceinline__ float3 operator()(const PointT& p) const { return make_float3(p.x, p.y, p.z); }
        };

        struct PlusFloat3
        {
            __device__ __forceinline__ float3 operator()(const float3& e1, const float3& e2) const 
            { 
                return make_float3(e1.x + e2.x, e1.y + e2.y, e1.z + e2.z); 
            }
        };
        
        struct TupleDistCvt
        {
            float3 pivot_;
            TupleDistCvt(const float3& pivot) : pivot_(pivot) {}
            __device__ __forceinline__ thrust::tuple<float, int> operator()(const thrust::tuple<float4, int>& t) const 
            { 
                float4 point = t.get<0>();

                float dx = pivot_.x - point.x;
                float dy = pivot_.y - point.y;
                float dz = pivot_.z - point.z;
                float dist = sqrt(dx*dx + dy*dy + dz*dz);

                return thrust::tuple<float, int>(dist, t.get<1>());                                
            }
        };

    }
}

template<typename PointT>
void pcl::device::compute3DCentroid(const DeviceArray<PointT>& cloud, float3& centroid)
{
    thrust::device_ptr<PointT> src_beg((PointT*)cloud.ptr());
    thrust::device_ptr<PointT> src_end = src_beg + cloud.size();
                
    centroid = transform_reduce(src_beg, src_beg, PointT2float3<PointT>(), make_float3(0.f, 0.f, 0.f), PlusFloat3());
    centroid *= 1.f/cloud.size();
}

template<typename PointT>
void pcl::device::compute3DCentroid(const DeviceArray<PointT>& cloud, const Indices& indices, float3& centroid)
{
    if (indices.empty())
        compute3DCentroid(cloud, centroid);
    else
    {
        thrust::device_ptr<PointT> src_beg((PointT*)cloud.ptr());    
        thrust::device_ptr<int> map_beg((int*)indices.ptr());
        thrust::device_ptr<int> map_end = map_beg + indices.size();


        centroid = transform_reduce(make_permutation_iterator(src_beg, map_beg),
            make_permutation_iterator(src_beg, map_end), 
            PointT2float3<PointT>(), make_float3(0.f, 0.f, 0.f), PlusFloat3());

        centroid *= 1.f/indices.size();
    }
}

template<typename PointT>
float3 pcl::device::getMaxDistance(const DeviceArray<PointT>& cloud, const float3& pivot)
{   
    thrust::device_ptr<PointT> src_beg((PointT*)cloud.ptr());
    thrust::device_ptr<PointT> src_end = src_beg + cloud.size();

    thrust::counting_iterator<int> cf(0) ;
    thrust::counting_iterator<int> ce = cf + cloud.size();

    thrust::tuple<float, int> init(0.f, 0);
    thrust::maximum<thrust::tuple<float, int>> op;
    
    thrust::tuple<float, int> res =
        transform_reduce(
        make_zip_iterator(make_tuple( src_beg, cf )),
        make_zip_iterator(make_tuple( src_beg, ce )),
        TupleDistCvt(pivot), init, op);

    float4 point = src_beg[res.get<1>()];

    return make_float3(point.x, point.y, point.z);
}


template<typename PointT>
float3 pcl::device::getMaxDistance(const DeviceArray<PointT>& cloud, const Indices& indices, const float3& pivot)
{   
    if (indices.empty())
        return getMaxDistance(cloud, pivot);

    thrust::device_ptr<PointT> src_beg((PointT*)cloud.ptr());    
    thrust::device_ptr<int> map_beg((int*)indices.ptr());
    thrust::device_ptr<int> map_end = map_beg + indices.size();

    thrust::counting_iterator<int> cf(0) ;
    thrust::counting_iterator<int> ce = cf + indices.size();

    thrust::tuple<float, int> init(0.f, 0);
    thrust::maximum<thrust::tuple<float, int>> op;
    
    thrust::tuple<float, int> res = transform_reduce(
        make_zip_iterator(make_tuple( make_permutation_iterator(src_beg, map_beg), cf )),
        make_zip_iterator(make_tuple( make_permutation_iterator(src_beg, map_end), ce )),
        TupleDistCvt(pivot), init, op);

    float4 point = src_beg[map_beg[res.get<1>()]];

    return make_float3(point.x, point.y, point.z);
}

template void pcl::device::compute3DCentroid<float4>(const DeviceArray<float4>& cloud, float3& centroid);
template void pcl::device::compute3DCentroid<float4>(const DeviceArray<float4>& cloud, const Indices& indices, float3& centroid);

template float3 pcl::device::getMaxDistance<float4>(const DeviceArray<float4>& cloud, const float3& pivot);
template float3 pcl::device::getMaxDistance<float4>(const DeviceArray<float4>& cloud, const Indices& indices, const float3& pivot);
