/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#pragma once

#include "internal.hpp"
#include "cuda_runtime.h"
#include <pcl/gpu/utils/device/static_check.hpp>
#include <pcl/exceptions.h>

#include<cassert>

//////////////////////////////////////////////////////////////////////////////////////
//////////////// Octree Host Interface implementation ////////////////////////////////

template <class T>
pcl::gpu::details::Octree<T>::Octree() : cloud_(nullptr), impl(nullptr)
{
    Static<sizeof(PointType) == sizeof(typename pcl::device::OctreeImpl<T>::PointType)>::check();

    int device;
    cudaSafeCall( cudaGetDevice( &device ) );
    
    cudaDeviceProp prop;
    cudaSafeCall( cudaGetDeviceProperties( &prop, device) );

    if (prop.major < 2)
        pcl::gpu::error("This code requires devices with compute capability >= 2.0", __FILE__, __LINE__);

    int bin, ptx;
    pcl::device::OctreeImpl<T>::get_gpu_arch_compiled_for(bin, ptx);
    if (bin < 0 || ptx < 0)
    {
        pcl::gpu::error(R"(cudaFuncGetAttributes() returned a value < 0.
This is likely a build configuration error.
Ensure that the proper compute capability is specified in the CUDA_ARCH_BIN cmake variable when building for your GPU.)",
            __FILE__, __LINE__);
    }

    if (bin < 20 && ptx < 20)
        pcl::gpu::error("This must be compiled for compute capability >= 2.0", __FILE__, __LINE__);    

    impl = new pcl::device::OctreeImpl<T>();
    built_ = false;
}

template <typename T>
pcl::gpu::details::Octree<T>::~Octree() { clear(); }

template <typename T>
void pcl::gpu::details::Octree<T>::clear()
{
        delete static_cast<pcl::device::OctreeImpl<T>*>(impl);
}

template <typename T>
void pcl::gpu::details::Octree<T>::setCloud(const PointCloud& cloud_arg)
{    
    const typename pcl::device::OctreeImpl<T>::PointCloud& cloud = (const typename pcl::device::OctreeImpl<T>::PointCloud&)cloud_arg;
    cloud_ =  &cloud_arg;
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->setCloud(cloud);
}

template <typename T>
void pcl::gpu::details::Octree<T>::build()
{
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->build();
    built_ = true;
}

template <typename T>
bool pcl::gpu::details::Octree<T>::isBuilt() const
{
    return built_;
}

template <typename T>
void pcl::gpu::details::Octree<T>::internalDownload()
{
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->internalDownload();
}

template <typename T>
void pcl::gpu::details::Octree<T>::radiusSearchHost(const PointType& center, float radius, std::vector<int>& out, int max_nn)
{
    if (!static_cast<pcl::device::OctreeImpl<T>*>(impl)->host_octree.downloaded)
        internalDownload();

    typename pcl::device::OctreeImpl<T>::PointType query;
    query.p.x = center.x;
    query.p.y = center.y;
    query.p.z = center.z;
    
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->radiusSearchHost(query, radius, out, max_nn);
}

template <typename T>
void  pcl::gpu::details::Octree<T>::approxNearestSearchHost(const PointType& query, int& out_index, float& sqr_dist)
{
    if (!static_cast<pcl::device::OctreeImpl<T>*>(impl)->host_octree.downloaded)
        internalDownload();

    typename pcl::device::OctreeImpl<T>::PointType q;
    q.p.x = query.x;
    q.p.y = query.y;
    q.p.z = query.z;
    
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->approxNearestSearchHost(q, out_index, sqr_dist);

}
                        
template <typename T>
void pcl::gpu::details::Octree<T>::radiusSearch(const Queries& queries, float radius, int max_results, NeighborIndices& results) const
{
    assert(queries.size() > 0);
    results.create(static_cast<int> (queries.size()), max_results);
    results.sizes.create(queries.size());
    
    const typename pcl::device::OctreeImpl<T>::Queries& q = (const typename pcl::device::OctreeImpl<T>::Queries&)queries;
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->radiusSearch(q, radius, results);
}

template <typename T>
void pcl::gpu::details::Octree<T>::radiusSearch(const Queries& queries, const Radiuses& radiuses, int max_results, NeighborIndices& results) const
{
    assert(queries.size() > 0);
    assert(queries.size() == radiuses.size());
    results.create(static_cast<int> (queries.size()), max_results);
    results.sizes.create(queries.size());
    
    const typename pcl::device::OctreeImpl<T>::Queries& q = (const typename pcl::device::OctreeImpl<T>::Queries&)queries;
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->radiusSearch(q, radiuses, results);
}

template <typename T>
void pcl::gpu::details::Octree<T>::radiusSearch(const Queries& queries, const Indices& indices, float radius, int max_results, NeighborIndices& results) const
{
    assert(queries.size() > 0 && indices.size() > 0);
    results.create(static_cast<int> (indices.size()), max_results);
    results.sizes.create(indices.size());
    
    const typename pcl::device::OctreeImpl<T>::Queries& q = (const typename pcl::device::OctreeImpl<T>::Queries&)queries;
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->radiusSearch(q, indices, radius, results);
}

template <typename T>
void pcl::gpu::details::Octree<T>::approxNearestSearch(const Queries& queries, NeighborIndices& results) const
{
    ResultSqrDists sqr_distance;
    approxNearestSearch(queries, results, sqr_distance);
}

template <typename T>
void pcl::gpu::details::Octree<T>::approxNearestSearch(const Queries& queries, NeighborIndices& results, ResultSqrDists& sqr_distance) const
{
    assert(queries.size() > 0);    
    results.create(static_cast<int> (queries.size()), 1);
    sqr_distance.create(queries.size());
    
    const typename pcl::device::OctreeImpl<T>::Queries& q = (const typename pcl::device::OctreeImpl<T>::Queries&)queries;
    static_cast<pcl::device::OctreeImpl<T>*>(impl)->approxNearestSearch(q, results, sqr_distance);
}

template <typename T>
void pcl::gpu::details::Octree<T>::nearestKSearchBatch(const Queries& queries, int k, NeighborIndices& results) const
{
    ResultSqrDists sqr_distances;
    nearestKSearchBatch(queries, k, results, sqr_distances);
}

template <typename T>
void pcl::gpu::details::Octree<T>::nearestKSearchBatch(const Queries& queries, int k, NeighborIndices& results, ResultSqrDists& sqr_distances) const
{    
    if (k != 1)
        throw pcl::PCLException("OctreeGPU::knnSearch is supported only for k == 1", __FILE__, "", __LINE__);
    
    assert(queries.size() > 0);
    results.create(static_cast<int> (queries.size()), k);	    
    sqr_distances.create(queries.size() * k);

	const typename pcl::device::OctreeImpl<T>::Queries& q = (const typename pcl::device::OctreeImpl<T>::Queries&)queries;

    static_cast<pcl::device::OctreeImpl<T>*>(impl)->nearestKSearchBatch(q, k, results, sqr_distances);
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////// Brute Force Radius Search Mediator //////////////////////////////////

template <typename T>
void pcl::gpu::details::bruteForceRadiusSearchGPU(const typename Octree<T>::PointCloud& cloud, const typename Octree<T>::PointType& query,  float radius,  DeviceArray<int>& result,  DeviceArray<int>& buffer)
{
    using PointType = typename pcl::device::OctreeImpl<T>::PointType;
    using PointCloud = typename pcl::device::OctreeImpl<T>::PointCloud;
    
    PointType query_local;
    query_local.p.x = query.x;
    query_local.p.y = query.y;
    query_local.p.z = query.z;

    Static<sizeof(PointType) == sizeof(typename pcl::device::OctreeImpl<T>::PointType)>::check();

    PointCloud cloud_local((PointType*)cloud.ptr(), cloud.size());
    pcl::device::bruteForceRadiusSearch<T>(cloud_local, query_local, radius, result, buffer);
}
