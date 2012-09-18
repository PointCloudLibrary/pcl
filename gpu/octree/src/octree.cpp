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

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/utils/timers_cuda.hpp>
#include <pcl/gpu/utils/safe_call.hpp>

#include "internal.hpp"
#include "cuda_runtime.h"
#include <pcl/gpu/utils/device/static_check.hpp>
#include <pcl/exceptions.h>

#include<assert.h>

using namespace pcl::device;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////
//////////////// Octree Host Interface implementation ////////////////////////////////

pcl::gpu::Octree::Octree() : cloud_(0), impl(0)
{
    Static<sizeof(PointType) == sizeof(OctreeImpl::PointType)>::check();

    int device;
    cudaSafeCall( cudaGetDevice( &device ) );
    
    cudaDeviceProp prop;
    cudaSafeCall( cudaGetDeviceProperties( &prop, device) );

    if (prop.major < 2)
        pcl::gpu::error("This code requires devices with compute capability >= 2.0", __FILE__, __LINE__);

    int bin, ptx;
    OctreeImpl::get_gpu_arch_compiled_for(bin, ptx);

    if (bin < 20 && ptx < 20)
        pcl::gpu::error("This must be compiled for compute capability >= 2.0", __FILE__, __LINE__);    

    impl = new OctreeImpl();        
    built_ = false;
}

pcl::gpu::Octree::~Octree() { clear(); }

void pcl::gpu::Octree::clear()
{
    if (impl)
        delete static_cast<OctreeImpl*>(impl);
}

void pcl::gpu::Octree::setCloud(const PointCloud& cloud_arg)
{    
    const OctreeImpl::PointCloud& cloud = (const OctreeImpl::PointCloud&)cloud_arg;
    cloud_ =  &cloud_arg;
    static_cast<OctreeImpl*>(impl)->setCloud(cloud);
}

void pcl::gpu::Octree::build()
{
    static_cast<OctreeImpl*>(impl)->build();    
    built_ = true;
}

bool pcl::gpu::Octree::isBuilt()
{
    return built_;
}

void pcl::gpu::Octree::internalDownload()
{
    static_cast<OctreeImpl*>(impl)->internalDownload();
}

void pcl::gpu::Octree::radiusSearchHost(const PointType& center, float radius, std::vector<int>& out, int max_nn)
{
    if (!static_cast<OctreeImpl*>(impl)->host_octree.downloaded)
        internalDownload();

    OctreeImpl::PointType query;
    query.x = center.x;
    query.y = center.y;
    query.z = center.z;
    
    static_cast<OctreeImpl*>(impl)->radiusSearchHost(query, radius, out, max_nn);
}

void  pcl::gpu::Octree::approxNearestSearchHost(const PointType& query, int& out_index, float& sqr_dist)
{
    if (!static_cast<OctreeImpl*>(impl)->host_octree.downloaded)
        internalDownload();

    OctreeImpl::PointType q;
    q.x = query.x;
    q.y = query.y;
    q.z = query.z;
    
    static_cast<OctreeImpl*>(impl)->approxNearestSearchHost(q, out_index, sqr_dist);

}
                        
void pcl::gpu::Octree::radiusSearch(const Queries& queries, float radius, int max_results, NeighborIndices& results) const
{
    assert(queries.size() > 0);
    results.create(static_cast<int> (queries.size()), max_results);
    results.sizes.create(queries.size());
    
    const OctreeImpl::Queries& q = (const OctreeImpl::Queries&)queries;
    static_cast<OctreeImpl*>(impl)->radiusSearch(q, radius, results);
}

void pcl::gpu::Octree::radiusSearch(const Queries& queries, const Radiuses& radiuses, int max_results, NeighborIndices& results) const
{
    assert(queries.size() > 0);
    assert(queries.size() == radiuses.size());
    results.create(static_cast<int> (queries.size()), max_results);
    results.sizes.create(queries.size());
    
    const OctreeImpl::Queries& q = (const OctreeImpl::Queries&)queries;
    static_cast<OctreeImpl*>(impl)->radiusSearch(q, radiuses, results);
}

void pcl::gpu::Octree::radiusSearch(const Queries& queries, const Indices& indices, float radius, int max_results, NeighborIndices& results) const
{
    assert(queries.size() > 0 && indices.size() > 0);
    results.create(static_cast<int> (indices.size()), max_results);
    results.sizes.create(indices.size());
    
    const OctreeImpl::Queries& q = (const OctreeImpl::Queries&)queries;
    static_cast<OctreeImpl*>(impl)->radiusSearch(q, indices, radius, results);
}

void pcl::gpu::Octree::approxNearestSearch(const Queries& queries, NeighborIndices& results) const
{
    assert(queries.size() > 0);    
    results.create(static_cast<int> (queries.size()), 1);
    
    const OctreeImpl::Queries& q = (const OctreeImpl::Queries&)queries;
    static_cast<OctreeImpl*>(impl)->approxNearestSearch(q, results);
}

void pcl::gpu::Octree::nearestKSearchBatch(const Queries& queries, int k, NeighborIndices& results) const
{    
    if (k != 1)
        throw pcl::PCLException("OctreeGPU::knnSearch is supported only for k == 1", __FILE__, "", __LINE__);
    
    assert(queries.size() > 0);
    results.create(static_cast<int> (queries.size()), k);	    

	const OctreeImpl::Queries& q = (const OctreeImpl::Queries&)queries;

    static_cast<OctreeImpl*>(impl)->nearestKSearchBatch(q, k, results);
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////// Brute Force Radius Search Mediator //////////////////////////////////

void pcl::gpu::bruteForceRadiusSearchGPU(const Octree::PointCloud& cloud, const PointXYZ& query,  float radius,  DeviceArray<int>& result,  DeviceArray<int>& buffer)
{
    typedef OctreeImpl::PointType PointType;
    typedef OctreeImpl::PointCloud PointCloud;    
    
    PointType query_local;
    query_local.x = query.x;
    query_local.y = query.y;
    query_local.z = query.z;

    Static<sizeof(PointType) == sizeof(OctreeImpl::PointType)>::check();

    PointCloud cloud_local((PointType*)cloud.ptr(), cloud.size());
    bruteForceRadiusSearch(cloud_local, query_local, radius, result, buffer);
}
