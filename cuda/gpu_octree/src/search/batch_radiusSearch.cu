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

#include "cuda_interface.hpp"
#include "search/batch_radiusSearch.hpp"

#include "utils/funcattrib.hpp"

using namespace pcl::device::batch_radius_search;

template<typename BatchType>
void pcl::gpu::OctreeImpl::radiusSearchBatchEx(BatchType& batch, const BatchQueries& queries, int max_results, BatchResult& output, BatchResultSizes& out_sizes)
{
          
    batch.indices = indices;
    batch.octree = octreeGlobal;

    batch.queries_num = (int)queries.size();
    batch.max_results = max_results;
    
    batch.output = output;                
    batch.output_sizes = out_sizes;

    batch.points  = points_sorted;
    batch.points_step = points_sorted.step()/points_sorted.elem_size;
    batch.queries = queries;
    
    int block = pcl::device::batch_radius_search::KernelPolicy::CTA_SIZE;
    int grid = (batch.queries_num + block - 1) / block;    

    cudaSafeCall( cudaFuncSetCacheConfig(pcl::device::batch_radius_search::KernelB<BatchType>, cudaFuncCachePreferL1) );

    pcl::device::batch_radius_search::KernelB<<<grid, block>>>(batch);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}


void pcl::gpu::OctreeImpl::radiusSearchBatch(const BatchQueries& queries, float radius, int max_results, BatchResult& output, BatchResultSizes& out_sizes)
{        
    typedef OctreeImpl::PointType PointType;
    typedef Batch<PointType, SharedRadius> BatchType;

    BatchType batch;
    batch.radius = radius;
    radiusSearchBatchEx(batch, queries, max_results, output, out_sizes);              
}

void pcl::gpu::OctreeImpl::radiusSearchBatch(const BatchQueries& queries, const BatchRadiuses& radiuses, int max_results, BatchResult& output, BatchResultSizes& out_sizes)
{
    typedef OctreeImpl::PointType PointType;
    typedef Batch<PointType, IndividualRadius> BatchType;

    BatchType batch;
    batch.radiuses = radiuses;
    radiusSearchBatchEx(batch, queries, max_results, output, out_sizes);              
}