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
#include "search/batch_appNearestSearch.hpp"

#include "utils/funcattrib.hpp"

void pcl::gpu::OctreeImpl::approxNearestSearchBatch(const BatchQueries& queries, BatchResult& output) const
{
    typedef OctreeImpl::PointType PointType;
    typedef pcl::device::batch_appnearest_search::Batch<PointType> BatchType;


    BatchType batch;
    batch.indices = indices;
    batch.octree = octreeGlobal;

    batch.queries_num = (int)queries.size();        
    batch.output = output;     

    batch.points = points_sorted;
    batch.points_step = points_sorted.step()/points_sorted.elem_size;
    batch.queries = queries;

    int block = pcl::device::batch_appnearest_search::KernelPolicy::CTA_SIZE;
    int grid = (batch.queries_num + block - 1) / block;    



    cudaSafeCall( cudaFuncSetCacheConfig(pcl::device::batch_appnearest_search::KernelB<PointType>, cudaFuncCachePreferL1) );

    pcl::device::batch_appnearest_search::KernelB<<<grid, block>>>(batch);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}