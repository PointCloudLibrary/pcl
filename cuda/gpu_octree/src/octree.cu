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

#include "pcl/gpu/octree/octree.hpp"
#include "pcl/gpu/common/timers_cuda.hpp"
#include "pcl/gpu/common/safe_call.hpp"

#include "cuda_interface.hpp"
#include "utils/boxutils.hpp"

using namespace pcl::gpu;
using namespace pcl::cuda;
using namespace pcl::device;
using namespace std;

namespace pcl
{
    namespace cuda
    {
        namespace getcc
        {
            __global__ void get_cc_kernel(int *data)
            {
                data[threadIdx.x + blockDim.x * blockIdx.x] = threadIdx.x;
            }
        }
    }
}

void  pcl::gpu::OctreeImpl::get_gpu_arch_compiled_for(int& bin, int& ptx)
{
    cudaFuncAttributes attrs;
    cudaSafeCall( cudaFuncGetAttributes(&attrs, getcc::get_cc_kernel) );  
    bin = attrs.binaryVersion;
    ptx = attrs.ptxVersion;
}

void pcl::gpu::OctreeImpl::setCloud(const pcl::gpu::DeviceArray_<float3>& input_points)
{
    points_num = input_points.size();
    points = input_points;
}


int getBitsNum(int interger)
{
    int count = 0;
    while(interger > 0)
    {
        if (interger & 1)
            ++count;
        interger>>=1;
    }
    return count;
} 

struct OctreeIteratorHost
{        
    const static int MAX_LEVELS_PLUS_ROOT = 11;
    int paths[MAX_LEVELS_PLUS_ROOT];          
    int level;

    OctreeIteratorHost()
    {
        level = 0; // root level
        paths[level] = (0 << 8) + 1;                    
    }

    void gotoNextLevel(int first, int len) 
    {   
        ++level;
        paths[level] = (first << 8) + len;        
    }       

    int operator*() const 
    { 
        return paths[level] >> 8; 
    }        

    void operator++()
    {
        while(level >= 0)
        {
            int data = paths[level];

            if ((data & 0xFF) > 1) // there are another siblings, can goto there
            {                           
                data += (1 << 8) - 1;  // +1 to first and -1 from len
                paths[level] = data;
                break;
            }
            else
                --level; //goto parent;            
       }        
    }        
};

void pcl::gpu::OctreeImpl::radiusSearchHost(const float3& center, float radius, vector<int>& out, int max_nn) const
{            
    out.clear();  

    OctreeIteratorHost iterator;

    while(iterator.level >= 0)
    {        
        int node_idx = *iterator;
        int code = host_octree.node_codes[node_idx];

        float3 node_minp = octreeGlobal.minp;
        float3 node_maxp = octreeGlobal.maxp;        
        calcBoundingBox(iterator.level, code, node_minp, node_maxp);

        //if true, take nothing, and go to next
        if (checkIfNodeOutsideSphere(node_minp, node_maxp, center, radius))        
        {                
            ++iterator;            
            continue;
        }

        //if true, take all, and go to next
        if (checkIfNodeInsideSphere(node_minp, node_maxp, center, radius))
        {            
            int beg = host_octree.begs[node_idx];
            int end = host_octree.ends[node_idx];

            end = beg + min<int>(out.size() + end - beg, max_nn) - out.size();

            out.insert(out.end(), host_octree.indices.begin() + beg, host_octree.indices.begin() + end);
            if (out.size() == max_nn)
                return;

            ++iterator;
            continue;
        }

        // test children
        int children_mask = host_octree.nodes[node_idx] & 0xFF;
        
        bool isLeaf = children_mask == 0;

        if (isLeaf)
        {            
            const int beg = host_octree.begs[node_idx];
            const int end = host_octree.ends[node_idx];                                    

            for(int j = beg; j < end; ++j)
            {
                int index = host_octree.indices[j];
                const float3& point = host_octree.points_sorted[j];

                float dx = (point.x - center.x);
                float dy = (point.y - center.y);
                float dz = (point.z - center.z);

                float dist2 = dx * dx + dy * dy + dz * dz;

                if (dist2 < radius * radius)
                    out.push_back(index);
                
                if (out.size() == max_nn)
                    return;
            }               
            ++iterator;               
            continue;
        }
        
        int first  = host_octree.nodes[node_idx] >> 8;
        iterator.gotoNextLevel(first, getBitsNum(children_mask));                
    }
}

void pcl::gpu::OctreeImpl::internalDownload()
{
    int number;
    DeviceArray_<int>(octreeGlobal.nodes_num, 1).download(&number); 

    DeviceArray_<int>(octreeGlobal.begs,  number).download(host_octree.begs);    
    DeviceArray_<int>(octreeGlobal.ends,  number).download(host_octree.ends);    
    DeviceArray_<int>(octreeGlobal.nodes, number).download(host_octree.nodes);    
    DeviceArray_<int>(octreeGlobal.codes, number).download(host_octree.node_codes); 

    points_sorted.download(host_octree.points_sorted);    
    indices.download(host_octree.indices);    

    host_octree.downloaded = true;
}


//////////////////////////////////////////////////////////////////////////////////////
//////////////// Octree Host Interface implementation ////////////////////////////////

pcl::gpu::Octree::Octree() : impl(0)
{
    int device;
    cudaSafeCall( cudaGetDevice( &device ) );
    
    cudaDeviceProp prop;
    cudaSafeCall( cudaGetDeviceProperties( &prop, device) );

    if (prop.major < 2)
        pcl::cuda::error("This code requires devices with compute capabiliti >= 2.0", __FILE__, __LINE__);

    int bin, ptx;
    OctreeImpl::get_gpu_arch_compiled_for(bin, ptx);

    if (bin < 20 && ptx < 20)
        pcl::cuda::error("This must be compiled for compute capability >= 2.0", __FILE__, __LINE__);    

    impl = new pcl::gpu::OctreeImpl(prop.multiProcessorCount);        
}

pcl::gpu::Octree::~Octree() 
{
    if (impl)
        delete static_cast<OctreeImpl*>(impl);
}

void pcl::gpu::Octree::setCloud(const PointCloud& cloud_arg)
{
    const DeviceArray_<float3>& cloud = (const DeviceArray_<float3>&)cloud_arg;
    static_cast<OctreeImpl*>(impl)->setCloud(cloud);
}

void pcl::gpu::Octree::build()
{
    static_cast<OctreeImpl*>(impl)->build();    
}

void pcl::gpu::Octree::internalDownload()
{
    static_cast<OctreeImpl*>(impl)->internalDownload();
}

void pcl::gpu::Octree::radiusSearchHost(const PointXYZ& center, float radius, std::vector<int>& out, int max_nn)
{
    if (!static_cast<OctreeImpl*>(impl)->host_octree.downloaded)
        internalDownload();

    float3 c = *(float3*)(&center.x);
    static_cast<OctreeImpl*>(impl)->radiusSearchHost(c, radius, out, max_nn);
}


void pcl::gpu::Octree::radiusSearchBatchGPU(const BatchQueries& queries, float radius, int max_results, BatchResult& output, BatchResultSizes& out_sizes) const
{
    out_sizes.create(queries.size());
    output.create(queries.size() * max_results);

    const DeviceArray_<float3>& q = (const DeviceArray_<float3>&)queries;
    static_cast<OctreeImpl*>(impl)->radiusSearchBatch(q, radius, max_results, output, out_sizes);
}
