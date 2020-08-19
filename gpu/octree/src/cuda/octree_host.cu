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

#include "pcl/gpu/utils/timers_cuda.hpp"
#include "pcl/gpu/utils/safe_call.hpp"

#include "internal.hpp"
#include "utils/approx_nearest_utils.hpp"
#include "utils/boxutils.hpp"

#include<algorithm>
#include<limits>
#include <tuple>

using namespace pcl::gpu;
using namespace pcl::device;

namespace pcl
{
    namespace device
    {
        __global__ void get_cc_kernel(int *data)
        {
            data[threadIdx.x + blockDim.x * blockIdx.x] = threadIdx.x;
        }
    }
}

void  pcl::device::OctreeImpl::get_gpu_arch_compiled_for(int& bin, int& ptx)
{
    cudaFuncAttributes attrs;
    cudaSafeCall( cudaFuncGetAttributes(&attrs, get_cc_kernel) );  
    bin = attrs.binaryVersion;
    ptx = attrs.ptxVersion;
}

void pcl::device::OctreeImpl::setCloud(const PointCloud& input_points)
{
    points = input_points;
}

void pcl::device::OctreeImpl::internalDownload()
{
    int number;
    DeviceArray<int>(octreeGlobal.nodes_num, 1).download(&number); 

    DeviceArray<int>(octreeGlobal.begs,  number).download(host_octree.begs);    
    DeviceArray<int>(octreeGlobal.ends,  number).download(host_octree.ends);    
    DeviceArray<int>(octreeGlobal.nodes, number).download(host_octree.nodes);    
    DeviceArray<int>(octreeGlobal.codes, number).download(host_octree.codes); 

    points_sorted.download(host_octree.points_sorted, host_octree.points_sorted_step);    
    indices.download(host_octree.indices);    

    host_octree.downloaded = true;
}

namespace 
{
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

}

void pcl::device::OctreeImpl::radiusSearchHost(const PointType& query, float radius, std::vector<int>& out, int max_nn) const
{            
    out.clear();  

    float3 center = make_float3(query.x, query.y, query.z);

    OctreeIteratorHost iterator;

    while(iterator.level >= 0)
    {        
        int node_idx = *iterator;
        int code = host_octree.codes[node_idx];

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

            end = beg + std::min<int>((int)out.size() + end - beg, max_nn) - (int)out.size();

            out.insert(out.end(), host_octree.indices.begin() + beg, host_octree.indices.begin() + end);
            if (out.size() == (std::size_t)max_nn)
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
                float point_x = host_octree.points_sorted[j                                     ];
                float point_y = host_octree.points_sorted[j + host_octree.points_sorted_step    ];
                float point_z = host_octree.points_sorted[j + host_octree.points_sorted_step * 2];

                float dx = (point_x - center.x);
                float dy = (point_y - center.y);
                float dz = (point_z - center.z);

                float dist2 = dx * dx + dy * dy + dz * dz;

                if (dist2 < radius * radius)
                    out.push_back(index);

                if (out.size() == (std::size_t)max_nn)
                    return;
            }               
            ++iterator;               
            continue;
        }

        int first  = host_octree.nodes[node_idx] >> 8;        
        iterator.gotoNextLevel(first, getBitsNum(children_mask));                
    }
}

void  pcl::device::OctreeImpl::approxNearestSearchHost(const PointType& query, int& out_index, float& sqr_dist) const
{
    const float3& minp = octreeGlobal.minp;
    const float3& maxp = octreeGlobal.maxp;
    const float3 query_point = make_float3(query.x, query.y, query.z);

    const int node_idx = pcl::device::findNode(minp, maxp, query_point, host_octree.nodes);

    int beg = host_octree.begs[node_idx];
    int end = host_octree.ends[node_idx];

    sqr_dist = std::numeric_limits<float>::max();

    for(int i = beg; i < end; ++i)
    {
        float point_x = host_octree.points_sorted[i                                     ];
        float point_y = host_octree.points_sorted[i + host_octree.points_sorted_step    ];
        float point_z = host_octree.points_sorted[i + host_octree.points_sorted_step * 2];

        float dx = (point_x - query.x);
        float dy = (point_y - query.y);
        float dz = (point_z - query.z);

        float d2 = dx * dx + dy * dy + dz * dz;

        if (sqr_dist > d2)
        {
            sqr_dist = d2;
            out_index = i;
        }
    }

    out_index = host_octree.indices[out_index];
}
