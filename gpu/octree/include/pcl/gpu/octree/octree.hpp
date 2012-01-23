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

#ifndef _PCL_GPU_OCTREE_
#define _PCL_GPU_OCTREE_

#include <vector>

#include "pcl/point_types.h"
#include "pcl/pcl_macros.h"
#include "pcl/gpu/containers/device_array.hpp"
#include "pcl/gpu/octree/device_format.hpp"

namespace pcl
{
    namespace gpu
    {   
        class PCL_EXPORTS Octree
        {
        public:
            Octree();
            virtual ~Octree();

            /* Types */

            typedef pcl::PointXYZ PointType;
            typedef DeviceArray<PointType> PointCloud;
            
            typedef DeviceArray<PointType> Queries;
            typedef DeviceArray<float> Radiuses;            
            typedef DeviceArray<int> Indices;    
            
            typedef DeviceArray<float> ResultSqrDists;
            
            /*  Methods */            
            void setCloud(const PointCloud& cloud_arg);

			void build();

            void internalDownload();
            void radiusSearchHost(const PointType& center, float radius, std::vector<int>& out, int max_nn = INT_MAX);
            void approxNearestSearchHost(const PointType& query, int& out_index, float& sqr_dist);

            void radiusSearch(const Queries& centers, float radius, int max_results, NeighborIndices& result) const;
            void radiusSearch(const Queries& centers, const Radiuses& radiuses, int max_results, NeighborIndices& result) const;

            void radiusSearch(const Queries& centers, const Indices& indices, float radius, int max_results, NeighborIndices& result) const;

            void approxNearestSearch(const Queries& queries, NeighborIndices& result) const;

            void clear();
            
        private:
            void *impl;            
        };        

        
        PCL_EXPORTS void bruteForceRadiusSearchGPU(const Octree::PointCloud& cloud, const Octree::PointType& query, float radius, DeviceArray<int>& result, DeviceArray<int>& buffer);
    }
}

#endif /* _PCL_GPU_OCTREE_ */