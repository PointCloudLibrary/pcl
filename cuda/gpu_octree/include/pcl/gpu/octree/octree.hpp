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

#include "pcl/pcl_macros.h"
#include "pcl/gpu/common/device_array.hpp"

namespace pcl
{
    namespace gpu
    {        
		struct PointXYZ
		{
			float x, y, z; 
		};

        class PCL_EXPORTS Octree
        {
        public:
            Octree();
            virtual ~Octree();

            /* Types */

            typedef pcl::gpu::PointXYZ PointTypeGpu;
            typedef DeviceArray_<PointXYZ> PointCloud;
            
            typedef DeviceArray_<PointXYZ> BatchQueries;
            typedef DeviceArray_<int> BatchResult;
            typedef DeviceArray_<int> BatchResultSizes;

            /*  Methods */            
            void setCloud(const PointCloud& cloud_arg);

			void build();

            void internalDownload();
            void radiusSearchHost(const PointXYZ& center, float radius, std::vector<int>& out, int max_nn = INT_MAX);

            void radiusSearchBatchGPU(const BatchQueries& centers, float radius, int max_results, BatchResult& out, BatchResultSizes& out_sizes) const;
        private:
            void *impl;            
        };        

        
        PCL_EXPORTS void bruteForceRadiusSearchGPU(const Octree::PointCloud& cloud, const PointXYZ& query, float radius, DeviceArray_<int>& result, DeviceArray_<int>& buffer);
    }
}

#endif /* _PCL_GPU_OCTREE_ */