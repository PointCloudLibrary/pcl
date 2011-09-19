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


#ifndef PCL_GPU_NORMAL_3D_HPP_
#define PCL_GPU_NORMAL_3D_HPP_

#include "pcl/pcl_macros.h"
#include "pcl/point_types.h"
#include "pcl/gpu/containers/device_array.hpp"
#include "pcl/gpu/octree/device_format.hpp"
#include "pcl/gpu/octree/octree.hpp"


namespace pcl
{
    namespace gpu
    {
        class PCL_EXPORTS NormalEstimation
        {
        public:

            typedef PointXYZ PointType;
            typedef PointXYZ NormalType;  // float x, y, z, curvature; -> sizeof(PointXYZ) = 4 * sizeof(float)

            typedef DeviceArray< PointType> PointCloud;
            typedef DeviceArray<NormalType> Normals;
            typedef DeviceArray<int> Indices;

            static void computeNormals(const PointCloud& cloud, const NeighborIndices& nn_indices, Normals& normals);
            static void flipNormalTowardsViewpoint(const PointCloud& cloud, float vp_x, float vp_y, float vp_z, Normals& normals);            
            static void flipNormalTowardsViewpoint(const PointCloud& cloud, const Indices& indices, float vp_x, float vp_y, float vp_z, Normals& normals);
 
            NormalEstimation();

            void compute(Normals& normals);

            void setInputCloud(const PointCloud& cloud);
            void setSearchSurface(const PointCloud& surface);
            void setIndices(const Indices& indices);
            void setRadiusSearch(float radius, int max_results);

            void setViewPoint(float  vpx, float  vpy, float  vpz);  
            void getViewPoint(float& vpx, float& vpy, float& vpz);      
        private:              
            float vpx_, vpy_, vpz_;
            PointCloud cloud_;
            PointCloud surface_;
            Indices indices_;
            float radius_;
            int max_results_;

            Octree octree_;
            NeighborIndices nn_indices_;
        };        
    }
}

#endif /* PCL_GPU_NORMAL_3D_HPP_ */
