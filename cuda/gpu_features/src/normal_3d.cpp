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

#include "pcl/gpu/features/normal_3d.hpp"
#include "internal.hpp"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(pcl::PointXYZ)

pcl::gpu::NormalEstimation::NormalEstimation() : vpx_(0), vpy_(0), vpz_(0) {}


void pcl::gpu::NormalEstimation::computeNormals(const PointCloud& cloud, const NeighborIndices& nn_indices, Normals& normals)
{       
    normals.create(nn_indices.neighboors_size());    

    const device::PointCloud& c = (const device::PointCloud&)cloud;
    device::Normals& n = (device::Normals&)normals;

    device::computeNormals(c, nn_indices, n); 
}

void pcl::gpu::NormalEstimation::flipNormalTowardsViewpoint(const PointCloud& cloud, float vp_x, float vp_y, float vp_z, Normals& normals)
{    
    const device::PointCloud& c = (const device::PointCloud&)cloud;
    device::Normals& n = (device::Normals&)normals;

    device::flipNormalTowardsViewpoint(c, make_float3(vp_x, vp_y, vp_z), n);
}

void pcl::gpu::NormalEstimation::flipNormalTowardsViewpoint(const PointCloud& cloud, const Indices& indices, float vp_x, float vp_y, float vp_z, Normals& normals)
{
    const device::PointCloud& c = (const device::PointCloud&)cloud;
    device::Normals& n = (device::Normals&)normals;

    device::flipNormalTowardsViewpoint(c, indices, make_float3(vp_x, vp_y, vp_z), n);
}


void pcl::gpu::NormalEstimation::setViewPoint (float vpx, float vpy, float vpz)
{
    vpx_ = vpx; vpy_ = vpy; vpz_ = vpz;
}

void pcl::gpu::NormalEstimation::getViewPoint (float &vpx, float &vpy, float &vpz)
{
    vpx = vpx_; vpy = vpy_; vpz = vpz_;
}

void pcl::gpu::NormalEstimation::setInputCloud(const PointCloud& cloud)
{
    cloud_ = cloud;
}

void pcl::gpu::NormalEstimation::setSearchSurface(const PointCloud& surface)
{
    surface_ = surface;
}

void pcl::gpu::NormalEstimation::setIndices(const Indices& indices)
{
    indices_ = indices;
}

void pcl::gpu::NormalEstimation::setRadiusSearch(float radius, int max_results)
{
    radius_ = radius;  max_results_ = max_results;
}

void pcl::gpu::NormalEstimation::compute(Normals& normals)
{
    assert(!cloud_.empty());

    PointCloud& surface = surface_.empty() ? cloud_ : surface_;

    octree_.setCloud(surface);
    octree_.build();

    if (indices_.empty() || (!indices_.empty() && indices_.size() == cloud_.size()))
    {
        octree_.radiusSearch(cloud_, radius_, max_results_, nn_indices_);        
        computeNormals(surface, nn_indices_, normals);
        flipNormalTowardsViewpoint(cloud_, vpx_, vpy_, vpz_, normals);
    }
    else
    {
        octree_.radiusSearch(cloud_, indices_, radius_, max_results_, nn_indices_);
        computeNormals(surface, nn_indices_, normals);
        flipNormalTowardsViewpoint(cloud_, indices_, vpx_, vpy_, vpz_, normals);
    }    
}
