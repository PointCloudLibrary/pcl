/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *
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
 * Author : jpapon@gmail.com
 * Email  : jpapon@gmail.com
 *
 */

/*
 * Do not use pre-compiled versions in this compilation unit (cpp-file),
 * especially for the octree classes. This way the OctreePointCloudAdjacency
 * class is instantiated with the custom leaf container SupervoxelClustering.
 */
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/segmentation/impl/supervoxel_clustering.hpp>
#include <pcl/octree/impl/octree_pointcloud_adjacency.hpp>

namespace pcl
{ 
  namespace octree
  {
    //Explicit overloads for RGB types
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB,pcl::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData>::addPoint (const pcl::PointXYZRGB &new_point)
    {
      ++num_points_;
      //Same as before here
      data_.xyz_[0] += new_point.x;
      data_.xyz_[1] += new_point.y;
      data_.xyz_[2] += new_point.z;
      //Separate sums for r,g,b since we cant sum in uchars
      data_.rgb_[0] += static_cast<float> (new_point.r); 
      data_.rgb_[1] += static_cast<float> (new_point.g); 
      data_.rgb_[2] += static_cast<float> (new_point.b); 
    }
    
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA,pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData>::addPoint (const pcl::PointXYZRGBA &new_point)
    {
      ++num_points_;
      //Same as before here
      data_.xyz_[0] += new_point.x;
      data_.xyz_[1] += new_point.y;
      data_.xyz_[2] += new_point.z;
      //Separate sums for r,g,b since we cant sum in uchars
      data_.rgb_[0] += static_cast<float> (new_point.r); 
      data_.rgb_[1] += static_cast<float> (new_point.g); 
      data_.rgb_[2] += static_cast<float> (new_point.b); 
    }
    
    
    
    //Explicit overloads for RGB types
    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB,pcl::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData>::computeData ()
    {
      data_.rgb_[0] /= (static_cast<float> (num_points_));
      data_.rgb_[1] /= (static_cast<float> (num_points_));
      data_.rgb_[2] /= (static_cast<float> (num_points_));
      data_.xyz_[0] /= (static_cast<float> (num_points_));
      data_.xyz_[1] /= (static_cast<float> (num_points_));
      data_.xyz_[2] /= (static_cast<float> (num_points_));    
    }
    
    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA,pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData>::computeData ()
    {
      data_.rgb_[0] /= (static_cast<float> (num_points_));
      data_.rgb_[1] /= (static_cast<float> (num_points_));
      data_.rgb_[2] /= (static_cast<float> (num_points_));
      data_.xyz_[0] /= (static_cast<float> (num_points_));
      data_.xyz_[1] /= (static_cast<float> (num_points_));
      data_.xyz_[2] /= (static_cast<float> (num_points_));
    }
    
    //Explicit overloads for XYZ types
    template<>
    void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ,pcl::SupervoxelClustering<pcl::PointXYZ>::VoxelData>::addPoint (const pcl::PointXYZ &new_point)
    {
      ++num_points_;
      //Same as before here
      data_.xyz_[0] += new_point.x;
      data_.xyz_[1] += new_point.y;
      data_.xyz_[2] += new_point.z;
    }
    
    template<> void
    pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ,pcl::SupervoxelClustering<pcl::PointXYZ>::VoxelData>::computeData ()
    {
      data_.xyz_[0] /= (static_cast<float> (num_points_));
      data_.xyz_[1] /= (static_cast<float> (num_points_));
      data_.xyz_[2] /= (static_cast<float> (num_points_));
    }
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  template<> void
  pcl::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData::getPoint (pcl::PointXYZRGB &point_arg) const
  {
    point_arg.rgba = static_cast<uint32_t>(rgb_[0]) << 16 | 
    static_cast<uint32_t>(rgb_[1]) << 8 | 
    static_cast<uint32_t>(rgb_[2]);  
    point_arg.x = xyz_[0];
    point_arg.y = xyz_[1];
    point_arg.z = xyz_[2];
  }
  
  template<> void
  pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData::getPoint (pcl::PointXYZRGBA &point_arg ) const
  {
    point_arg.rgba = static_cast<uint32_t>(rgb_[0]) << 16 | 
    static_cast<uint32_t>(rgb_[1]) << 8 | 
    static_cast<uint32_t>(rgb_[2]);  
    point_arg.x = xyz_[0];
    point_arg.y = xyz_[1];
    point_arg.z = xyz_[2];
  }
}

typedef pcl::SupervoxelClustering<pcl::PointXYZ>::VoxelData VoxelDataT;
typedef pcl::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData VoxelDataRGBT;
typedef pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData VoxelDataRGBAT;

typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT> AdjacencyContainerT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT> AdjacencyContainerRGBT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT> AdjacencyContainerRGBAT;

template class pcl::SupervoxelClustering<pcl::PointXYZ>;
template class pcl::SupervoxelClustering<pcl::PointXYZRGB>;
template class pcl::SupervoxelClustering<pcl::PointXYZRGBA>;

template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT>;

template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZ, AdjacencyContainerT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGB, AdjacencyContainerRGBT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGBA, AdjacencyContainerRGBAT>;
