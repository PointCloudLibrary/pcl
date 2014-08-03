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

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/segmentation/impl/supervoxel_clustering.hpp>

#include <pcl/octree/impl/octree_pointcloud_adjacency.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_iterator.hpp>

template class pcl::SupervoxelClustering<pcl::PointXYZRGBA>;
template class pcl::SupervoxelClustering<pcl::PointXYZRGB>;
template class pcl::SupervoxelClustering<pcl::PointXYZ>;

typedef pcl::SupervoxelClustering<pcl::PointXYZ>::VoxelData VoxelDataT;
typedef pcl::SupervoxelClustering<pcl::PointXYZRGB>::VoxelData VoxelDataRGBT;
typedef pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData VoxelDataRGBAT;

typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT> AdjacencyContainerT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT> AdjacencyContainerRGBT;
typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT> AdjacencyContainerRGBAT;

template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZ, VoxelDataT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGB, VoxelDataRGBT>;
template class pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, VoxelDataRGBAT>;

template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZ, AdjacencyContainerT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGB, AdjacencyContainerRGBT>;
template class pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGBA, AdjacencyContainerRGBAT>;

//template class pcl::octree::OctreeBase<AdjacencyContainerRGBT, pcl::octree::OctreeContainerEmpty>;
//template class pcl::octree::OctreeBase<AdjacencyContainerRGBAT, pcl::octree::OctreeContainerEmpty>;

//template class pcl::octree::OctreeBreadthFirstIterator<pcl::octree::OctreeBase<AdjacencyContainerRGBT, pcl::octree::OctreeContainerEmpty> >;
//template class pcl::octree::OctreeBreadthFirstIterator<pcl::octree::OctreeBase<AdjacencyContainerRGBAT, pcl::octree::OctreeContainerEmpty> >;

