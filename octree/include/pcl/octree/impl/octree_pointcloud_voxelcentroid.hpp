/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id: octree_pointcloud_voxelcentroid.hpp 6459 2012-07-18 07:50:37Z dpb $
 */

#ifndef PCL_OCTREE_VOXELCENTROID_HPP
#define PCL_OCTREE_VOXELCENTROID_HPP

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> bool
pcl::octree::OctreePointCloudVoxelCentroid<PointT, LeafContainerT, BranchContainerT>::getVoxelCentroidAtPoint (
    const PointT& point_arg, PointT& voxel_centroid_arg) const
{
  OctreeKey key;
  LeafNode* leaf = 0;

  // generate key
  genOctreeKeyforPoint (point_arg, key);

  leaf = this->findLeaf (key);

  if (leaf)
  {
    LeafContainerT* container = leaf;
    container->getCentroid (voxel_centroid_arg);
  }

  return (leaf != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> size_t
pcl::octree::OctreePointCloudVoxelCentroid<PointT, LeafContainerT, BranchContainerT>::getVoxelCentroids (
    typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::AlignedPointTVector &voxel_centroid_list_arg) const
{
  OctreeKey new_key;

  // reset output vector
  voxel_centroid_list_arg.clear ();
  voxel_centroid_list_arg.reserve (this->leaf_count_);

  getVoxelCentroidsRecursive (this->root_node_, new_key, voxel_centroid_list_arg );

  // return size of centroid vector
  return (voxel_centroid_list_arg.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudVoxelCentroid<PointT, LeafContainerT, BranchContainerT>::getVoxelCentroidsRecursive (
    const BranchNode* branch_arg, OctreeKey& key_arg,
    typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::AlignedPointTVector &voxel_centroid_list_arg) const
{
  // child iterator
  unsigned char child_idx;

  // iterate over all children
  for (child_idx = 0; child_idx < 8; child_idx++)
  {
    // if child exist
    if (branch_arg->hasChild (child_idx))
    {
      // add current branch voxel to key
      key_arg.pushBranch (child_idx);

      OctreeNode *child_node = branch_arg->getChildPtr (child_idx);

      switch (child_node->getNodeType ())
      {
        case BRANCH_NODE:
        {
          // recursively proceed with indexed child branch
          getVoxelCentroidsRecursive (static_cast<const BranchNode*> (child_node), key_arg, voxel_centroid_list_arg);
          break;
        }
        case LEAF_NODE:
        {
          PointT new_centroid;

          LeafNode* container = static_cast<LeafNode*> (child_node);

          container->getContainer().getCentroid (new_centroid);

          voxel_centroid_list_arg.push_back (new_centroid);
          break;
        }
        default:
          break;
       }

      // pop current branch voxel from key
      key_arg.popBranch ();
    }
  }
}


#define PCL_INSTANTIATE_OctreePointCloudVoxelCentroid(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudVoxelCentroid<T>;

#endif

