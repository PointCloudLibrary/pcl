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
 */

#ifndef PCL_OCTREE_POINTCLOUD_DYNAMIC_DEPTH_HPP
#define PCL_OCTREE_POINTCLOUD_DYNAMIC_DEPTH_HPP

#include <pcl/octree/octree_pointcloud_dynamic_depth.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename BranchContainerT, typename OctreeT> void
pcl::octree::OctreePointCloudDynamicDepth<PointT, BranchContainerT, OctreeT>::addPointIdx (const int point_idx_arg)
{
  OctreeKey key;
  
  assert (point_idx_arg < static_cast<int> (this->input_->points.size ()));
  
  const PointT& point = this->input_->points[point_idx_arg];
  
  // make sure bounding box is big enough
  this->adoptBoundingBoxToPoint (point);
  
  // generate key
  this->genOctreeKeyforPoint (point, key);
  
  LeafNode* leaf_node;
  BranchNode* parent_branch_of_leaf_node;
  unsigned int depth_mask = this->createLeafRecursive (key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);
  
  if (depth_mask)
  {
    // get amount of objects in leaf container
    std::size_t leaf_obj_count = (*leaf_node)->getSize ();
    
    while  (leaf_obj_count>=max_objs_per_leaf_ && depth_mask)
    {
      // index to branch child
      unsigned char child_idx = key.getChildIdxWithDepthMask (depth_mask*2);
      
      expandLeafNode (leaf_node,
                      parent_branch_of_leaf_node,
                      child_idx,
                      depth_mask);
      
      depth_mask = this->createLeafRecursive (key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);
      leaf_obj_count = (*leaf_node)->getSize ();
    }
    
  }

  LeafContainerTraits<LeafContainerT>::insert (leaf_node->getContainer (), point_idx_arg, point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename BranchContainerT, typename OctreeT> void
pcl::octree::OctreePointCloudDynamicDepth<PointT, BranchContainerT, OctreeT>::expandLeafNode (LeafNode* leaf_node, BranchNode* parent_branch, unsigned char child_idx, unsigned int depth_mask)
{
  
  if (depth_mask)
  {
    // get amount of objects in leaf container
    size_t leaf_obj_count = (*leaf_node)->getSize ();
    
    // copy leaf data
    std::vector<int> leafIndices;
    leafIndices.reserve(leaf_obj_count);
    
    (*leaf_node)->getPointIndices(leafIndices);
    
    // delete current leaf node
    this->deleteBranchChild(*parent_branch, child_idx);
    this->leaf_count_ --;
    
    // create new branch node
    BranchNode* childBranch = this->createBranchChild (*parent_branch, child_idx);
    this->branch_count_ ++;
    
    typename std::vector<int>::iterator it = leafIndices.begin();
    typename std::vector<int>::const_iterator it_end = leafIndices.end();
    
    // add data to new branch
    OctreeKey new_index_key;
    
    for (it = leafIndices.begin(); it!=it_end; ++it)
    {
      
      const PointT& point_from_index = this->input_->points[*it];
      // generate key
      this->genOctreeKeyforPoint (point_from_index, new_index_key);
      
      LeafNode* newLeaf;
      BranchNode* newBranchParent;
      this->createLeafRecursive (new_index_key, depth_mask, childBranch, newLeaf, newBranchParent);
      
      //(*newLeaf)->addPointIndex(*it);
      //(*newLeaf)->insert (*it, point_from_index);
      LeafContainerTraits<LeafContainerT>::insert (newLeaf->getContainer (), *it, point_from_index);
    }
  }
  
  
}

#define PCL_INSTANTIATE_OctreePointCloudDynamicDepth(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudDynamicDepth<T>;

#endif

