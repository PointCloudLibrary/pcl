/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 */

#ifndef PCL_OCTREE_BASE_HPP
#define PCL_OCTREE_BASE_HPP

#include <vector>

namespace pcl {
namespace octree {
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
OctreeBase<LeafContainerT, BranchContainerT>::OctreeBase()
: root_node_(new BranchNode())
{}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
OctreeBase<LeafContainerT, BranchContainerT>::~OctreeBase()
{
  // deallocate tree structure
  deleteTree();
  delete (root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::setMaxVoxelIndex(
    uindex_t max_voxel_index_arg)
{
  uindex_t tree_depth;

  if (max_voxel_index_arg <= 0) {
    PCL_ERROR("[pcl::octree::OctreeBase::setMaxVoxelIndex] Max voxel index (%lu) must "
              "be > 0!\n",
              max_voxel_index_arg);
    return;
  }

  // tree depth == bitlength of maxVoxels
  tree_depth =
      std::min(static_cast<uindex_t>(OctreeKey::maxDepth),
               static_cast<uindex_t>(std::ceil(std::log2(max_voxel_index_arg))));

  setTreeDepth(tree_depth);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::setTreeDepth(uindex_t depth_arg)
{
  if (depth_arg <= 0) {
    PCL_ERROR("[pcl::octree::OctreeBase::setTreeDepth] Tree depth (%lu) must be > 0!\n",
              depth_arg);
    return;
  }
  if (depth_arg > OctreeKey::maxDepth) {
    PCL_ERROR("[pcl::octree::OctreeBase::setTreeDepth] Tree depth (%lu) must be <= max "
              "depth(%lu)!\n",
              depth_arg,
              OctreeKey::maxDepth);
    return;
  }

  // set octree depth
  octree_depth_ = depth_arg;

  // define depth_mask_ by setting a single bit to 1 at bit position == tree depth
  depth_mask_ = (1 << (depth_arg - 1));

  // define max_key_
  max_key_.x = max_key_.y = max_key_.z = (1 << depth_arg) - 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
LeafContainerT*
OctreeBase<LeafContainerT, BranchContainerT>::findLeaf(uindex_t idx_x_arg,
                                                       uindex_t idx_y_arg,
                                                       uindex_t idx_z_arg) const
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // find the leaf node addressed by key
  return (findLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
LeafContainerT*
OctreeBase<LeafContainerT, BranchContainerT>::createLeaf(uindex_t idx_x_arg,
                                                         uindex_t idx_y_arg,
                                                         uindex_t idx_z_arg)
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // create a leaf node addressed by key
  return (createLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
bool
OctreeBase<LeafContainerT, BranchContainerT>::existLeaf(uindex_t idx_x_arg,
                                                        uindex_t idx_y_arg,
                                                        uindex_t idx_z_arg) const
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // check if key exist in octree
  return (existLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::removeLeaf(uindex_t idx_x_arg,
                                                         uindex_t idx_y_arg,
                                                         uindex_t idx_z_arg)
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // delete the leaf node addressed by key
  deleteLeafRecursive(key, depth_mask_, root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::deleteTree()
{

  if (root_node_) {
    // reset octree
    deleteBranch(*root_node_);
    leaf_count_ = 0;
    branch_count_ = 1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::serializeTree(
    std::vector<char>& binary_tree_out_arg) const
{

  OctreeKey new_key;

  // clear binary vector
  binary_tree_out_arg.clear();
  binary_tree_out_arg.reserve(this->branch_count_);

  serializeTreeRecursive(root_node_, new_key, &binary_tree_out_arg, nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::serializeTree(
    std::vector<char>& binary_tree_out_arg,
    std::vector<LeafContainerT*>& leaf_container_vector_arg) const
{

  OctreeKey new_key;

  // clear output vectors
  binary_tree_out_arg.clear();
  leaf_container_vector_arg.clear();

  binary_tree_out_arg.reserve(this->branch_count_);
  leaf_container_vector_arg.reserve(this->leaf_count_);

  serializeTreeRecursive(
      root_node_, new_key, &binary_tree_out_arg, &leaf_container_vector_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::serializeLeafs(
    std::vector<LeafContainerT*>& leaf_container_vector_arg)
{
  OctreeKey new_key;

  // clear output vector
  leaf_container_vector_arg.clear();

  leaf_container_vector_arg.reserve(this->leaf_count_);

  serializeTreeRecursive(root_node_, new_key, nullptr, &leaf_container_vector_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::deserializeTree(
    std::vector<char>& binary_tree_out_arg)
{
  OctreeKey new_key;

  // free existing tree before tree rebuild
  deleteTree();

  // iterator for binary tree structure vector
  auto binary_tree_out_it = binary_tree_out_arg.cbegin();
  auto binary_tree_out_it_end = binary_tree_out_arg.cend();

  deserializeTreeRecursive(root_node_,
                           depth_mask_,
                           new_key,
                           binary_tree_out_it,
                           binary_tree_out_it_end,
                           nullptr,
                           nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::deserializeTree(
    std::vector<char>& binary_tree_in_arg,
    std::vector<LeafContainerT*>& leaf_container_vector_arg)
{
  OctreeKey new_key;

  // set data iterator to first element
  auto leaf_vector_it = leaf_container_vector_arg.cbegin();

  // set data iterator to last element
  auto leaf_vector_it_end = leaf_container_vector_arg.cend();

  // free existing tree before tree rebuild
  deleteTree();

  // iterator for binary tree structure vector
  auto binary_tree_input_it = binary_tree_in_arg.cbegin();
  auto binary_tree_input_it_end = binary_tree_in_arg.cend();

  deserializeTreeRecursive(root_node_,
                           depth_mask_,
                           new_key,
                           binary_tree_input_it,
                           binary_tree_input_it_end,
                           &leaf_vector_it,
                           &leaf_vector_it_end);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreeBase<LeafContainerT, BranchContainerT>::createLeafRecursive(
    const OctreeKey& key_arg,
    uindex_t depth_mask_arg,
    BranchNode* branch_arg,
    LeafNode*& return_leaf_arg,
    BranchNode*& parent_of_leaf_arg)
{
  // index to branch child
  unsigned char child_idx;

  // find branch child from key
  child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  OctreeNode* child_node = (*branch_arg)[child_idx];

  if (!child_node) {
    if ((!dynamic_depth_enabled_) && (depth_mask_arg > 1)) {
      // if required branch does not exist -> create it
      BranchNode* childBranch = createBranchChild(*branch_arg, child_idx);

      branch_count_++;

      // recursively proceed with indexed child branch
      return createLeafRecursive(key_arg,
                                 depth_mask_arg >> 1,
                                 childBranch,
                                 return_leaf_arg,
                                 parent_of_leaf_arg);
    }
    // if leaf node at child_idx does not exist
    LeafNode* leaf_node = createLeafChild(*branch_arg, child_idx);
    return_leaf_arg = leaf_node;
    parent_of_leaf_arg = branch_arg;
    this->leaf_count_++;
  }
  else {
    // Node exists already
    switch (child_node->getNodeType()) {
    case BRANCH_NODE:
      // recursively proceed with indexed child branch
      return createLeafRecursive(key_arg,
                                 depth_mask_arg >> 1,
                                 static_cast<BranchNode*>(child_node),
                                 return_leaf_arg,
                                 parent_of_leaf_arg);
      break;

    case LEAF_NODE:
      return_leaf_arg = static_cast<LeafNode*>(child_node);
      parent_of_leaf_arg = branch_arg;
      break;
    }
  }

  return (depth_mask_arg >> 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::findLeafRecursive(
    const OctreeKey& key_arg,
    uindex_t depth_mask_arg,
    BranchNode* branch_arg,
    LeafContainerT*& result_arg) const
{
  // index to branch child
  unsigned char child_idx;

  // find branch child from key
  child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  OctreeNode* child_node = (*branch_arg)[child_idx];

  if (child_node) {
    switch (child_node->getNodeType()) {
    case BRANCH_NODE:
      // we have not reached maximum tree depth
      BranchNode* child_branch;
      child_branch = static_cast<BranchNode*>(child_node);

      findLeafRecursive(key_arg, depth_mask_arg >> 1, child_branch, result_arg);
      break;

    case LEAF_NODE:
      // return existing leaf node
      LeafNode* child_leaf;
      child_leaf = static_cast<LeafNode*>(child_node);

      result_arg = child_leaf->getContainerPtr();
      break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
bool
OctreeBase<LeafContainerT, BranchContainerT>::deleteLeafRecursive(
    const OctreeKey& key_arg, uindex_t depth_mask_arg, BranchNode* branch_arg)
{
  // index to branch child
  unsigned char child_idx;
  // indicates if branch has children, if so, it can't be removed
  bool b_has_children;

  // find branch child from key
  child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  OctreeNode* child_node = (*branch_arg)[child_idx];

  if (child_node) {
    switch (child_node->getNodeType()) {

    case BRANCH_NODE:
      BranchNode* child_branch;
      child_branch = static_cast<BranchNode*>(child_node);

      // recursively explore the indexed child branch
      b_has_children = deleteLeafRecursive(key_arg, depth_mask_arg >> 1, child_branch);

      if (!b_has_children) {
        // child branch does not own any sub-child nodes anymore -> delete child branch
        deleteBranchChild(*branch_arg, child_idx);
        branch_count_--;
      }
      break;

    case LEAF_NODE:
      // return existing leaf node

      // our child is a leaf node -> delete it
      deleteBranchChild(*branch_arg, child_idx);
      this->leaf_count_--;
      break;
    }
  }

  // check if current branch still owns children
  b_has_children = false;
  for (child_idx = 0; (!b_has_children) && (child_idx < 8); child_idx++) {
    b_has_children = branch_arg->hasChild(child_idx);
  }
  // return false if current branch can be deleted
  return (b_has_children);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::serializeTreeRecursive(
    const BranchNode* branch_arg,
    OctreeKey& key_arg,
    std::vector<char>* binary_tree_out_arg,
    typename std::vector<LeafContainerT*>* leaf_container_vector_arg) const
{
  char node_bit_pattern;

  // branch occupancy bit pattern
  node_bit_pattern = getBranchBitPattern(*branch_arg);

  // write bit pattern to output vector
  if (binary_tree_out_arg)
    binary_tree_out_arg->push_back(node_bit_pattern);

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {

    // if child exist
    if (branch_arg->hasChild(child_idx)) {
      // add current branch voxel to key
      key_arg.pushBranch(child_idx);

      OctreeNode* childNode = branch_arg->getChildPtr(child_idx);

      switch (childNode->getNodeType()) {
      case BRANCH_NODE: {
        // recursively proceed with indexed child branch
        serializeTreeRecursive(static_cast<const BranchNode*>(childNode),
                               key_arg,
                               binary_tree_out_arg,
                               leaf_container_vector_arg);
        break;
      }
      case LEAF_NODE: {
        auto* child_leaf = static_cast<LeafNode*>(childNode);

        if (leaf_container_vector_arg)
          leaf_container_vector_arg->push_back(child_leaf->getContainerPtr());

        // we reached a leaf node -> execute serialization callback
        serializeTreeCallback(**child_leaf, key_arg);
        break;
      }
      default:
        break;
      }

      // pop current branch voxel from key
      key_arg.popBranch();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
OctreeBase<LeafContainerT, BranchContainerT>::deserializeTreeRecursive(
    BranchNode* branch_arg,
    uindex_t depth_mask_arg,
    OctreeKey& key_arg,
    typename std::vector<char>::const_iterator& binary_tree_input_it_arg,
    typename std::vector<char>::const_iterator& binary_tree_input_it_end_arg,
    typename std::vector<LeafContainerT*>::const_iterator* leaf_container_vector_it_arg,
    typename std::vector<LeafContainerT*>::const_iterator*
        leaf_container_vector_it_end_arg)
{

  if (binary_tree_input_it_arg != binary_tree_input_it_end_arg) {
    // read branch occupancy bit pattern from input vector
    char node_bits = (*binary_tree_input_it_arg);
    binary_tree_input_it_arg++;

    // iterate over all children
    for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
      // if occupancy bit for child_idx is set..
      if (node_bits & (1 << child_idx)) {
        // add current branch voxel to key
        key_arg.pushBranch(child_idx);

        if (depth_mask_arg > 1) {
          // we have not reached maximum tree depth
          BranchNode* newBranch = createBranchChild(*branch_arg, child_idx);

          branch_count_++;

          // recursively proceed with new child branch
          deserializeTreeRecursive(newBranch,
                                   depth_mask_arg >> 1,
                                   key_arg,
                                   binary_tree_input_it_arg,
                                   binary_tree_input_it_end_arg,
                                   leaf_container_vector_it_arg,
                                   leaf_container_vector_it_end_arg);
        }
        else {
          // we reached leaf node level

          LeafNode* child_leaf = createLeafChild(*branch_arg, child_idx);

          if (leaf_container_vector_it_arg &&
              (*leaf_container_vector_it_arg != *leaf_container_vector_it_end_arg)) {
            LeafContainerT& container = **child_leaf;
            LeafContainerT* src_container_ptr = **leaf_container_vector_it_arg;
            container = *src_container_ptr;
            ++*leaf_container_vector_it_arg;
          }

          leaf_count_++;

          // execute deserialization callback
          deserializeTreeCallback(**child_leaf, key_arg);
        }

        // pop current branch voxel from key
        key_arg.popBranch();
      }
    }
  }
}

} // namespace octree
} // namespace pcl

#define PCL_INSTANTIATE_OctreeBase(T)                                                  \
  template class PCL_EXPORTS pcl::octree::OctreeBase<T>;

#endif
