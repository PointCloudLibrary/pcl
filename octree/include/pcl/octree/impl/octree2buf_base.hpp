/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_OCTREE_2BUF_BASE_HPP
#define PCL_OCTREE_2BUF_BASE_HPP

namespace pcl {
namespace octree {
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
Octree2BufBase<LeafContainerT, BranchContainerT>::Octree2BufBase()
: leaf_count_(0)
, branch_count_(1)
, root_node_(new BranchNode())
, depth_mask_(0)
, buffer_selector_(0)
, tree_dirty_flag_(false)
, octree_depth_(0)
, dynamic_depth_enabled_(false)
{}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
Octree2BufBase<LeafContainerT, BranchContainerT>::~Octree2BufBase()
{
  // deallocate tree structure
  deleteTree();
  delete (root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::setMaxVoxelIndex(
    uindex_t max_voxel_index_arg)
{
  uindex_t treeDepth;

  assert(max_voxel_index_arg > 0);

  // tree depth == amount of bits of maxVoxels
  treeDepth =
      std::max<uindex_t>(std::min<uindex_t>(OctreeKey::maxDepth,
                                            std::ceil(std::log2(max_voxel_index_arg))),
                         0);

  // define depthMask_ by setting a single bit to 1 at bit position == tree depth
  depth_mask_ = (1 << (treeDepth - 1));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::setTreeDepth(uindex_t depth_arg)
{
  assert(depth_arg > 0);

  // set octree depth
  octree_depth_ = depth_arg;

  // define depthMask_ by setting a single bit to 1 at bit position == tree depth
  depth_mask_ = (1 << (depth_arg - 1));

  // define max. keys
  max_key_.x = max_key_.y = max_key_.z = (1 << depth_arg) - 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
LeafContainerT*
Octree2BufBase<LeafContainerT, BranchContainerT>::findLeaf(uindex_t idx_x_arg,
                                                           uindex_t idx_y_arg,
                                                           uindex_t idx_z_arg)
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // check if key exist in octree
  return (findLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
LeafContainerT*
Octree2BufBase<LeafContainerT, BranchContainerT>::createLeaf(uindex_t idx_x_arg,
                                                             uindex_t idx_y_arg,
                                                             uindex_t idx_z_arg)
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // check if key exist in octree
  return (createLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
bool
Octree2BufBase<LeafContainerT, BranchContainerT>::existLeaf(uindex_t idx_x_arg,
                                                            uindex_t idx_y_arg,
                                                            uindex_t idx_z_arg) const
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // check if key exist in octree
  return existLeaf(key);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::removeLeaf(uindex_t idx_x_arg,
                                                             uindex_t idx_y_arg,
                                                             uindex_t idx_z_arg)
{
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // free voxel at key
  return (this->removeLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::deleteTree()
{
  if (root_node_) {
    // reset octree
    deleteBranch(*root_node_);
    leaf_count_ = 0;
    branch_count_ = 1;

    tree_dirty_flag_ = false;
    depth_mask_ = 0;
    octree_depth_ = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::switchBuffers()
{
  if (tree_dirty_flag_) {
    // make sure that all unused branch nodes from previous buffer are deleted
    treeCleanUpRecursive(root_node_);
  }

  // switch butter selector
  buffer_selector_ = !buffer_selector_;

  // reset flags
  tree_dirty_flag_ = true;
  leaf_count_ = 0;
  branch_count_ = 1;

  // we can safely remove children references of root node
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
    root_node_->setChildPtr(buffer_selector_, child_idx, nullptr);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::serializeTree(
    std::vector<char>& binary_tree_out_arg, bool do_XOR_encoding_arg)
{
  OctreeKey new_key;

  // clear binary vector
  binary_tree_out_arg.clear();
  binary_tree_out_arg.reserve(this->branch_count_);

  serializeTreeRecursive(
      root_node_, new_key, &binary_tree_out_arg, nullptr, do_XOR_encoding_arg, false);

  // serializeTreeRecursive cleans-up unused octree nodes in previous octree
  tree_dirty_flag_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::serializeTree(
    std::vector<char>& binary_tree_out_arg,
    std::vector<LeafContainerT*>& leaf_container_vector_arg,
    bool do_XOR_encoding_arg)
{
  OctreeKey new_key;

  // clear output vectors
  binary_tree_out_arg.clear();
  leaf_container_vector_arg.clear();

  leaf_container_vector_arg.reserve(leaf_count_);
  binary_tree_out_arg.reserve(this->branch_count_);

  serializeTreeRecursive(root_node_,
                         new_key,
                         &binary_tree_out_arg,
                         &leaf_container_vector_arg,
                         do_XOR_encoding_arg,
                         false);

  // serializeTreeRecursive cleans-up unused octree nodes in previous octree
  tree_dirty_flag_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::serializeLeafs(
    std::vector<LeafContainerT*>& leaf_container_vector_arg)
{
  OctreeKey new_key;

  // clear output vector
  leaf_container_vector_arg.clear();

  leaf_container_vector_arg.reserve(leaf_count_);

  serializeTreeRecursive(
      root_node_, new_key, nullptr, &leaf_container_vector_arg, false, false);

  // serializeLeafsRecursive cleans-up unused octree nodes in previous octree
  tree_dirty_flag_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::deserializeTree(
    std::vector<char>& binary_tree_in_arg, bool do_XOR_decoding_arg)
{
  OctreeKey new_key;

  // we will rebuild an octree -> reset leafCount
  leaf_count_ = 0;

  // iterator for binary tree structure vector
  std::vector<char>::const_iterator binary_tree_in_it = binary_tree_in_arg.begin();
  std::vector<char>::const_iterator binary_tree_in_it_end = binary_tree_in_arg.end();

  deserializeTreeRecursive(root_node_,
                           depth_mask_,
                           new_key,
                           binary_tree_in_it,
                           binary_tree_in_it_end,
                           nullptr,
                           nullptr,
                           false,
                           do_XOR_decoding_arg);

  // we modified the octree structure -> clean-up/tree-reset might be required
  tree_dirty_flag_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::deserializeTree(
    std::vector<char>& binary_tree_in_arg,
    std::vector<LeafContainerT*>& leaf_container_vector_arg,
    bool do_XOR_decoding_arg)
{
  OctreeKey new_key;

  // set data iterator to first element
  typename std::vector<LeafContainerT*>::const_iterator leaf_container_vector_it =
      leaf_container_vector_arg.begin();

  // set data iterator to last element
  typename std::vector<LeafContainerT*>::const_iterator leaf_container_vector_it_end =
      leaf_container_vector_arg.end();

  // we will rebuild an octree -> reset leafCount
  leaf_count_ = 0;

  // iterator for binary tree structure vector
  std::vector<char>::const_iterator binary_tree_in_it = binary_tree_in_arg.begin();
  std::vector<char>::const_iterator binary_tree_in_it_end = binary_tree_in_arg.end();

  deserializeTreeRecursive(root_node_,
                           depth_mask_,
                           new_key,
                           binary_tree_in_it,
                           binary_tree_in_it_end,
                           &leaf_container_vector_it,
                           &leaf_container_vector_it_end,
                           false,
                           do_XOR_decoding_arg);

  // we modified the octree structure -> clean-up/tree-reset might be required
  tree_dirty_flag_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::serializeNewLeafs(
    std::vector<LeafContainerT*>& leaf_container_vector_arg)
{
  OctreeKey new_key;

  // clear output vector
  leaf_container_vector_arg.clear();
  leaf_container_vector_arg.reserve(leaf_count_);

  serializeTreeRecursive(
      root_node_, new_key, nullptr, &leaf_container_vector_arg, false, true);

  // serializeLeafsRecursive cleans-up unused octree nodes in previous octree buffer
  tree_dirty_flag_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
uindex_t
Octree2BufBase<LeafContainerT, BranchContainerT>::createLeafRecursive(
    const OctreeKey& key_arg,
    uindex_t depth_mask_arg,
    BranchNode* branch_arg,
    LeafNode*& return_leaf_arg,
    BranchNode*& parent_of_leaf_arg,
    bool branch_reset_arg)
{
  // branch reset -> this branch has been taken from previous buffer
  if (branch_reset_arg) {
    // we can safely remove children references
    for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
      branch_arg->setChildPtr(buffer_selector_, child_idx, nullptr);
    }
  }

  // find branch child from key
  unsigned char child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  if (depth_mask_arg > 1) {
    // we have not reached maximum tree depth
    BranchNode* child_branch;
    bool doNodeReset;

    doNodeReset = false;

    // if required branch does not exist
    if (!branch_arg->hasChild(buffer_selector_, child_idx)) {
      // check if we find a branch node reference in previous buffer

      if (branch_arg->hasChild(!buffer_selector_, child_idx)) {
        OctreeNode* child_node = branch_arg->getChildPtr(!buffer_selector_, child_idx);

        if (child_node->getNodeType() == BRANCH_NODE) {
          child_branch = static_cast<BranchNode*>(child_node);
          branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
        }
        else {
          // depth has changed.. child in preceding buffer is a leaf node.
          deleteBranchChild(*branch_arg, !buffer_selector_, child_idx);
          child_branch = createBranchChild(*branch_arg, child_idx);
        }

        // take child branch from previous buffer
        doNodeReset = true; // reset the branch pointer array of stolen child node
      }
      else {
        // if required branch does not exist -> create it
        child_branch = createBranchChild(*branch_arg, child_idx);
      }

      branch_count_++;
    }
    // required branch node already exists - use it
    else
      child_branch = static_cast<BranchNode*>(
          branch_arg->getChildPtr(buffer_selector_, child_idx));

    // recursively proceed with indexed child branch
    return createLeafRecursive(key_arg,
                               depth_mask_arg / 2,
                               child_branch,
                               return_leaf_arg,
                               parent_of_leaf_arg,
                               doNodeReset);
  }

  // branch childs are leaf nodes
  LeafNode* child_leaf;
  if (!branch_arg->hasChild(buffer_selector_, child_idx)) {
    // leaf node at child_idx does not exist

    // check if we can take copy a reference from previous buffer
    if (branch_arg->hasChild(!buffer_selector_, child_idx)) {

      OctreeNode* child_node = branch_arg->getChildPtr(!buffer_selector_, child_idx);
      if (child_node->getNodeType() == LEAF_NODE) {
        child_leaf = static_cast<LeafNode*>(child_node);
        child_leaf->getContainer() = LeafContainer(); // Clear contents of leaf
        branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
      }
      else {
        // depth has changed.. child in preceding buffer is a leaf node.
        deleteBranchChild(*branch_arg, !buffer_selector_, child_idx);
        child_leaf = createLeafChild(*branch_arg, child_idx);
      }
      leaf_count_++;
    }
    else {
      // if required leaf does not exist -> create it
      child_leaf = createLeafChild(*branch_arg, child_idx);
      leaf_count_++;
    }

    // return leaf node
    return_leaf_arg = child_leaf;
    parent_of_leaf_arg = branch_arg;
  }
  else {
    // leaf node already exist
    return_leaf_arg =
        static_cast<LeafNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));
    parent_of_leaf_arg = branch_arg;
  }

  return depth_mask_arg;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::findLeafRecursive(
    const OctreeKey& key_arg,
    uindex_t depth_mask_arg,
    BranchNode* branch_arg,
    LeafContainerT*& result_arg) const
{
  // return leaf node
  unsigned char child_idx;

  // find branch child from key
  child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  if (depth_mask_arg > 1) {
    // we have not reached maximum tree depth
    BranchNode* child_branch;
    child_branch =
        static_cast<BranchNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));

    if (child_branch)
      // recursively proceed with indexed child branch
      findLeafRecursive(key_arg, depth_mask_arg / 2, child_branch, result_arg);
  }
  else {
    // we reached leaf node level
    if (branch_arg->hasChild(buffer_selector_, child_idx)) {
      // return existing leaf node
      LeafNode* leaf_node =
          static_cast<LeafNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));
      result_arg = leaf_node->getContainerPtr();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
bool
Octree2BufBase<LeafContainerT, BranchContainerT>::deleteLeafRecursive(
    const OctreeKey& key_arg, uindex_t depth_mask_arg, BranchNode* branch_arg)
{
  // index to branch child
  unsigned char child_idx;
  // indicates if branch is empty and can be safely removed
  bool bNoChilds;

  // find branch child from key
  child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  if (depth_mask_arg > 1) {
    // we have not reached maximum tree depth
    BranchNode* child_branch;

    // next branch child on our path through the tree
    child_branch =
        static_cast<BranchNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));

    if (child_branch) {
      // recursively explore the indexed child branch
      bool bBranchOccupied =
          deleteLeafRecursive(key_arg, depth_mask_arg / 2, child_branch);

      if (!bBranchOccupied) {
        // child branch does not own any sub-child nodes anymore -> delete child branch
        deleteBranchChild(*branch_arg, buffer_selector_, child_idx);
        branch_count_--;
      }
    }
  }
  else {
    // our child is a leaf node -> delete it
    deleteBranchChild(*branch_arg, buffer_selector_, child_idx);
    leaf_count_--;
  }

  // check if current branch still owns childs
  bNoChilds = false;
  for (child_idx = 0; child_idx < 8; child_idx++) {
    bNoChilds = branch_arg->hasChild(buffer_selector_, child_idx);
    if (bNoChilds)
      break;
  }

  // return true if current branch can be deleted
  return (bNoChilds);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::serializeTreeRecursive(
    BranchNode* branch_arg,
    OctreeKey& key_arg,
    std::vector<char>* binary_tree_out_arg,
    typename std::vector<LeafContainerT*>* leaf_container_vector_arg,
    bool do_XOR_encoding_arg,
    bool new_leafs_filter_arg)
{
  if (binary_tree_out_arg) {
    // occupancy bit patterns of branch node  (current octree buffer)
    const char branch_bit_pattern_curr_buffer =
        getBranchBitPattern(*branch_arg, buffer_selector_);
    if (do_XOR_encoding_arg) {
      // occupancy bit patterns of branch node  (previous octree buffer)
      const char branch_bit_pattern_prev_buffer =
          getBranchBitPattern(*branch_arg, !buffer_selector_);
      // XOR of current and previous occupancy bit patterns
      const char node_XOR_bit_pattern =
          branch_bit_pattern_curr_buffer ^ branch_bit_pattern_prev_buffer;
      // write XOR bit pattern to output vector
      binary_tree_out_arg->push_back(node_XOR_bit_pattern);
    }
    else {
      // write bit pattern of current buffer to output vector
      binary_tree_out_arg->push_back(branch_bit_pattern_curr_buffer);
    }
  }

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
    if (branch_arg->hasChild(buffer_selector_, child_idx)) {
      // add current branch voxel to key
      key_arg.pushBranch(child_idx);

      OctreeNode* child_node = branch_arg->getChildPtr(buffer_selector_, child_idx);

      switch (child_node->getNodeType()) {
      case BRANCH_NODE: {
        // recursively proceed with indexed child branch
        serializeTreeRecursive(static_cast<BranchNode*>(child_node),
                               key_arg,
                               binary_tree_out_arg,
                               leaf_container_vector_arg,
                               do_XOR_encoding_arg,
                               new_leafs_filter_arg);
        break;
      }
      case LEAF_NODE: {
        LeafNode* child_leaf = static_cast<LeafNode*>(child_node);

        if (new_leafs_filter_arg) {
          if (!branch_arg->hasChild(!buffer_selector_, child_idx)) {
            if (leaf_container_vector_arg)
              leaf_container_vector_arg->push_back(child_leaf->getContainerPtr());

            serializeTreeCallback(**child_leaf, key_arg);
          }
        }
        else {

          if (leaf_container_vector_arg)
            leaf_container_vector_arg->push_back(child_leaf->getContainerPtr());

          serializeTreeCallback(**child_leaf, key_arg);
        }

        break;
      }
      default:
        break;
      }

      // pop current branch voxel from key
      key_arg.popBranch();
    }
    else if (branch_arg->hasChild(!buffer_selector_, child_idx)) {
      // delete branch, free memory
      deleteBranchChild(*branch_arg, !buffer_selector_, child_idx);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::deserializeTreeRecursive(
    BranchNode* branch_arg,
    uindex_t depth_mask_arg,
    OctreeKey& key_arg,
    typename std::vector<char>::const_iterator& binaryTreeIT_arg,
    typename std::vector<char>::const_iterator& binaryTreeIT_End_arg,
    typename std::vector<LeafContainerT*>::const_iterator* dataVectorIterator_arg,
    typename std::vector<LeafContainerT*>::const_iterator* dataVectorEndIterator_arg,
    bool branch_reset_arg,
    bool do_XOR_decoding_arg)
{

  // branch reset -> this branch has been taken from previous buffer
  if (branch_reset_arg) {
    // we can safely remove children references
    for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
      branch_arg->setChildPtr(buffer_selector_, child_idx, nullptr);
    }
  }

  if (binaryTreeIT_arg != binaryTreeIT_End_arg) {
    // read branch occupancy bit pattern from vector
    char nodeBits = *binaryTreeIT_arg++;

    // recover branch occupancy bit pattern
    char recoveredNodeBits;
    if (do_XOR_decoding_arg) {
      recoveredNodeBits =
          getBranchBitPattern(*branch_arg, !buffer_selector_) ^ nodeBits;
    }
    else {
      recoveredNodeBits = nodeBits;
    }

    // iterate over all children
    for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
      // if occupancy bit for child_idx is set..
      if (recoveredNodeBits & (1 << child_idx)) {
        // add current branch voxel to key
        key_arg.pushBranch(child_idx);

        if (depth_mask_arg > 1) {
          // we have not reached maximum tree depth

          bool doNodeReset = false;

          BranchNode* child_branch;

          // check if we find a branch node reference in previous buffer
          if (!branch_arg->hasChild(buffer_selector_, child_idx)) {

            if (branch_arg->hasChild(!buffer_selector_, child_idx)) {
              OctreeNode* child_node =
                  branch_arg->getChildPtr(!buffer_selector_, child_idx);

              if (child_node->getNodeType() == BRANCH_NODE) {
                child_branch = static_cast<BranchNode*>(child_node);
                branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
              }
              else {
                // depth has changed.. child in preceding buffer is a leaf node.
                deleteBranchChild(*branch_arg, !buffer_selector_, child_idx);
                child_branch = createBranchChild(*branch_arg, child_idx);
              }

              // take child branch from previous buffer
              doNodeReset = true; // reset the branch pointer array of stolen child node
            }
            else {
              // if required branch does not exist -> create it
              child_branch = createBranchChild(*branch_arg, child_idx);
            }

            branch_count_++;
          }
          else {
            // required branch node already exists - use it
            child_branch = static_cast<BranchNode*>(
                branch_arg->getChildPtr(buffer_selector_, child_idx));
          }

          // recursively proceed with indexed child branch
          deserializeTreeRecursive(child_branch,
                                   depth_mask_arg / 2,
                                   key_arg,
                                   binaryTreeIT_arg,
                                   binaryTreeIT_End_arg,
                                   dataVectorIterator_arg,
                                   dataVectorEndIterator_arg,
                                   doNodeReset,
                                   do_XOR_decoding_arg);
        }
        else {
          // branch childs are leaf nodes
          LeafNode* child_leaf;

          // check if we can take copy a reference pointer from previous buffer
          if (branch_arg->hasChild(!buffer_selector_, child_idx)) {
            // take child leaf node from previous buffer
            OctreeNode* child_node =
                branch_arg->getChildPtr(!buffer_selector_, child_idx);
            if (child_node->getNodeType() == LEAF_NODE) {
              child_leaf = static_cast<LeafNode*>(child_node);
              branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
            }
            else {
              // depth has changed.. child in preceding buffer is a leaf node.
              deleteBranchChild(*branch_arg, !buffer_selector_, child_idx);
              child_leaf = createLeafChild(*branch_arg, child_idx);
            }
          }
          else {
            // if required leaf does not exist -> create it
            child_leaf = createLeafChild(*branch_arg, child_idx);
          }

          // we reached leaf node level

          if (dataVectorIterator_arg &&
              (*dataVectorIterator_arg != *dataVectorEndIterator_arg)) {
            LeafContainerT& container = **child_leaf;
            container = ***dataVectorIterator_arg;
            ++*dataVectorIterator_arg;
          }

          leaf_count_++;

          // execute deserialization callback
          deserializeTreeCallback(**child_leaf, key_arg);
        }

        // pop current branch voxel from key
        key_arg.popBranch();
      }
      else if (branch_arg->hasChild(!buffer_selector_, child_idx)) {
        // remove old branch pointer information in current branch
        branch_arg->setChildPtr(buffer_selector_, child_idx, nullptr);

        // remove unused branches in previous buffer
        deleteBranchChild(*branch_arg, !buffer_selector_, child_idx);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT, typename BranchContainerT>
void
Octree2BufBase<LeafContainerT, BranchContainerT>::treeCleanUpRecursive(
    BranchNode* branch_arg)
{
  // occupancy bit pattern of branch node  (previous octree buffer)
  char occupied_children_bit_pattern_prev_buffer =
      getBranchBitPattern(*branch_arg, !buffer_selector_);

  // XOR of current and previous occupancy bit patterns
  char node_XOR_bit_pattern = getBranchXORBitPattern(*branch_arg);

  // bit pattern indicating unused octree nodes in previous branch
  char unused_branches_bit_pattern =
      node_XOR_bit_pattern & occupied_children_bit_pattern_prev_buffer;

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
    if (branch_arg->hasChild(buffer_selector_, child_idx)) {
      OctreeNode* child_node = branch_arg->getChildPtr(buffer_selector_, child_idx);

      switch (child_node->getNodeType()) {
      case BRANCH_NODE: {
        // recursively proceed with indexed child branch
        treeCleanUpRecursive(static_cast<BranchNode*>(child_node));
        break;
      }
      case LEAF_NODE:
        // leaf level - nothing to do..
        break;
      default:
        break;
      }
    }

    // check for unused branches in previous buffer
    if (unused_branches_bit_pattern & (1 << child_idx)) {
      // delete branch, free memory
      deleteBranchChild(*branch_arg, !buffer_selector_, child_idx);
    }
  }
}
} // namespace octree
} // namespace pcl

#define PCL_INSTANTIATE_Octree2BufBase(T)                                              \
  template class PCL_EXPORTS pcl::octree::Octree2BufBase<T>;

#endif
