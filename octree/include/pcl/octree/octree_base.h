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

#pragma once

#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_key.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/pcl_macros.h>

#include <vector>

namespace pcl {
namespace octree {

/** \brief Octree class
 * \note The tree depth defines the maximum amount of octree voxels / leaf nodes (should
 * be initially defined).
 * \note All leaf nodes are addressed by integer indices.
 * \note The tree depth equates to the bit length of the voxel indices.
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename LeafContainerT = index_t,
          typename BranchContainerT = OctreeContainerEmpty>
class OctreeBase {
public:
  using OctreeT = OctreeBase<LeafContainerT, BranchContainerT>;

  using BranchNode = OctreeBranchNode<BranchContainerT>;
  using LeafNode = OctreeLeafNode<LeafContainerT>;

  using BranchContainer = BranchContainerT;
  using LeafContainer = LeafContainerT;

protected:
  ///////////////////////////////////////////////////////////////////////
  // Members
  ///////////////////////////////////////////////////////////////////////

  /** \brief Amount of leaf nodes   **/
  std::size_t leaf_count_;

  /** \brief Amount of branch nodes   **/
  std::size_t branch_count_;

  /** \brief Pointer to root branch node of octree   **/
  BranchNode* root_node_;

  /** \brief Depth mask based on octree depth   **/
  uindex_t depth_mask_;

  /** \brief Octree depth */
  uindex_t octree_depth_;

  /** \brief Enable dynamic_depth **/
  bool dynamic_depth_enabled_;

  /** \brief key range */
  OctreeKey max_key_;

public:
  // iterators are friends
  friend class OctreeIteratorBase<OctreeT>;
  friend class OctreeDepthFirstIterator<OctreeT>;
  friend class OctreeBreadthFirstIterator<OctreeT>;
  friend class OctreeFixedDepthIterator<OctreeT>;
  friend class OctreeLeafNodeDepthFirstIterator<OctreeT>;
  friend class OctreeLeafNodeBreadthFirstIterator<OctreeT>;
  friend class OctreeIteratorBase<const OctreeT>;
  friend class OctreeDepthFirstIterator<const OctreeT>;
  friend class OctreeBreadthFirstIterator<const OctreeT>;
  friend class OctreeFixedDepthIterator<const OctreeT>;
  friend class OctreeLeafNodeDepthFirstIterator<const OctreeT>;
  friend class OctreeLeafNodeBreadthFirstIterator<const OctreeT>;

  // Octree default iterators
  using Iterator = OctreeDepthFirstIterator<OctreeT>;
  using ConstIterator = OctreeDepthFirstIterator<const OctreeT>;

  Iterator
  begin(uindex_t max_depth_arg = 0u)
  {
    return Iterator(this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  ConstIterator
  begin(uindex_t max_depth_arg = 0u) const
  {
    return ConstIterator(this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  ConstIterator
  cbegin(uindex_t max_depth_arg = 0u) const
  {
    return ConstIterator(this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  const Iterator
  end()
  {
    return Iterator(this, 0, nullptr);
  };

  const ConstIterator
  end() const
  {
    return ConstIterator(this, 0, nullptr);
  };

  const ConstIterator
  cend() const
  {
    return ConstIterator(this, 0, nullptr);
  };

  // Octree leaf node iterators
  // The previous deprecated names
  // LeafNodeIterator and ConstLeafNodeIterator are deprecated.
  // Please use LeafNodeDepthFirstIterator and ConstLeafNodeDepthFirstIterator instead.
  using LeafNodeIterator = OctreeLeafNodeDepthFirstIterator<OctreeT>;
  using ConstLeafNodeIterator = OctreeLeafNodeDepthFirstIterator<const OctreeT>;

  // The currently valid names
  using LeafNodeDepthFirstIterator = OctreeLeafNodeDepthFirstIterator<OctreeT>;
  using ConstLeafNodeDepthFirstIterator =
      OctreeLeafNodeDepthFirstIterator<const OctreeT>;

  LeafNodeDepthFirstIterator
  leaf_depth_begin(uindex_t max_depth_arg = 0u)
  {
    return LeafNodeDepthFirstIterator(
        this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  ConstLeafNodeDepthFirstIterator
  leaf_depth_begin(uindex_t max_depth_arg = 0u) const
  {
    return ConstLeafNodeDepthFirstIterator(
        this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  const LeafNodeDepthFirstIterator
  leaf_depth_end()
  {
    return LeafNodeDepthFirstIterator(this, 0, nullptr);
  };

  const ConstLeafNodeDepthFirstIterator
  leaf_depth_end() const
  {
    return ConstLeafNodeDepthFirstIterator(this, 0, nullptr);
  };

  // Octree depth-first iterators
  using DepthFirstIterator = OctreeDepthFirstIterator<OctreeT>;
  using ConstDepthFirstIterator = OctreeDepthFirstIterator<const OctreeT>;

  DepthFirstIterator
  depth_begin(uindex_t max_depth_arg = 0u)
  {
    return DepthFirstIterator(this,
                              max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  ConstDepthFirstIterator
  depth_begin(uindex_t max_depth_arg = 0u) const
  {
    return ConstDepthFirstIterator(this,
                                   max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  const DepthFirstIterator
  depth_end()
  {
    return DepthFirstIterator(this, 0, nullptr);
  };

  const ConstDepthFirstIterator
  depth_end() const
  {
    return ConstDepthFirstIterator(this, 0, nullptr);
  };

  // Octree breadth-first iterators
  using BreadthFirstIterator = OctreeBreadthFirstIterator<OctreeT>;
  using ConstBreadthFirstIterator = OctreeBreadthFirstIterator<const OctreeT>;

  BreadthFirstIterator
  breadth_begin(uindex_t max_depth_arg = 0u)
  {
    return BreadthFirstIterator(this,
                                max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  ConstBreadthFirstIterator
  breadth_begin(uindex_t max_depth_arg = 0u) const
  {
    return ConstBreadthFirstIterator(
        this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  const BreadthFirstIterator
  breadth_end()
  {
    return BreadthFirstIterator(this, 0, nullptr);
  };

  const ConstBreadthFirstIterator
  breadth_end() const
  {
    return ConstBreadthFirstIterator(this, 0, nullptr);
  };

  // Octree breadth iterators at a given depth
  using FixedDepthIterator = OctreeFixedDepthIterator<OctreeT>;
  using ConstFixedDepthIterator = OctreeFixedDepthIterator<const OctreeT>;

  FixedDepthIterator
  fixed_depth_begin(uindex_t fixed_depth_arg = 0u)
  {
    return FixedDepthIterator(this, fixed_depth_arg);
  };

  ConstFixedDepthIterator
  fixed_depth_begin(uindex_t fixed_depth_arg = 0u) const
  {
    return ConstFixedDepthIterator(this, fixed_depth_arg);
  };

  const FixedDepthIterator
  fixed_depth_end()
  {
    return FixedDepthIterator(this, 0, nullptr);
  };

  const ConstFixedDepthIterator
  fixed_depth_end() const
  {
    return ConstFixedDepthIterator(this, 0, nullptr);
  };

  // Octree leaf node iterators
  using LeafNodeBreadthFirstIterator = OctreeLeafNodeBreadthFirstIterator<OctreeT>;
  using ConstLeafNodeBreadthFirstIterator =
      OctreeLeafNodeBreadthFirstIterator<const OctreeT>;

  LeafNodeBreadthFirstIterator
  leaf_breadth_begin(uindex_t max_depth_arg = 0u)
  {
    return LeafNodeBreadthFirstIterator(
        this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  ConstLeafNodeBreadthFirstIterator
  leaf_breadth_begin(uindex_t max_depth_arg = 0u) const
  {
    return ConstLeafNodeBreadthFirstIterator(
        this, max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  const LeafNodeBreadthFirstIterator
  leaf_breadth_end()
  {
    return LeafNodeBreadthFirstIterator(this, 0, nullptr);
  };

  const ConstLeafNodeBreadthFirstIterator
  leaf_breadth_end() const
  {
    return ConstLeafNodeBreadthFirstIterator(this, 0, nullptr);
  };

  /** \brief Empty constructor. */
  OctreeBase();

  /** \brief Empty deconstructor. */
  virtual ~OctreeBase();

  /** \brief Copy constructor. */
  OctreeBase(const OctreeBase& source)
  : leaf_count_(source.leaf_count_)
  , branch_count_(source.branch_count_)
  , root_node_(new (BranchNode)(*(source.root_node_)))
  , depth_mask_(source.depth_mask_)
  , octree_depth_(source.octree_depth_)
  , dynamic_depth_enabled_(source.dynamic_depth_enabled_)
  , max_key_(source.max_key_)
  {}

  /** \brief Copy operator. */
  OctreeBase&
  operator=(const OctreeBase& source)
  {
    leaf_count_ = source.leaf_count_;
    branch_count_ = source.branch_count_;
    delete root_node_;

    root_node_ = new (BranchNode)(*(source.root_node_));
    depth_mask_ = source.depth_mask_;
    max_key_ = source.max_key_;
    octree_depth_ = source.octree_depth_;
    dynamic_depth_enabled_ = source.dynamic_depth_enabled_;
    return (*this);
  }

  /** \brief Set the maximum amount of voxels per dimension.
   * \param[in] max_voxel_index_arg maximum amount of voxels per dimension
   */
  void
  setMaxVoxelIndex(uindex_t max_voxel_index_arg);

  /** \brief Set the maximum depth of the octree.
   *  \param max_depth_arg: maximum depth of octree
   */
  void
  setTreeDepth(uindex_t max_depth_arg);

  /** \brief Get the maximum depth of the octree.
   *  \return depth_arg: maximum depth of octree
   */
  uindex_t
  getTreeDepth() const
  {
    return this->octree_depth_;
  }

  /** \brief Create new leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
   *  \note If leaf node already exist, this method returns the existing node
   *  \param idx_x_arg: index of leaf node in the X axis.
   *  \param idx_y_arg: index of leaf node in the Y axis.
   *  \param idx_z_arg: index of leaf node in the Z axis.
   *  \return pointer to new leaf node container.
   */
  LeafContainerT*
  createLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg);

  /** \brief Find leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
   *  \note If leaf node already exist, this method returns the existing node
   *  \param idx_x_arg: index of leaf node in the X axis.
   *  \param idx_y_arg: index of leaf node in the Y axis.
   *  \param idx_z_arg: index of leaf node in the Z axis.
   *  \return pointer to leaf node container if found, null pointer otherwise.
   */
  LeafContainerT*
  findLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg) const;

  /** \brief idx_x_arg for the existence of leaf node at (idx_x_arg, idx_y_arg,
   * idx_z_arg).
   * \param idx_x_arg: index of leaf node in the X axis.
   * \param idx_y_arg: index of leaf node in the Y axis.
   * \param idx_z_arg: index of leaf node in the Z axis.
   * \return "true" if leaf node search is successful, otherwise it returns "false".
   */
  bool
  existLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg) const;

  /** \brief Remove leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
   *  \param idx_x_arg: index of leaf node in the X axis.
   *  \param idx_y_arg: index of leaf node in the Y axis.
   *  \param idx_z_arg: index of leaf node in the Z axis.
   */
  void
  removeLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg);

  /** \brief Return the amount of existing leafs in the octree.
   *  \return amount of registered leaf nodes.
   */
  std::size_t
  getLeafCount() const
  {
    return leaf_count_;
  }

  /** \brief Return the amount of existing branch nodes in the octree.
   *  \return amount of branch nodes.
   */
  std::size_t
  getBranchCount() const
  {
    return branch_count_;
  }

  /** \brief Delete the octree structure and its leaf nodes.
   */
  void
  deleteTree();

  /** \brief Serialize octree into a binary output vector describing its branch node
   * structure.
   * \param binary_tree_out_arg: reference to output vector for writing binary tree
   * structure.
   */
  void
  serializeTree(std::vector<char>& binary_tree_out_arg) const;

  /** \brief Serialize octree into a binary output vector describing its branch node
   * structure and push all LeafContainerT elements stored in the octree to a vector.
   * \param binary_tree_out_arg: reference to output vector for writing binary tree
   * structure.
   * \param leaf_container_vector_arg: pointer to all LeafContainerT objects in the
   * octree
   */
  void
  serializeTree(std::vector<char>& binary_tree_out_arg,
                std::vector<LeafContainerT*>& leaf_container_vector_arg) const;

  /** \brief Outputs a vector of all LeafContainerT elements that are stored within the
   * octree leaf nodes.
   * \param leaf_container_vector_arg: pointers to LeafContainerT vector that receives a
   * copy of all LeafContainerT objects in the octree.
   */
  void
  serializeLeafs(std::vector<LeafContainerT*>& leaf_container_vector_arg);

  /** \brief Deserialize a binary octree description vector and create a corresponding
   * octree structure. Leaf nodes are initialized with getDataTByKey(..).
   * \param binary_tree_input_arg: reference to input vector for reading binary tree
   * structure.
   */
  void
  deserializeTree(std::vector<char>& binary_tree_input_arg);

  /** \brief Deserialize a binary octree description and create a corresponding octree
   * structure. Leaf nodes are initialized with LeafContainerT elements from the
   * dataVector.
   * \param binary_tree_input_arg: reference to input vector for reading binary tree
   * structure. \param leaf_container_vector_arg: pointer to container vector.
   */
  void
  deserializeTree(std::vector<char>& binary_tree_input_arg,
                  std::vector<LeafContainerT*>& leaf_container_vector_arg);

protected:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Protected octree methods based on octree keys
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Create a leaf node
   *  \param key_arg: octree key addressing a leaf node.
   *  \return pointer to leaf node
   */
  LeafContainerT*
  createLeaf(const OctreeKey& key_arg)
  {

    LeafNode* leaf_node = nullptr;
    BranchNode* leaf_node_parent;

    createLeafRecursive(key_arg, depth_mask_, root_node_, leaf_node, leaf_node_parent);

    LeafContainerT* ret = leaf_node->getContainerPtr();

    return ret;
  }

  /** \brief Find leaf node
   *  \param key_arg: octree key addressing a leaf node.
   *  \return pointer to leaf node. If leaf node is not found, this pointer returns 0.
   */
  LeafContainerT*
  findLeaf(const OctreeKey& key_arg) const
  {
    LeafContainerT* result = nullptr;
    findLeafRecursive(key_arg, depth_mask_, root_node_, result);
    return result;
  }

  /** \brief Check for existence of a leaf node in the octree
   *  \param key_arg: octree key addressing a leaf node.
   *  \return "true" if leaf node is found; "false" otherwise
   */
  bool
  existLeaf(const OctreeKey& key_arg) const
  {
    return (findLeaf(key_arg) != nullptr);
  }

  /** \brief Remove leaf node from octree
   *  \param key_arg: octree key addressing a leaf node.
   */
  void
  removeLeaf(const OctreeKey& key_arg)
  {
    if (key_arg <= max_key_)
      deleteLeafRecursive(key_arg, depth_mask_, root_node_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Branch node access functions
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Retrieve root node */
  OctreeNode*
  getRootNode() const
  {
    return this->root_node_;
  }

  /** \brief Check if branch is pointing to a particular child node
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \return "true" if pointer to child node exists; "false" otherwise
   */
  bool
  branchHasChild(const BranchNode& branch_arg, unsigned char child_idx_arg) const
  {
    // test occupancyByte for child existence
    return (branch_arg.getChildPtr(child_idx_arg) != nullptr);
  }

  /** \brief Retrieve a child node pointer for child node at child_idx.
   * \param branch_arg: reference to octree branch class
   * \param child_idx_arg: index to child node
   * \return pointer to octree child node class
   */
  OctreeNode*
  getBranchChildPtr(const BranchNode& branch_arg, unsigned char child_idx_arg) const
  {
    return branch_arg.getChildPtr(child_idx_arg);
  }

  /** \brief Assign new child node to branch
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \param new_child_arg: pointer to new child node
   */
  void
  setBranchChildPtr(BranchNode& branch_arg,
                    unsigned char child_idx_arg,
                    OctreeNode* new_child_arg)
  {
    branch_arg[child_idx_arg] = new_child_arg;
  }

  /** \brief Generate bit pattern reflecting the existence of child node pointers
   *  \param branch_arg: reference to octree branch class
   *  \return a single byte with 8 bits of child node information
   */
  char
  getBranchBitPattern(const BranchNode& branch_arg) const
  {
    char node_bits;

    // create bit pattern
    node_bits = 0;
    for (unsigned char i = 0; i < 8; i++) {
      const OctreeNode* child = branch_arg.getChildPtr(i);
      node_bits |= static_cast<char>((!!child) << i);
    }

    return (node_bits);
  }

  /** \brief Delete child node and all its subchilds from octree
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   */
  void
  deleteBranchChild(BranchNode& branch_arg, unsigned char child_idx_arg)
  {
    if (branch_arg.hasChild(child_idx_arg)) {
      OctreeNode* branch_child = branch_arg[child_idx_arg];

      switch (branch_child->getNodeType()) {
      case BRANCH_NODE: {
        // free child branch recursively
        deleteBranch(*static_cast<BranchNode*>(branch_child));
        // delete branch node
        delete branch_child;
      } break;

      case LEAF_NODE: {
        // delete leaf node
        delete branch_child;
        break;
      }
      default:
        break;
      }

      // set branch child pointer to 0
      branch_arg[child_idx_arg] = nullptr;
    }
  }

  /** \brief Delete branch and all its subchilds from octree
   *  \param branch_arg: reference to octree branch class
   */
  void
  deleteBranch(BranchNode& branch_arg)
  {
    // delete all branch node children
    for (char i = 0; i < 8; i++)
      deleteBranchChild(branch_arg, i);
  }

  /** \brief Create and add a new branch child to a branch class
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \return pointer of new branch child to this reference
   */
  BranchNode*
  createBranchChild(BranchNode& branch_arg, unsigned char child_idx_arg)
  {
    auto* new_branch_child = new BranchNode();
    branch_arg[child_idx_arg] = static_cast<OctreeNode*>(new_branch_child);

    return new_branch_child;
  }

  /** \brief Create and add a new leaf child to a branch class
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \return pointer of new leaf child to this reference
   */
  LeafNode*
  createLeafChild(BranchNode& branch_arg, unsigned char child_idx_arg)
  {
    auto* new_leaf_child = new LeafNode();
    branch_arg[child_idx_arg] = static_cast<OctreeNode*>(new_leaf_child);

    return new_leaf_child;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Recursive octree methods
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Create a leaf node at octree key. If leaf node does already exist, it is
   * returned.
   * \param key_arg: reference to an octree key
   * \param depth_mask_arg: depth mask used for octree key analysis and for branch depth
   * indicator
   * \param branch_arg: current branch node
   * \param return_leaf_arg: return pointer to leaf node
   * \param parent_of_leaf_arg: return pointer to parent of leaf node
   * \return depth mask at which leaf node was created
   **/
  uindex_t
  createLeafRecursive(const OctreeKey& key_arg,
                      uindex_t depth_mask_arg,
                      BranchNode* branch_arg,
                      LeafNode*& return_leaf_arg,
                      BranchNode*& parent_of_leaf_arg);

  /** \brief Recursively search for a given leaf node and return a pointer.
   *  \note  If leaf node does not exist, a 0 pointer is returned.
   *  \param key_arg: reference to an octree key
   *  \param depth_mask_arg: depth mask used for octree key analysis and for branch
   * depth indicator
   * \param branch_arg: current branch node
   * \param result_arg: pointer to leaf node class
   **/
  void
  findLeafRecursive(const OctreeKey& key_arg,
                    uindex_t depth_mask_arg,
                    BranchNode* branch_arg,
                    LeafContainerT*& result_arg) const;

  /** \brief Recursively search and delete leaf node
   *  \param key_arg: reference to an octree key
   *  \param depth_mask_arg: depth mask used for octree key analysis and branch depth
   * indicator
   * \param branch_arg: current branch node
   * \return "true" if current branch contains child(ren); "false" otherwise. If it's
   * true, current branch cannot be deleted.
   **/
  bool
  deleteLeafRecursive(const OctreeKey& key_arg,
                      uindex_t depth_mask_arg,
                      BranchNode* branch_arg);

  /** \brief Recursively explore the octree and output binary octree description
   * together with a vector of leaf node LeafContainerTs.
   * \param branch_arg: current branch node
   * \param key_arg: reference to an octree key
   * \param binary_tree_out_arg: binary output vector
   * \param leaf_container_vector_arg: writes LeafContainerT pointers to this
   *LeafContainerT* vector.
   **/
  void
  serializeTreeRecursive(
      const BranchNode* branch_arg,
      OctreeKey& key_arg,
      std::vector<char>* binary_tree_out_arg,
      typename std::vector<LeafContainerT*>* leaf_container_vector_arg) const;

  /** \brief Recursive method for deserializing octree structure
   *  \param branch_arg: current branch node
   *  \param depth_mask_arg: depth mask used for octree key analysis and branch depth
   * indicator
   * \param key_arg: reference to an octree key
   * \param binary_tree_input_it_arg: iterator to binary input vector
   * \param binary_tree_input_it_end_arg: end iterator of binary input vector
   * \param leaf_container_vector_it_arg: iterator pointing to current LeafContainerT
   * object to be added to a leaf node
   * \param leaf_container_vector_it_end_arg: iterator pointing to last object in
   * LeafContainerT input vector.
   **/
  void
  deserializeTreeRecursive(
      BranchNode* branch_arg,
      uindex_t depth_mask_arg,
      OctreeKey& key_arg,
      typename std::vector<char>::const_iterator& binary_tree_input_it_arg,
      typename std::vector<char>::const_iterator& binary_tree_input_it_end_arg,
      typename std::vector<LeafContainerT*>::const_iterator*
          leaf_container_vector_it_arg,
      typename std::vector<LeafContainerT*>::const_iterator*
          leaf_container_vector_it_end_arg);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serialization callbacks
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Callback executed for every leaf node during serialization
   **/
  virtual void
  serializeTreeCallback(LeafContainerT&, const OctreeKey&) const
  {}

  /** \brief Callback executed for every leaf node during deserialization
   **/
  virtual void
  deserializeTreeCallback(LeafContainerT&, const OctreeKey&)
  {}

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Helpers
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Test if octree is able to dynamically change its depth. This is required
   *for adaptive bounding box adjustment.
   * \return "true"
   **/
  bool
  octreeCanResize() const
  {
    return (true);
  }
};
} // namespace octree
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_base.hpp>
#endif
