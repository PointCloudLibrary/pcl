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

#include <array>
#include <vector>

namespace pcl {
namespace octree {

template <typename ContainerT>
class BufferedBranchNode : public OctreeNode {

public:
  /** \brief Empty constructor. */
  BufferedBranchNode() : OctreeNode() { reset(); }

  /** \brief Copy constructor. */
  BufferedBranchNode(const BufferedBranchNode& source) : OctreeNode()
  {
    *this = source;
  }

  /** \brief Copy operator. */
  inline BufferedBranchNode&
  operator=(const BufferedBranchNode& source_arg)
  {
    child_node_array_ = {};
    for (unsigned char b = 0; b < 2; ++b) {
      for (unsigned char i = 0; i < 8; ++i) {
        if (source_arg.child_node_array_[b][i]) {
          child_node_array_[b][i] = source_arg.child_node_array_[b][i]->deepCopy();
        }
      }
    }
    return (*this);
  }

  /** \brief Empty constructor. */
  ~BufferedBranchNode() override = default;

  /** \brief Method to perform a deep copy of the octree */
  BufferedBranchNode*
  deepCopy() const override
  {
    return new BufferedBranchNode(*this);
  }

  /** \brief Get child pointer in current branch node
   *  \param buffer_arg: buffer selector
   *  \param index_arg: index of child in node
   *  \return pointer to child node
   */
  inline OctreeNode*
  getChildPtr(unsigned char buffer_arg, unsigned char index_arg) const
  {
    assert((buffer_arg < 2) && (index_arg < 8));
    return child_node_array_[buffer_arg][index_arg];
  }

  /** \brief Set child pointer in current branch node
   *  \param buffer_arg: buffer selector
   *  \param index_arg: index of child in node
   *  \param newNode_arg: pointer to new child node
   */
  inline void
  setChildPtr(unsigned char buffer_arg,
              unsigned char index_arg,
              OctreeNode* newNode_arg)
  {
    assert((buffer_arg < 2) && (index_arg < 8));
    child_node_array_[buffer_arg][index_arg] = newNode_arg;
  }

  /** \brief Check if branch is pointing to a particular child node
   *  \param buffer_arg: buffer selector
   *  \param index_arg: index of child in node
   *  \return "true" if pointer to child node exists; "false" otherwise
   */
  inline bool
  hasChild(unsigned char buffer_arg, unsigned char index_arg) const
  {
    assert((buffer_arg < 2) && (index_arg < 8));
    return (child_node_array_[buffer_arg][index_arg] != nullptr);
  }

  /** \brief Get the type of octree node. Returns LEAVE_NODE type */
  node_type_t
  getNodeType() const override
  {
    return BRANCH_NODE;
  }

  /** \brief Reset branch node container for every branch buffer. */
  inline void
  reset()
  {
    child_node_array_ = {};
  }

  /** \brief Get const pointer to container */
  const ContainerT*
  operator->() const
  {
    return &container_;
  }

  /** \brief Get pointer to container */
  ContainerT*
  operator->()
  {
    return &container_;
  }

  /** \brief Get const reference to container */
  const ContainerT&
  operator*() const
  {
    return container_;
  }

  /** \brief Get reference to container */
  ContainerT&
  operator*()
  {
    return container_;
  }

  /** \brief Get const reference to container */
  const ContainerT&
  getContainer() const
  {
    return container_;
  }

  /** \brief Get reference to container */
  ContainerT&
  getContainer()
  {
    return container_;
  }

  /** \brief Get const pointer to container */
  const ContainerT*
  getContainerPtr() const
  {
    return &container_;
  }

  /** \brief Get pointer to container */
  ContainerT*
  getContainerPtr()
  {
    return &container_;
  }

protected:
  ContainerT container_;

  template <typename T, std::size_t ROW, std::size_t COL>
  using OctreeMatrix = std::array<std::array<T, COL>, ROW>;

  OctreeMatrix<OctreeNode*, 2, 8> child_node_array_{};
};

/** \brief @b Octree double buffer class
 *
 * This octree implementation keeps two separate octree structures in memory
 * which allows for differentially comparison of the octree structures (change
 * detection, differential encoding).
 * \note The tree depth defines the maximum amount of octree voxels / leaf nodes (should
 * be initially defined).
 * \note All leaf nodes are addressed by integer indices.
 * \note The tree depth equates to the bit length of the voxel indices.
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename LeafContainerT = index_t,
          typename BranchContainerT = OctreeContainerEmpty>
class Octree2BufBase {

public:
  using OctreeT = Octree2BufBase<LeafContainerT, BranchContainerT>;

  // iterators are friends
  friend class OctreeIteratorBase<OctreeT>;
  friend class OctreeDepthFirstIterator<OctreeT>;
  friend class OctreeBreadthFirstIterator<OctreeT>;
  friend class OctreeLeafNodeDepthFirstIterator<OctreeT>;
  friend class OctreeLeafNodeBreadthFirstIterator<OctreeT>;

  using BranchNode = BufferedBranchNode<BranchContainerT>;
  using LeafNode = OctreeLeafNode<LeafContainerT>;

  using BranchContainer = BranchContainerT;
  using LeafContainer = LeafContainerT;

  // Octree default iterators
  using Iterator = OctreeDepthFirstIterator<OctreeT>;
  using ConstIterator = const OctreeDepthFirstIterator<OctreeT>;
  Iterator
  begin(uindex_t max_depth_arg = 0)
  {
    return Iterator(this, max_depth_arg);
  };
  const Iterator
  end()
  {
    return Iterator();
  };

  // Octree leaf node iterators
  // The previous deprecated names
  // LeafNodeIterator and ConstLeafNodeIterator are deprecated.
  // Please use LeafNodeDepthFirstIterator and ConstLeafNodeDepthFirstIterator instead.
  using LeafNodeIterator = OctreeLeafNodeDepthFirstIterator<OctreeT>;
  using ConstLeafNodeIterator = const OctreeLeafNodeDepthFirstIterator<OctreeT>;

  // The currently valide names
  using LeafNodeDepthFirstIterator = OctreeLeafNodeDepthFirstIterator<OctreeT>;
  using ConstLeafNodeDepthFirstIterator =
      const OctreeLeafNodeDepthFirstIterator<OctreeT>;
  LeafNodeDepthFirstIterator
  leaf_depth_begin(uindex_t max_depth_arg = 0)
  {
    return LeafNodeDepthFirstIterator(this, max_depth_arg);
  };

  const LeafNodeDepthFirstIterator
  leaf_depth_end()
  {
    return LeafNodeDepthFirstIterator();
  };

  // Octree depth-first iterators
  using DepthFirstIterator = OctreeDepthFirstIterator<OctreeT>;
  using ConstDepthFirstIterator = const OctreeDepthFirstIterator<OctreeT>;
  DepthFirstIterator
  depth_begin(uindex_t maxDepth_arg = 0)
  {
    return DepthFirstIterator(this, maxDepth_arg);
  };
  const DepthFirstIterator
  depth_end()
  {
    return DepthFirstIterator();
  };

  // Octree breadth-first iterators
  using BreadthFirstIterator = OctreeBreadthFirstIterator<OctreeT>;
  using ConstBreadthFirstIterator = const OctreeBreadthFirstIterator<OctreeT>;
  BreadthFirstIterator
  breadth_begin(uindex_t max_depth_arg = 0)
  {
    return BreadthFirstIterator(this, max_depth_arg);
  };
  const BreadthFirstIterator
  breadth_end()
  {
    return BreadthFirstIterator();
  };

  // Octree leaf node iterators
  using LeafNodeBreadthIterator = OctreeLeafNodeBreadthFirstIterator<OctreeT>;
  using ConstLeafNodeBreadthIterator =
      const OctreeLeafNodeBreadthFirstIterator<OctreeT>;

  LeafNodeBreadthIterator
  leaf_breadth_begin(uindex_t max_depth_arg = 0u)
  {
    return LeafNodeBreadthIterator(this,
                                   max_depth_arg ? max_depth_arg : this->octree_depth_);
  };

  const LeafNodeBreadthIterator
  leaf_breadth_end()
  {
    return LeafNodeBreadthIterator(this, 0, nullptr);
  };

  /** \brief Empty constructor. */
  Octree2BufBase();

  /** \brief Empty deconstructor. */
  virtual ~Octree2BufBase();

  /** \brief Copy constructor. */
  Octree2BufBase(const Octree2BufBase& source)
  : leaf_count_(source.leaf_count_)
  , branch_count_(source.branch_count_)
  , root_node_(new (BranchNode)(*(source.root_node_)))
  , depth_mask_(source.depth_mask_)
  , max_key_(source.max_key_)
  , buffer_selector_(source.buffer_selector_)
  , tree_dirty_flag_(source.tree_dirty_flag_)
  , octree_depth_(source.octree_depth_)
  , dynamic_depth_enabled_(source.dynamic_depth_enabled_)
  {}

  /** \brief Copy constructor. */
  inline Octree2BufBase&
  operator=(const Octree2BufBase& source)
  {
    leaf_count_ = source.leaf_count_;
    branch_count_ = source.branch_count_;
    root_node_ = new (BranchNode)(*(source.root_node_));
    depth_mask_ = source.depth_mask_;
    max_key_ = source.max_key_;
    buffer_selector_ = source.buffer_selector_;
    tree_dirty_flag_ = source.tree_dirty_flag_;
    octree_depth_ = source.octree_depth_;
    dynamic_depth_enabled_ = source.dynamic_depth_enabled_;
    return (*this);
  }

  /** \brief Set the maximum amount of voxels per dimension.
   *  \param max_voxel_index_arg: maximum amount of voxels per dimension
   */
  void
  setMaxVoxelIndex(uindex_t max_voxel_index_arg);

  /** \brief Set the maximum depth of the octree.
   *  \param depth_arg: maximum depth of octree
   */
  void
  setTreeDepth(uindex_t depth_arg);

  /** \brief Get the maximum depth of the octree.
   *  \return depth_arg: maximum depth of octree
   */
  inline uindex_t
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
  findLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg);

  /** \brief Check for the existence of leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
   *  \param idx_x_arg: index of leaf node in the X axis.
   *  \param idx_y_arg: index of leaf node in the Y axis.
   *  \param idx_z_arg: index of leaf node in the Z axis.
   *  \return "true" if leaf node search is successful, otherwise it returns "false".
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
  inline std::size_t
  getLeafCount() const
  {
    return (leaf_count_);
  }

  /** \brief Return the amount of existing branches in the octree.
   *  \return amount of branch nodes.
   */
  inline std::size_t
  getBranchCount() const
  {
    return (branch_count_);
  }

  /** \brief Delete the octree structure and its leaf nodes.
   */
  void
  deleteTree();

  /** \brief Delete octree structure of previous buffer. */
  inline void
  deletePreviousBuffer()
  {
    treeCleanUpRecursive(root_node_);
  }

  /** \brief Delete the octree structure in the current buffer. */
  inline void
  deleteCurrentBuffer()
  {
    buffer_selector_ = !buffer_selector_;
    treeCleanUpRecursive(root_node_);
    leaf_count_ = 0;
  }

  /** \brief Switch buffers and reset current octree structure. */
  void
  switchBuffers();

  /** \brief Serialize octree into a binary output vector describing its branch node
   * structure.
   * \param binary_tree_out_arg: reference to output vector for writing binary
   * tree structure.
   * \param do_XOR_encoding_arg: select if binary tree structure should be generated
   * based on current octree (false) of based on a XOR comparison between current and
   * previous octree
   **/
  void
  serializeTree(std::vector<char>& binary_tree_out_arg,
                bool do_XOR_encoding_arg = false);

  /** \brief Serialize octree into a binary output vector describing its branch node
   * structure and and push all DataT elements stored in the octree to a vector.
   * \param binary_tree_out_arg: reference to output vector for writing binary tree
   * structure.
   * \param leaf_container_vector_arg: pointer to all LeafContainerT objects in the
   * octree
   * \param do_XOR_encoding_arg: select if binary tree structure should be
   * generated based on current octree (false) of based on a XOR comparison between
   * current and previous octree
   **/
  void
  serializeTree(std::vector<char>& binary_tree_out_arg,
                std::vector<LeafContainerT*>& leaf_container_vector_arg,
                bool do_XOR_encoding_arg = false);

  /** \brief Outputs a vector of all DataT elements that are stored within the octree
   * leaf nodes.
   * \param leaf_container_vector_arg: vector of pointers to all LeafContainerT objects
   * in the octree
   */
  void
  serializeLeafs(std::vector<LeafContainerT*>& leaf_container_vector_arg);

  /** \brief Outputs a vector of all DataT elements from leaf nodes, that do not exist
   * in the previous octree buffer.
   * \param leaf_container_vector_arg: vector of pointers to all LeafContainerT objects
   * in the octree
   */
  void
  serializeNewLeafs(std::vector<LeafContainerT*>& leaf_container_vector_arg);

  /** \brief Deserialize a binary octree description vector and create a corresponding
   * octree structure. Leaf nodes are initialized with getDataTByKey(..).
   * \param binary_tree_in_arg: reference to input vector for reading binary tree
   * structure.
   * \param do_XOR_decoding_arg: select if binary tree structure is based on current
   * octree (false) of based on a XOR comparison between current and previous octree
   */
  void
  deserializeTree(std::vector<char>& binary_tree_in_arg,
                  bool do_XOR_decoding_arg = false);

  /** \brief Deserialize a binary octree description and create a corresponding octree
   * structure. Leaf nodes are initialized with DataT elements from the dataVector.
   * \param binary_tree_in_arg: reference to inpvectoream for reading binary tree
   * structure.
   * \param leaf_container_vector_arg: vector of pointers to all LeafContainerT objects
   * in the octree
   * \param do_XOR_decoding_arg: select if binary tree structure is based on current
   * octree (false) of based on a XOR comparison between current and previous octree
   */
  void
  deserializeTree(std::vector<char>& binary_tree_in_arg,
                  std::vector<LeafContainerT*>& leaf_container_vector_arg,
                  bool do_XOR_decoding_arg = false);

protected:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Protected octree methods based on octree keys
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Retrieve root node */
  OctreeNode*
  getRootNode() const
  {
    return (this->root_node_);
  }

  /** \brief Find leaf node
   *  \param key_arg: octree key addressing a leaf node.
   *  \return pointer to leaf container. If leaf node is not found, this pointer returns
   * 0.
   */
  inline LeafContainerT*
  findLeaf(const OctreeKey& key_arg) const
  {
    LeafContainerT* result = nullptr;
    findLeafRecursive(key_arg, depth_mask_, root_node_, result);
    return result;
  }

  /** \brief Create a leaf node.
   *  \note If the leaf node at the given octree node does not exist, it will be created
   * and added to the tree.
   * \param key_arg: octree key addressing a leaf node.
   * \return pointer to an existing or created leaf container.
   */
  inline LeafContainerT*
  createLeaf(const OctreeKey& key_arg)
  {
    LeafNode* leaf_node;
    BranchNode* leaf_node_parent;

    createLeafRecursive(
        key_arg, depth_mask_, root_node_, leaf_node, leaf_node_parent, false);

    LeafContainerT* ret = leaf_node->getContainerPtr();

    return ret;
  }

  /** \brief Check if leaf doesn't exist in the octree
   *  \param key_arg: octree key addressing a leaf node.
   *  \return "true" if leaf node is found; "false" otherwise
   */
  inline bool
  existLeaf(const OctreeKey& key_arg) const
  {
    return (findLeaf(key_arg) != nullptr);
  }

  /** \brief Remove leaf node from octree
   *  \param key_arg: octree key addressing a leaf node.
   */
  inline void
  removeLeaf(const OctreeKey& key_arg)
  {
    if (key_arg <= max_key_) {
      deleteLeafRecursive(key_arg, depth_mask_, root_node_);

      // we changed the octree structure -> dirty
      tree_dirty_flag_ = true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Branch node accessor inline functions
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Check if branch is pointing to a particular child node
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \return "true" if pointer to child node exists; "false" otherwise
   */
  inline bool
  branchHasChild(const BranchNode& branch_arg, unsigned char child_idx_arg) const
  {
    // test occupancyByte for child existence
    return (branch_arg.getChildPtr(buffer_selector_, child_idx_arg) != nullptr);
  }

  /** \brief Retrieve a child node pointer for child node at child_idx.
   * \param branch_arg: reference to octree branch class
   * \param child_idx_arg: index to child node
   * \return pointer to octree child node class
   */
  inline OctreeNode*
  getBranchChildPtr(const BranchNode& branch_arg, unsigned char child_idx_arg) const
  {
    return branch_arg.getChildPtr(buffer_selector_, child_idx_arg);
  }

  /** \brief Assign new child node to branch
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \param new_child_arg: pointer to new child node
   */
  inline void
  setBranchChildPtr(BranchNode& branch_arg,
                    unsigned char child_idx_arg,
                    OctreeNode* new_child_arg)
  {
    branch_arg.setChildPtr(buffer_selector_, child_idx_arg, new_child_arg);
  }

  /** \brief Generate bit pattern reflecting the existence of child node pointers for
   * current buffer
   * \param branch_arg: reference to octree branch class
   * \return a single byte with 8 bits of child node information
   */
  inline char
  getBranchBitPattern(const BranchNode& branch_arg) const
  {
    char node_bits;

    // create bit pattern
    node_bits = 0;
    for (unsigned char i = 0; i < 8; i++) {
      const OctreeNode* child = branch_arg.getChildPtr(buffer_selector_, i);
      node_bits |= static_cast<char>((!!child) << i);
    }

    return (node_bits);
  }

  /** \brief Generate bit pattern reflecting the existence of child node pointers in
   * specific buffer
   * \param branch_arg: reference to octree branch class
   * \param bufferSelector_arg: buffer selector
   * \return a single byte with 8 bits of child node information
   */
  inline char
  getBranchBitPattern(const BranchNode& branch_arg,
                      unsigned char bufferSelector_arg) const
  {
    char node_bits;

    // create bit pattern
    node_bits = 0;
    for (unsigned char i = 0; i < 8; i++) {
      const OctreeNode* child = branch_arg.getChildPtr(bufferSelector_arg, i);
      node_bits |= static_cast<char>((!!child) << i);
    }

    return (node_bits);
  }

  /** \brief Generate XOR bit pattern reflecting differences between the two octree
   * buffers
   * \param branch_arg: reference to octree branch class
   * \return a single byte with 8 bits of child node XOR difference information
   */
  inline char
  getBranchXORBitPattern(const BranchNode& branch_arg) const
  {
    char node_bits[2];

    // create bit pattern for both buffers
    node_bits[0] = node_bits[1] = 0;

    for (unsigned char i = 0; i < 8; i++) {
      const OctreeNode* childA = branch_arg.getChildPtr(0, i);
      const OctreeNode* childB = branch_arg.getChildPtr(1, i);

      node_bits[0] |= static_cast<char>((!!childA) << i);
      node_bits[1] |= static_cast<char>((!!childB) << i);
    }

    return node_bits[0] ^ node_bits[1];
  }

  /** \brief Test if branch changed between previous and current buffer
   *  \param branch_arg: reference to octree branch class
   *  \return "true", if child node information differs between current and previous
   * octree buffer
   */
  inline bool
  hasBranchChanges(const BranchNode& branch_arg) const
  {
    return (getBranchXORBitPattern(branch_arg) > 0);
  }

  /** \brief Delete child node and all its subchilds from octree in specific buffer
   *  \param branch_arg: reference to octree branch class
   *  \param buffer_selector_arg: buffer selector
   *  \param child_idx_arg: index to child node
   */
  inline void
  deleteBranchChild(BranchNode& branch_arg,
                    unsigned char buffer_selector_arg,
                    unsigned char child_idx_arg)
  {
    if (branch_arg.hasChild(buffer_selector_arg, child_idx_arg)) {
      OctreeNode* branchChild =
          branch_arg.getChildPtr(buffer_selector_arg, child_idx_arg);

      switch (branchChild->getNodeType()) {
      case BRANCH_NODE: {
        // free child branch recursively
        deleteBranch(*static_cast<BranchNode*>(branchChild));

        // delete unused branch
        delete (branchChild);
        break;
      }

      case LEAF_NODE: {
        // push unused leaf to branch pool
        delete (branchChild);
        break;
      }
      default:
        break;
      }

      // set branch child pointer to 0
      branch_arg.setChildPtr(buffer_selector_arg, child_idx_arg, nullptr);
    }
  }

  /** \brief Delete child node and all its subchilds from octree in current buffer
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   */
  inline void
  deleteBranchChild(BranchNode& branch_arg, unsigned char child_idx_arg)
  {
    deleteBranchChild(branch_arg, buffer_selector_, child_idx_arg);
  }

  /** \brief Delete branch and all its subchilds from octree (both buffers)
   *  \param branch_arg: reference to octree branch class
   */
  inline void
  deleteBranch(BranchNode& branch_arg)
  {
    // delete all branch node children
    for (char i = 0; i < 8; i++) {

      if (branch_arg.getChildPtr(0, i) == branch_arg.getChildPtr(1, i)) {
        // reference was copied - there is only one child instance to be deleted
        deleteBranchChild(branch_arg, 0, i);

        // remove pointers from both buffers
        branch_arg.setChildPtr(0, i, nullptr);
        branch_arg.setChildPtr(1, i, nullptr);
      }
      else {
        deleteBranchChild(branch_arg, 0, i);
        deleteBranchChild(branch_arg, 1, i);
      }
    }
  }

  /** \brief Fetch and add a new branch child to a branch class in current buffer
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \return pointer of new branch child to this reference
   */
  inline BranchNode*
  createBranchChild(BranchNode& branch_arg, unsigned char child_idx_arg)
  {
    auto* new_branch_child = new BranchNode();

    branch_arg.setChildPtr(
        buffer_selector_, child_idx_arg, static_cast<OctreeNode*>(new_branch_child));

    return new_branch_child;
  }

  /** \brief Fetch and add a new leaf child to a branch class
   *  \param branch_arg: reference to octree branch class
   *  \param child_idx_arg: index to child node
   *  \return pointer of new leaf child to this reference
   */
  inline LeafNode*
  createLeafChild(BranchNode& branch_arg, unsigned char child_idx_arg)
  {
    auto* new_leaf_child = new LeafNode();

    branch_arg.setChildPtr(buffer_selector_, child_idx_arg, new_leaf_child);

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
   * \param return_leaf_arg: return pointer to leaf container
   * \param parent_of_leaf_arg: return pointer to parent of leaf node
   * \param branch_reset_arg: Reset pointer array of current branch
   * \return depth mask at which leaf node was created/found
   **/
  uindex_t
  createLeafRecursive(const OctreeKey& key_arg,
                      uindex_t depth_mask_arg,
                      BranchNode* branch_arg,
                      LeafNode*& return_leaf_arg,
                      BranchNode*& parent_of_leaf_arg,
                      bool branch_reset_arg = false);

  /** \brief Recursively search for a given leaf node and return a pointer.
   *  \note  If leaf node does not exist, a 0 pointer is returned.
   *  \param key_arg: reference to an octree key
   *  \param depth_mask_arg: depth mask used for octree key analysis and for branch
   *  depth indicator
   *  \param branch_arg: current branch node
   *  \param result_arg: pointer to leaf container class
   **/
  void
  findLeafRecursive(const OctreeKey& key_arg,
                    uindex_t depth_mask_arg,
                    BranchNode* branch_arg,
                    LeafContainerT*& result_arg) const;

  /** \brief Recursively search and delete leaf node
   *  \param key_arg: reference to an octree key
   *  \param depth_mask_arg: depth mask used for octree key analysis and branch depth
   *  indicator
   *  \param branch_arg: current branch node
   *  \return "true" if branch does not contain any childs; "false" otherwise. This
   *  indicates if current branch can be deleted.
   **/
  bool
  deleteLeafRecursive(const OctreeKey& key_arg,
                      uindex_t depth_mask_arg,
                      BranchNode* branch_arg);

  /** \brief Recursively explore the octree and output binary octree description
   * together with a vector of leaf node DataT content.
   * \param branch_arg: current branch node
   * \param key_arg: reference to an octree key
   * \param binary_tree_out_arg: binary output vector
   * \param leaf_container_vector_arg: vector to return pointers to all leaf container
   * in the tree.
   * \param do_XOR_encoding_arg: select if binary tree structure should be generated
   * based on current octree (false) of based on a XOR comparison between current and
   * previous octree
   * \param new_leafs_filter_arg: execute callback only for leaf nodes that did not
   * exist in preceding buffer
   **/
  void
  serializeTreeRecursive(
      BranchNode* branch_arg,
      OctreeKey& key_arg,
      std::vector<char>* binary_tree_out_arg,
      typename std::vector<LeafContainerT*>* leaf_container_vector_arg,
      bool do_XOR_encoding_arg = false,
      bool new_leafs_filter_arg = false);

  /** \brief Rebuild an octree based on binary XOR octree description and DataT objects
   * for leaf node initialization.
   * \param branch_arg: current branch node
   * \param depth_mask_arg: depth mask used for octree key analysis and branch depth
   * indicator
   * \param key_arg: reference to an octree key
   * \param binary_tree_in_it_arg iterator of binary input data
   * \param binary_tree_in_it_end_arg
   * \param leaf_container_vector_it_arg: iterator pointing to leaf container pointers
   * to be added to a leaf node
   * \param leaf_container_vector_it_end_arg: iterator pointing to leaf container
   * pointers pointing to last object in input container.
   * \param branch_reset_arg: Reset pointer array of current branch
   * \param do_XOR_decoding_arg: select if binary tree structure is based on current
   * octree (false) of based on a XOR comparison between current and previous octree
   **/
  void
  deserializeTreeRecursive(
      BranchNode* branch_arg,
      uindex_t depth_mask_arg,
      OctreeKey& key_arg,
      typename std::vector<char>::const_iterator& binary_tree_in_it_arg,
      typename std::vector<char>::const_iterator& binary_tree_in_it_end_arg,
      typename std::vector<LeafContainerT*>::const_iterator*
          leaf_container_vector_it_arg,
      typename std::vector<LeafContainerT*>::const_iterator*
          leaf_container_vector_it_end_arg,
      bool branch_reset_arg = false,
      bool do_XOR_decoding_arg = false);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serialization callbacks
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Callback executed for every leaf node data during serialization
   **/
  virtual void
  serializeTreeCallback(LeafContainerT&, const OctreeKey&)
  {}

  /** \brief Callback executed for every leaf node data during deserialization
   **/
  virtual void
  deserializeTreeCallback(LeafContainerT&, const OctreeKey&)
  {}

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Helpers
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Recursively explore the octree and remove unused branch and leaf nodes
   *  \param branch_arg: current branch node
   **/
  void
  treeCleanUpRecursive(BranchNode* branch_arg);

  /** \brief Test if octree is able to dynamically change its depth. This is required
   * for adaptive bounding box adjustment.
   * \return "false" - not resizeable due to XOR serialization
   **/
  inline bool
  octreeCanResize()
  {
    return (false);
  }

  /** \brief Prints binary representation of a byte - used for debugging
   *  \param data_arg - byte to be printed to stdout
   **/
  inline void
  printBinary(char data_arg)
  {
    unsigned char mask = 1; // Bit mask

    // Extract the bits
    for (int i = 0; i < 8; i++) {
      // Mask each bit in the byte and print it
      std::cout << ((data_arg & (mask << i)) ? "1" : "0");
    }
    std::cout << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Globals
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Amount of leaf nodes   **/
  std::size_t leaf_count_;

  /** \brief Amount of branch nodes   **/
  std::size_t branch_count_;

  /** \brief Pointer to root branch node of octree   **/
  BranchNode* root_node_;

  /** \brief Depth mask based on octree depth   **/
  uindex_t depth_mask_;

  /** \brief key range */
  OctreeKey max_key_;

  /** \brief Currently active octree buffer  **/
  unsigned char buffer_selector_;

  /** \brief flags indicating if unused branches and leafs might exist in previous
   * buffer  **/
  bool tree_dirty_flag_;

  /** \brief Octree depth */
  uindex_t octree_depth_;

  /** \brief Enable dynamic_depth
   *  \note Note that this parameter is ignored in octree2buf! */
  bool dynamic_depth_enabled_;
};
} // namespace octree
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree2buf_base.hpp>
#endif
