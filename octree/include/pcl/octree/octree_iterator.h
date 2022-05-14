/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2017-, Open Perception, Inc.
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

#include <pcl/octree/octree_key.h>
#include <pcl/octree/octree_nodes.h>

#include <cstddef>
#include <deque>
#include <iterator>
#include <vector>

// Ignore warnings in the above headers
#ifdef __GNUC__
#pragma GCC system_header
#endif

namespace pcl {
namespace octree {

// Octree iterator state pushed on stack/list
struct IteratorState {
  OctreeNode* node_;
  OctreeKey key_;
  uindex_t depth_;
};

/** \brief @b Abstract octree iterator class
 * \note Octree iterator base class
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename OctreeT>
class OctreeIteratorBase : public std::iterator<std::forward_iterator_tag,
                                                const OctreeNode,
                                                void,
                                                const OctreeNode*,
                                                const OctreeNode&> {
public:
  using LeafNode = typename OctreeT::LeafNode;
  using BranchNode = typename OctreeT::BranchNode;

  using LeafContainer = typename OctreeT::LeafContainer;
  using BranchContainer = typename OctreeT::BranchContainer;

  /** \brief Empty constructor.
   */
  OctreeIteratorBase() : OctreeIteratorBase(nullptr, 0u) {}

  /** \brief Constructor.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeIteratorBase(uindex_t max_depth_arg)
  : octree_(nullptr), current_state_(nullptr), max_octree_depth_(max_depth_arg)
  {
    this->reset();
  }

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   */
  OctreeIteratorBase(OctreeT* octree_arg) : OctreeIteratorBase(octree_arg, 0u) {}

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeIteratorBase(OctreeT* octree_arg, uindex_t max_depth_arg)
  : octree_(octree_arg), current_state_(0), max_octree_depth_(max_depth_arg)
  {
    this->reset();
  }

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   * \param[in] current_state A pointer to the current iterator state
   *
   *  \warning For advanced users only.
   */
  explicit OctreeIteratorBase(OctreeT* octree_arg,
                              uindex_t max_depth_arg,
                              IteratorState* current_state)
  : octree_(octree_arg), current_state_(current_state), max_octree_depth_(max_depth_arg)
  {}

  /** \brief Empty deconstructor. */
  virtual ~OctreeIteratorBase() {}

  /** \brief Equal comparison operator
   * \param[in] other OctreeIteratorBase to compare with
   */
  bool
  operator==(const OctreeIteratorBase& other) const
  {
    if (this == &other) // same object
      return true;
    if (octree_ != other.octree_) // refer to different octrees
      return false;
    if (!current_state_ && !other.current_state_) // both are end iterators
      return true;
    if (max_octree_depth_ == other.max_octree_depth_ && current_state_ &&
        other.current_state_ && // null dereference protection
        current_state_->key_ == other.current_state_->key_)
      return true;
    return false;
  }

  /** \brief Inequal comparison operator
   * \param[in] other OctreeIteratorBase to compare with
   */
  bool
  operator!=(const OctreeIteratorBase& other) const
  {
    return !operator==(other);
  }

  /** \brief Reset iterator */
  inline void
  reset()
  {
    current_state_ = 0;
    if (octree_ && (!max_octree_depth_)) {
      max_octree_depth_ = octree_->getTreeDepth();
    }
  }

  /** \brief Get octree key for the current iterator octree node
   * \return octree key of current node
   */
  inline const OctreeKey&
  getCurrentOctreeKey() const
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);

    return (current_state_->key_);
  }

  /** \brief Get the current depth level of octree
   * \return depth level
   */
  inline uindex_t
  getCurrentOctreeDepth() const
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);

    return (current_state_->depth_);
  }

  /** \brief Get the current octree node
   * \return pointer to current octree node
   */
  inline OctreeNode*
  getCurrentOctreeNode() const
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);

    return (current_state_->node_);
  }

  /** \brief check if current node is a branch node
   * \return true if current node is a branch node, false otherwise
   */
  inline bool
  isBranchNode() const
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);

    return (current_state_->node_->getNodeType() == BRANCH_NODE);
  }

  /** \brief check if current node is a branch node
   * \return true if current node is a branch node, false otherwise
   */
  inline bool
  isLeafNode() const
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);

    return (current_state_->node_->getNodeType() == LEAF_NODE);
  }

  /** \brief *operator.
   * \return pointer to the current octree node
   */
  inline OctreeNode*
  operator*() const
  { // return designated object
    if (octree_ && current_state_) {
      return (current_state_->node_);
    }
    else {
      return 0;
    }
  }

  /** \brief Get bit pattern of children configuration of current node
   * \return bit pattern (byte) describing the existence of 8 children of the current
   * node
   */
  inline char
  getNodeConfiguration() const
  {
    char ret = 0;

    assert(octree_ != 0);
    assert(current_state_ != 0);

    if (isBranchNode()) {

      // current node is a branch node
      const BranchNode* current_branch =
          static_cast<const BranchNode*>(current_state_->node_);

      // get child configuration bit pattern
      ret = octree_->getBranchBitPattern(*current_branch);
    }

    return (ret);
  }

  /** \brief Method for retrieving a single leaf container from the octree leaf node
   * \return Reference to container class of leaf node.
   */
  const LeafContainer&
  getLeafContainer() const
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);
    assert(this->isLeafNode());

    LeafNode* leaf_node = static_cast<LeafNode*>(current_state_->node_);

    return leaf_node->getContainer();
  }

  /** \brief Method for retrieving a single leaf container from the octree leaf node
   * \return Reference to container class of leaf node.
   */
  LeafContainer&
  getLeafContainer()
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);
    assert(this->isLeafNode());

    LeafNode* leaf_node = static_cast<LeafNode*>(current_state_->node_);

    return leaf_node->getContainer();
  }

  /** \brief Method for retrieving the container from an octree branch node
   * \return BranchContainer.
   */
  const BranchContainer&
  getBranchContainer() const
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);
    assert(this->isBranchNode());

    BranchNode* branch_node = static_cast<BranchNode*>(current_state_->node_);

    return branch_node->getContainer();
  }

  /** \brief Method for retrieving the container from an octree branch node
   * \return BranchContainer.
   */
  BranchContainer&
  getBranchContainer()
  {
    assert(octree_ != 0);
    assert(current_state_ != 0);
    assert(this->isBranchNode());

    BranchNode* branch_node = static_cast<BranchNode*>(current_state_->node_);

    return branch_node->getContainer();
  }

  /** \brief get a integer identifier for current node (note: identifier depends on tree
   * depth). \return node id.
   */
  virtual unsigned long
  getNodeID() const
  {
    unsigned long id = 0;

    assert(octree_ != 0);
    assert(current_state_ != 0);

    if (current_state_) {
      const OctreeKey& key = getCurrentOctreeKey();
      // calculate integer id with respect to octree key
      uindex_t depth = octree_->getTreeDepth();
      id = static_cast<unsigned long>(key.x) << (depth * 2) |
           static_cast<unsigned long>(key.y) << (depth * 1) |
           static_cast<unsigned long>(key.z) << (depth * 0);
    }

    return id;
  }

protected:
  /** \brief Reference to octree class. */
  OctreeT* octree_;

  /** \brief Pointer to current iterator state. */
  IteratorState* current_state_;

  /** \brief Maximum octree depth */
  uindex_t max_octree_depth_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Octree iterator class
 * \note This class implements a forward iterator for traversing octrees in a
 * depth-first manner.
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename OctreeT>
class OctreeDepthFirstIterator : public OctreeIteratorBase<OctreeT> {

public:
  using LeafNode = typename OctreeIteratorBase<OctreeT>::LeafNode;
  using BranchNode = typename OctreeIteratorBase<OctreeT>::BranchNode;

  /** \brief Empty constructor.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeDepthFirstIterator(uindex_t max_depth_arg = 0);

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeDepthFirstIterator(OctreeT* octree_arg, uindex_t max_depth_arg = 0);

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   * \param[in] current_state A pointer to the current iterator state
   *
   *  \warning For advanced users only.
   */
  explicit OctreeDepthFirstIterator(
      OctreeT* octree_arg,
      uindex_t max_depth_arg,
      IteratorState* current_state,
      const std::vector<IteratorState>& stack = std::vector<IteratorState>())
  : OctreeIteratorBase<OctreeT>(octree_arg, max_depth_arg, current_state), stack_(stack)
  {}

  /** \brief Copy Constructor.
   * \param[in] other Another OctreeDepthFirstIterator to copy from
   */
  OctreeDepthFirstIterator(const OctreeDepthFirstIterator& other)
  : OctreeIteratorBase<OctreeT>(other), stack_(other.stack_)
  {
    this->current_state_ = stack_.size() ? &stack_.back() : NULL;
  }

  /** \brief Copy assignment
   * \param[in] src the iterator to copy into this
   */
  inline OctreeDepthFirstIterator&
  operator=(const OctreeDepthFirstIterator& src)
  {

    OctreeIteratorBase<OctreeT>::operator=(src);

    stack_ = src.stack_;

    if (stack_.size()) {
      this->current_state_ = &stack_.back();
    }
    else {
      this->current_state_ = 0;
    }

    return (*this);
  }

  /** \brief Reset the iterator to the root node of the octree
   */
  virtual void
  reset();

  /** \brief Preincrement operator.
   * \note recursively step to next octree node
   */
  OctreeDepthFirstIterator&
  operator++();

  /** \brief postincrement operator.
   * \note recursively step to next octree node
   */
  inline OctreeDepthFirstIterator
  operator++(int)
  {
    OctreeDepthFirstIterator _Tmp = *this;
    ++*this;
    return (_Tmp);
  }

  /** \brief Skip all child voxels of current node and return to parent node.
   */
  void
  skipChildVoxels();

protected:
  /** Stack structure. */
  std::vector<IteratorState> stack_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Octree iterator class
 * \note This class implements a forward iterator for traversing octrees in a
 * breadth-first manner.
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename OctreeT>
class OctreeBreadthFirstIterator : public OctreeIteratorBase<OctreeT> {
public:
  // public typedefs
  using BranchNode = typename OctreeIteratorBase<OctreeT>::BranchNode;
  using LeafNode = typename OctreeIteratorBase<OctreeT>::LeafNode;

  /** \brief Empty constructor.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeBreadthFirstIterator(uindex_t max_depth_arg = 0);

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeBreadthFirstIterator(OctreeT* octree_arg, uindex_t max_depth_arg = 0);

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   * \param[in] current_state A pointer to the current iterator state
   *
   *  \warning For advanced users only.
   */
  explicit OctreeBreadthFirstIterator(
      OctreeT* octree_arg,
      uindex_t max_depth_arg,
      IteratorState* current_state,
      const std::deque<IteratorState>& fifo = std::deque<IteratorState>())
  : OctreeIteratorBase<OctreeT>(octree_arg, max_depth_arg, current_state), FIFO_(fifo)
  {}

  /** \brief Copy Constructor.
   * \param[in] other Another OctreeBreadthFirstIterator to copy from
   */
  OctreeBreadthFirstIterator(const OctreeBreadthFirstIterator& other)
  : OctreeIteratorBase<OctreeT>(other), FIFO_(other.FIFO_)
  {
    this->current_state_ = FIFO_.size() ? &FIFO_.front() : NULL;
  }

  /** \brief Copy operator.
   * \param[in] src the iterator to copy into this
   */
  inline OctreeBreadthFirstIterator&
  operator=(const OctreeBreadthFirstIterator& src)
  {

    OctreeIteratorBase<OctreeT>::operator=(src);

    FIFO_ = src.FIFO_;

    if (FIFO_.size()) {
      this->current_state_ = &FIFO_.front();
    }
    else {
      this->current_state_ = 0;
    }

    return (*this);
  }

  /** \brief Reset the iterator to the root node of the octree
   */
  void
  reset();

  /** \brief Preincrement operator.
   * \note step to next octree node
   */
  OctreeBreadthFirstIterator&
  operator++();

  /** \brief postincrement operator.
   * \note step to next octree node
   */
  inline OctreeBreadthFirstIterator
  operator++(int)
  {
    OctreeBreadthFirstIterator _Tmp = *this;
    ++*this;
    return (_Tmp);
  }

protected:
  /** FIFO list */
  std::deque<IteratorState> FIFO_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Octree iterator class
 * \note Iterator  over all existing nodes at a given depth. It walks across an octree
 *       in a breadth-first manner.
 * \ingroup octree
 * \author Fabien Rozar (fabien.rozar@gmail.com)
 */
template <typename OctreeT>
class OctreeFixedDepthIterator : public OctreeBreadthFirstIterator<OctreeT> {
public:
  // public typedefs
  using typename OctreeBreadthFirstIterator<OctreeT>::BranchNode;
  using typename OctreeBreadthFirstIterator<OctreeT>::LeafNode;

  /** \brief Empty constructor.
   */
  OctreeFixedDepthIterator();

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] fixed_depth_arg Depth level during traversal
   */
  explicit OctreeFixedDepthIterator(OctreeT* octree_arg, uindex_t fixed_depth_arg = 0);

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] fixed_depth_arg Depth level during traversal
   * \param[in] current_state A pointer to the current iterator state
   * \param[in] fifo Internal container of octree node to go through
   *
   *  \warning For advanced users only.
   */
  OctreeFixedDepthIterator(
      OctreeT* octree_arg,
      uindex_t fixed_depth_arg,
      IteratorState* current_state,
      const std::deque<IteratorState>& fifo = std::deque<IteratorState>())
  : OctreeBreadthFirstIterator<OctreeT>(
        octree_arg, fixed_depth_arg, current_state, fifo)
  , fixed_depth_(fixed_depth_arg)
  {}

  /** \brief Copy Constructor.
   * \param[in] other Another OctreeFixedDepthIterator to copy from
   */
  OctreeFixedDepthIterator(const OctreeFixedDepthIterator& other)
  : OctreeBreadthFirstIterator<OctreeT>(other)
  {
    this->fixed_depth_ = other.fixed_depth_;
  }

  /** \brief Copy assignment.
   * \param[in] src the iterator to copy into this
   * \return pointer to the current octree node
   */
  inline OctreeFixedDepthIterator&
  operator=(const OctreeFixedDepthIterator& src)
  {
    OctreeBreadthFirstIterator<OctreeT>::operator=(src);
    this->fixed_depth_ = src.fixed_depth_;

    return (*this);
  }

  /** \brief Reset the iterator to the first node at the depth given as parameter
   * \param[in] fixed_depth_arg Depth level during traversal
   */
  void
  reset(uindex_t fixed_depth_arg);

  /** \brief Reset the iterator to the first node at the current depth
   */
  void
  reset()
  {
    this->reset(fixed_depth_);
  }

protected:
  using OctreeBreadthFirstIterator<OctreeT>::FIFO_;

  /** \brief Given level of the node to be iterated */
  uindex_t fixed_depth_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Octree leaf node iterator class
 * \note This class implements a forward iterator for traversing the leaf nodes of an
 * octree data structure.
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
class OctreeLeafNodeDepthFirstIterator : public OctreeDepthFirstIterator<OctreeT> {
  using BranchNode = typename OctreeDepthFirstIterator<OctreeT>::BranchNode;
  using LeafNode = typename OctreeDepthFirstIterator<OctreeT>::LeafNode;

public:
  /** \brief Empty constructor.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeLeafNodeDepthFirstIterator(uindex_t max_depth_arg = 0)
  : OctreeDepthFirstIterator<OctreeT>(max_depth_arg)
  {
    reset();
  }

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeLeafNodeDepthFirstIterator(OctreeT* octree_arg,
                                            uindex_t max_depth_arg = 0)
  : OctreeDepthFirstIterator<OctreeT>(octree_arg, max_depth_arg)
  {
    reset();
  }

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   * \param[in] current_state A pointer to the current iterator state
   *
   *  \warning For advanced users only.
   */
  explicit OctreeLeafNodeDepthFirstIterator(
      OctreeT* octree_arg,
      uindex_t max_depth_arg,
      IteratorState* current_state,
      const std::vector<IteratorState>& stack = std::vector<IteratorState>())
  : OctreeDepthFirstIterator<OctreeT>(octree_arg, max_depth_arg, current_state, stack)
  {}

  /** \brief Reset the iterator to the root node of the octree
   */
  inline void
  reset()
  {
    OctreeDepthFirstIterator<OctreeT>::reset();
    this->operator++();
  }

  /** \brief Preincrement operator.
   * \note recursively step to next octree leaf node
   */
  inline OctreeLeafNodeDepthFirstIterator&
  operator++()
  {
    do {
      OctreeDepthFirstIterator<OctreeT>::operator++();
    } while ((this->current_state_) &&
             (this->current_state_->node_->getNodeType() != LEAF_NODE));

    return (*this);
  }

  /** \brief postincrement operator.
   * \note step to next octree node
   */
  inline OctreeLeafNodeDepthFirstIterator
  operator++(int)
  {
    OctreeLeafNodeDepthFirstIterator _Tmp = *this;
    ++*this;
    return (_Tmp);
  }

  /** \brief *operator.
   * \return pointer to the current octree leaf node
   */
  OctreeNode*
  operator*() const
  {
    // return designated object
    OctreeNode* ret = 0;

    if (this->current_state_ &&
        (this->current_state_->node_->getNodeType() == LEAF_NODE))
      ret = this->current_state_->node_;
    return (ret);
  }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Octree leaf node iterator class
 * \note This class implements a forward iterator for traversing the leaf nodes of an
 * octree data structure in the breadth first way.
 * \ingroup octree
 * \author Fabien Rozar
 * (fabien.rozar@gmail.com)
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
class OctreeLeafNodeBreadthFirstIterator : public OctreeBreadthFirstIterator<OctreeT> {
  using BranchNode = typename OctreeBreadthFirstIterator<OctreeT>::BranchNode;
  using LeafNode = typename OctreeBreadthFirstIterator<OctreeT>::LeafNode;

public:
  /** \brief Empty constructor.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeLeafNodeBreadthFirstIterator(uindex_t max_depth_arg = 0);

  /** \brief Constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   */
  explicit OctreeLeafNodeBreadthFirstIterator(OctreeT* octree_arg,
                                              uindex_t max_depth_arg = 0);

  /** \brief Copy constructor.
   * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
   * root node.
   * \param[in] max_depth_arg Depth limitation during traversal
   * \param[in] current_state A pointer to the current iterator state
   * \param[in] fifo Internal container of octree node to go through
   *
   *  \warning For advanced users only.
   */
  explicit OctreeLeafNodeBreadthFirstIterator(
      OctreeT* octree_arg,
      uindex_t max_depth_arg,
      IteratorState* current_state,
      const std::deque<IteratorState>& fifo = std::deque<IteratorState>());

  /** \brief Reset the iterator to the first leaf in the breadth first way.
   */
  inline void
  reset();

  /** \brief Preincrement operator.
   * \note recursively step to next octree leaf node
   */
  inline OctreeLeafNodeBreadthFirstIterator&
  operator++();

  /** \brief Postincrement operator.
   * \note step to next octree node
   */
  inline OctreeLeafNodeBreadthFirstIterator
  operator++(int);
};

} // namespace octree
} // namespace pcl

/*
 * Note: Since octree iterators depend on octrees, don't precompile them.
 */
#include <pcl/octree/impl/octree_iterator.hpp>
