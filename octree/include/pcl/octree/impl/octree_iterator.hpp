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

#ifndef PCL_OCTREE_ITERATOR_HPP_
#define PCL_OCTREE_ITERATOR_HPP_

#include <pcl/console/print.h>

namespace pcl {
namespace octree {
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeDepthFirstIterator<OctreeT>::OctreeDepthFirstIterator(uindex_t max_depth_arg)
: OctreeIteratorBase<OctreeT>(max_depth_arg), stack_()
{
  // initialize iterator
  this->reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeDepthFirstIterator<OctreeT>::OctreeDepthFirstIterator(OctreeT* octree_arg,
                                                            uindex_t max_depth_arg)
: OctreeIteratorBase<OctreeT>(octree_arg, max_depth_arg), stack_()
{
  // initialize iterator
  this->reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
void
OctreeDepthFirstIterator<OctreeT>::reset()
{
  OctreeIteratorBase<OctreeT>::reset();

  if (this->octree_) {
    // allocate stack
    stack_.reserve(this->max_octree_depth_);

    // empty stack
    stack_.clear();

    // pushing root node to stack
    IteratorState stack_entry;
    stack_entry.node_ = this->octree_->getRootNode();
    stack_entry.depth_ = 0;
    stack_entry.key_.x = stack_entry.key_.y = stack_entry.key_.z = 0;

    stack_.push_back(stack_entry);

    this->current_state_ = &stack_.back();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
void
OctreeDepthFirstIterator<OctreeT>::skipChildVoxels()
{

  if (stack_.size()) {
    // current depth
    unsigned char current_depth = stack_.back().depth_;

    // pop from stack as long as we find stack elements with depth >= current depth
    while (stack_.size() && (stack_.back().depth_ >= current_depth))
      stack_.pop_back();

    if (stack_.size()) {
      this->current_state_ = &stack_.back();
    }
    else {
      this->current_state_ = 0;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeDepthFirstIterator<OctreeT>&
OctreeDepthFirstIterator<OctreeT>::operator++()
{

  if (stack_.size()) {
    // get stack element
    IteratorState stack_entry = stack_.back();
    stack_.pop_back();

    stack_entry.depth_++;

    if ((this->max_octree_depth_ >= stack_entry.depth_) &&
        (stack_entry.node_->getNodeType() == BRANCH_NODE)) {
      // current node is a branch node
      BranchNode* current_branch = static_cast<BranchNode*>(stack_entry.node_);

      OctreeKey& current_key = stack_entry.key_;

      // add all children to stack
      for (std::int8_t i = 7; i >= 0; --i) {
        const unsigned char child_idx = (unsigned char)i;

        // if child exist
        if (this->octree_->branchHasChild(*current_branch, child_idx)) {
          // add child to stack
          current_key.pushBranch(child_idx);

          stack_entry.node_ =
              this->octree_->getBranchChildPtr(*current_branch, child_idx);

          stack_.push_back(stack_entry);

          current_key.popBranch();
        }
      }
    }

    if (stack_.size()) {
      this->current_state_ = &stack_.back();
    }
    else {
      this->current_state_ = 0;
    }
  }

  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeBreadthFirstIterator<OctreeT>::OctreeBreadthFirstIterator(uindex_t max_depth_arg)
: OctreeIteratorBase<OctreeT>(max_depth_arg), FIFO_()
{
  OctreeIteratorBase<OctreeT>::reset();

  // initialize iterator
  this->reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeBreadthFirstIterator<OctreeT>::OctreeBreadthFirstIterator(OctreeT* octree_arg,
                                                                uindex_t max_depth_arg)
: OctreeIteratorBase<OctreeT>(octree_arg, max_depth_arg), FIFO_()
{
  OctreeIteratorBase<OctreeT>::reset();

  // initialize iterator
  this->reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
void
OctreeBreadthFirstIterator<OctreeT>::reset()
{
  OctreeIteratorBase<OctreeT>::reset();

  // init FIFO
  FIFO_.clear();

  if (this->octree_) {
    // pushing root node to stack
    IteratorState FIFO_entry;
    FIFO_entry.node_ = this->octree_->getRootNode();
    FIFO_entry.depth_ = 0;
    FIFO_entry.key_.x = FIFO_entry.key_.y = FIFO_entry.key_.z = 0;

    FIFO_.push_back(FIFO_entry);

    this->current_state_ = &FIFO_.front();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeBreadthFirstIterator<OctreeT>&
OctreeBreadthFirstIterator<OctreeT>::operator++()
{

  if (FIFO_.size()) {
    // get stack element
    IteratorState FIFO_entry = FIFO_.front();
    FIFO_.pop_front();

    FIFO_entry.depth_++;

    if ((this->max_octree_depth_ >= FIFO_entry.depth_) &&
        (FIFO_entry.node_->getNodeType() == BRANCH_NODE)) {
      // current node is a branch node
      BranchNode* current_branch = static_cast<BranchNode*>(FIFO_entry.node_);

      // iterate over all children
      for (unsigned char child_idx = 0; child_idx < 8; ++child_idx) {

        // if child exist
        if (this->octree_->branchHasChild(*current_branch, child_idx)) {
          // add child to stack
          OctreeKey& current_key = FIFO_entry.key_;
          current_key.pushBranch(static_cast<unsigned char>(child_idx));

          FIFO_entry.node_ =
              this->octree_->getBranchChildPtr(*current_branch, child_idx);

          FIFO_.push_back(FIFO_entry);

          current_key.popBranch();
        }
      }
    }

    if (FIFO_.size()) {
      this->current_state_ = &FIFO_.front();
    }
    else {
      this->current_state_ = 0;
    }
  }

  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeFixedDepthIterator<OctreeT>::OctreeFixedDepthIterator()
: OctreeBreadthFirstIterator<OctreeT>(nullptr, 0), fixed_depth_(0u)
{}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeFixedDepthIterator<OctreeT>::OctreeFixedDepthIterator(OctreeT* octree_arg,
                                                            uindex_t fixed_depth_arg)
: OctreeBreadthFirstIterator<OctreeT>(octree_arg, fixed_depth_arg)
, fixed_depth_(fixed_depth_arg)
{
  this->reset(fixed_depth_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
void
OctreeFixedDepthIterator<OctreeT>::reset(uindex_t fixed_depth_arg)
{
  // Set the desired depth to walk through
  fixed_depth_ = fixed_depth_arg;

  if (!this->octree_) {
    return;
  }

  // If I'm nowhere, reset
  // If I'm somewhere and I can't guarantee I'm before the first node, reset
  if ((!this->current_state_) || (fixed_depth_ <= this->getCurrentOctreeDepth()))
    OctreeBreadthFirstIterator<OctreeT>::reset();

  if (this->octree_->getTreeDepth() < fixed_depth_) {
    PCL_WARN("[pcl::octree::FixedDepthIterator] The requested fixed depth was bigger "
             "than the octree's depth.\n");
    PCL_WARN("[pcl::octree::FixedDepthIterator] fixed_depth = %d (instead of %d)\n",
             this->octree_->getTreeDepth(),
             fixed_depth_);
  }

  // By default for the parent class OctreeBreadthFirstIterator, if the
  // depth argument is equal to 0, the iterator would run over every node.
  // For the OctreeFixedDepthIterator, whatever the depth ask, set the
  // max_octree_depth_ accordingly
  this->max_octree_depth_ = std::min(fixed_depth_, this->octree_->getTreeDepth());

  // Restore previous state in case breath first iterator had child nodes already set up
  if (FIFO_.size())
    this->current_state_ = &FIFO_.front();

  // Iterate all the way to the desired level
  while (this->current_state_ && (this->getCurrentOctreeDepth() != fixed_depth_))
    OctreeBreadthFirstIterator<OctreeT>::operator++();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeLeafNodeBreadthFirstIterator<OctreeT>::OctreeLeafNodeBreadthFirstIterator(
    uindex_t max_depth_arg)
: OctreeBreadthFirstIterator<OctreeT>(max_depth_arg)
{
  reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeLeafNodeBreadthFirstIterator<OctreeT>::OctreeLeafNodeBreadthFirstIterator(
    OctreeT* octree_arg, uindex_t max_depth_arg)
: OctreeBreadthFirstIterator<OctreeT>(octree_arg, max_depth_arg)
{
  reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeLeafNodeBreadthFirstIterator<OctreeT>::OctreeLeafNodeBreadthFirstIterator(
    OctreeT* octree_arg,
    uindex_t max_depth_arg,
    IteratorState* current_state,
    const std::deque<IteratorState>& fifo)
: OctreeBreadthFirstIterator<OctreeT>(octree_arg, max_depth_arg, current_state, fifo)
{}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
void
OctreeLeafNodeBreadthFirstIterator<OctreeT>::reset()
{
  OctreeBreadthFirstIterator<OctreeT>::reset();
  ++*this;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeLeafNodeBreadthFirstIterator<OctreeT>&
OctreeLeafNodeBreadthFirstIterator<OctreeT>::operator++()
{
  do {
    OctreeBreadthFirstIterator<OctreeT>::operator++();
  } while ((this->current_state_) &&
           (this->current_state_->node_->getNodeType() != LEAF_NODE));

  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename OctreeT>
OctreeLeafNodeBreadthFirstIterator<OctreeT>
OctreeLeafNodeBreadthFirstIterator<OctreeT>::operator++(int)
{
  OctreeLeafNodeBreadthFirstIterator _Tmp = *this;
  ++*this;
  return (_Tmp);
}
} // namespace octree
} // namespace pcl

#endif
