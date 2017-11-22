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

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeDepthFirstIterator<OctreeT>::OctreeDepthFirstIterator (unsigned int max_depth_arg) :
        OctreeIteratorBase<OctreeT> (max_depth_arg), stack_ ()
    {
      // initialize iterator
      this->reset ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeDepthFirstIterator<OctreeT>::OctreeDepthFirstIterator (OctreeT* octree_arg, unsigned int max_depth_arg) :
        OctreeIteratorBase<OctreeT> (octree_arg, max_depth_arg), stack_ ()
    {
      // initialize iterator
      this->reset ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeDepthFirstIterator<OctreeT>::~OctreeDepthFirstIterator ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    void OctreeDepthFirstIterator<OctreeT>::reset ()
    {
      OctreeIteratorBase<OctreeT>::reset ();

      if (this->octree_)
      {
        // allocate stack
        stack_.reserve (this->max_octree_depth_);

        // empty stack
        stack_.clear ();

        // pushing root node to stack
        IteratorState stack_entry;
        stack_entry.node_ = this->octree_->getRootNode ();
        stack_entry.depth_ = 0;
        stack_entry.key_.x = stack_entry.key_.y = stack_entry.key_.z = 0;

        stack_.push_back(stack_entry);

        this->current_state_ = &stack_.back();
      }

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    void OctreeDepthFirstIterator<OctreeT>::skipChildVoxels ()
    {

      if (stack_.size ())
      {
        // current depth
        unsigned char current_depth = stack_.back ().depth_;

        // pop from stack as long as we find stack elements with depth >= current depth
        while (stack_.size () && (stack_.back ().depth_ >= current_depth))
          stack_.pop_back ();

        if (stack_.size ())
        {
          this->current_state_ = &stack_.back();
        } else
        {
          this->current_state_ = 0;
        }
      }

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeDepthFirstIterator<OctreeT>&
    OctreeDepthFirstIterator<OctreeT>::operator++ ()
    {

      if (stack_.size ())
      {
        // get stack element
        IteratorState stack_entry = stack_.back ();
        stack_.pop_back ();

        stack_entry.depth_ ++;
        OctreeKey& current_key = stack_entry.key_;

        if ( (this->max_octree_depth_>=stack_entry.depth_) &&
             (stack_entry.node_->getNodeType () == BRANCH_NODE) )
        {
          unsigned char child_idx;

          // current node is a branch node
          BranchNode* current_branch =
              static_cast<BranchNode*> (stack_entry.node_);

          // add all children to stack
          for (child_idx = 0; child_idx < 8; ++child_idx)
          {

            // if child exist

            if (this->octree_->branchHasChild(*current_branch, child_idx))
            {
              // add child to stack
              current_key.pushBranch (child_idx);

              stack_entry.node_ = this->octree_->getBranchChildPtr(*current_branch, child_idx);

              stack_.push_back(stack_entry);

              current_key.popBranch();
            }
          }
        }

        if (stack_.size ())
        {
          this->current_state_ = &stack_.back();
        } else
        {
          this->current_state_ = 0;
        }
      }

      return (*this);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeBreadthFirstIterator<OctreeT>::OctreeBreadthFirstIterator (unsigned int max_depth_arg) :
        OctreeIteratorBase<OctreeT> (max_depth_arg), FIFO_ ()
    {
      OctreeIteratorBase<OctreeT>::reset ();

      // initialize iterator
      this->reset ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeBreadthFirstIterator<OctreeT>::OctreeBreadthFirstIterator (OctreeT* octree_arg, unsigned int max_depth_arg) :
        OctreeIteratorBase<OctreeT> (octree_arg, max_depth_arg), FIFO_ ()
    {
      OctreeIteratorBase<OctreeT>::reset ();

      // initialize iterator
      this->reset ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeBreadthFirstIterator<OctreeT>::~OctreeBreadthFirstIterator ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    void OctreeBreadthFirstIterator<OctreeT>::reset ()
    {
      OctreeIteratorBase<OctreeT>::reset ();

      // init FIFO
      FIFO_.clear ();

      if (this->octree_)
      {
        // pushing root node to stack
        IteratorState FIFO_entry;
        FIFO_entry.node_ = this->octree_->getRootNode ();
        FIFO_entry.depth_ = 0;
        FIFO_entry.key_.x = FIFO_entry.key_.y = FIFO_entry.key_.z = 0;

        FIFO_.push_back(FIFO_entry);

        this->current_state_ = &FIFO_.front();
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
    OctreeBreadthFirstIterator<OctreeT>&
    OctreeBreadthFirstIterator<OctreeT>::operator++ ()
    {

      if (FIFO_.size ())
      {
        // get stack element
        IteratorState FIFO_entry = FIFO_.front ();
        FIFO_.pop_front ();

        FIFO_entry.depth_ ++;
        OctreeKey& current_key = FIFO_entry.key_;

        if ( (this->max_octree_depth_>=FIFO_entry.depth_) &&
             (FIFO_entry.node_->getNodeType () == BRANCH_NODE) )
        {
          unsigned char child_idx;
          
          // current node is a branch node
          BranchNode* current_branch =
              static_cast<BranchNode*> (FIFO_entry.node_);

          // iterate over all children
          for (child_idx = 0; child_idx < 8 ; ++child_idx)
          {

            // if child exist
            if (this->octree_->branchHasChild(*current_branch, child_idx))
            {
              // add child to stack
              current_key.pushBranch (static_cast<unsigned char> (child_idx));

              FIFO_entry.node_ = this->octree_->getBranchChildPtr(*current_branch, child_idx);

              FIFO_.push_back(FIFO_entry);

              current_key.popBranch();
            }
          }
        }

        if (FIFO_.size ())
        {
          this->current_state_ = &FIFO_.front();
        } else
        {
          this->current_state_ = 0;
        }

      }

      return (*this);
    }
  }
}

#endif
