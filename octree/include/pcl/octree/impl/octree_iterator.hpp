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

#ifndef OCTREE_ITERATOR_HPP_
#define OCTREE_ITERATOR_HPP_

#include <vector>
#include <assert.h>

#include <pcl/common/common.h>

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    OctreeDepthFirstIterator<DataT, OctreeT>::OctreeDepthFirstIterator (
        OctreeT& octree_arg) :
        OctreeIteratorBase<DataT, OctreeT> (octree_arg), currentChildIdx_ (0), stack_ ()
    {
      // allocate stack
      stack_.reserve (this->octree_.getTreeDepth ());

      // initialize iterator
      reset ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    OctreeDepthFirstIterator<DataT, OctreeT>::~OctreeDepthFirstIterator ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    void OctreeDepthFirstIterator<DataT, OctreeT>::reset ()
    {
      OctreeIteratorBase<DataT, OctreeT>::reset ();

      // empty stack
      stack_.clear ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    void OctreeDepthFirstIterator<DataT, OctreeT>::skipChildVoxels ()
    {
      if (!this->currentNode_)
        return;

      // make sure, we are not at the root node
      if (stack_.size () > 0)
      {
        // pop stack
        std::pair<OctreeNode*, unsigned char>& stackEntry =
            stack_.back ();
        stack_.pop_back ();

        // assign parent node and child index
        this->currentNode_ = stackEntry.first;
        currentChildIdx_ = stackEntry.second;

        // update octree key
        this->currentOctreeKey_.x = (this->currentOctreeKey_.x >> 1);
        this->currentOctreeKey_.y = (this->currentOctreeKey_.y >> 1);
        this->currentOctreeKey_.z = (this->currentOctreeKey_.z >> 1);

        // update octree depth
        this->currentOctreeDepth_--;
      }
      else
        // we are at root node level - finish
        this->currentNode_ = NULL;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    OctreeDepthFirstIterator<DataT, OctreeT>&
    OctreeDepthFirstIterator<DataT, OctreeT>::operator++ ()
    {
      if (this->currentNode_)
      {

        bool bTreeUp = false;
        OctreeNode* itNode = 0;

        if (this->currentNode_->getNodeType () == BRANCH_NODE)
        {
          // current node is a branch node
          BranchNode* currentBranch =
              static_cast<BranchNode*> (this->currentNode_);

          if (currentChildIdx_ < 8)
          {
            itNode = this->octree_.getBranchChildPtr (*currentBranch,
                currentChildIdx_);

            while ( (currentChildIdx_ < 7) && ! (itNode))
            {

              // find next existing child node
              currentChildIdx_++;
              itNode = this->octree_.getBranchChildPtr (*currentBranch,
                  currentChildIdx_);
            }

            if (!itNode)
            {
              // all childs traversed -> back to parent node
              bTreeUp = true;
            }
          }
          else
          {
            bTreeUp = true;
          }

        }
        else
        {
          // at leaf node level, we need to return to parent node
          bTreeUp = true;
        }

        if (bTreeUp)
        {
          // return to parent node

          if (stack_.size () > 0)
          {
            // pop the stack
            std::pair<OctreeNode*, unsigned char>& stackEntry = stack_.back ();
            stack_.pop_back ();

            // assign parent node and child index
            this->currentNode_ = stackEntry.first;
            currentChildIdx_ = stackEntry.second;

            // update octree key
            this->currentOctreeKey_.x = (this->currentOctreeKey_.x >> 1);
            this->currentOctreeKey_.y = (this->currentOctreeKey_.y >> 1);
            this->currentOctreeKey_.z = (this->currentOctreeKey_.z >> 1);

            // update octree depth
            this->currentOctreeDepth_--;
          }
          else
          {
            // root level -> finish
            this->currentNode_ = NULL;
          }

        }
        else
        {
          // traverse child node

          // new stack entry
          std::pair<OctreeNode*, unsigned char> newStackEntry;

          // assign current node and child index to stack entry
          newStackEntry.first = this->currentNode_;
          newStackEntry.second = static_cast<unsigned char> (currentChildIdx_
              + 1);

          // push stack entry to stack
          stack_.push_back (newStackEntry);

          // update octree key
          this->currentOctreeKey_.x = (this->currentOctreeKey_.x << 1)
              | (!! (currentChildIdx_ & (1 << 2)));
          this->currentOctreeKey_.y = (this->currentOctreeKey_.y << 1)
              | (!! (currentChildIdx_ & (1 << 1)));
          this->currentOctreeKey_.z = (this->currentOctreeKey_.z << 1)
              | (!! (currentChildIdx_ & (1 << 0)));

          // update octree depth
          this->currentOctreeDepth_++;

          // traverse to child node
          currentChildIdx_ = 0;
          this->currentNode_ = itNode;
        }
      }

      return (*this);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    OctreeBreadthFirstIterator<DataT, OctreeT>::OctreeBreadthFirstIterator (
        OctreeT& octree_arg) :
        OctreeIteratorBase<DataT, OctreeT> (octree_arg), FIFO_ ()
    {
      OctreeIteratorBase<DataT, OctreeT>::reset ();

      // initialize iterator
      reset ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    OctreeBreadthFirstIterator<DataT, OctreeT>::~OctreeBreadthFirstIterator ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    void OctreeBreadthFirstIterator<DataT, OctreeT>::addChildNodesToFIFO (
        const OctreeNode* node)
    {
      unsigned char i;

      if (node && (node->getNodeType () == BRANCH_NODE))
      {

        for (i = 0; i < 8; i++)
        {
          // current node is a branch node
          BranchNode* currentBranch =
              static_cast<BranchNode*> (this->currentNode_);

          OctreeNode* itNode =
              static_cast<OctreeNode*> (this->octree_.getBranchChildPtr (
                  *currentBranch, i));

          // if node exist, push it to FIFO
          if (itNode)
          {
            OctreeKey newKey;

            // generate octree key
            newKey.x = (this->currentOctreeKey_.x << 1) | (!! (i & (1 << 2)));
            newKey.y = (this->currentOctreeKey_.y << 1) | (!! (i & (1 << 1)));
            newKey.z = (this->currentOctreeKey_.z << 1) | (!! (i & (1 << 0)));

            FIFOElement newListElement;
            newListElement.node = itNode;
            newListElement.key = newKey;
            newListElement.depth = this->currentOctreeDepth_ + 1;

            FIFO_.push_back (newListElement);
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    void OctreeBreadthFirstIterator<DataT, OctreeT>::reset ()
    {
      OctreeIteratorBase<DataT, OctreeT>::reset ();

      // init FIFO
      FIFO_.clear ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
    OctreeBreadthFirstIterator<DataT, OctreeT>&
    OctreeBreadthFirstIterator<DataT, OctreeT>::operator++ ()
    {
      if (this->currentNode_)
      {

        // add childs of current node to FIFO
        addChildNodesToFIFO (this->currentNode_);

        if (FIFO_.size () > 0)
        {
          FIFOElement FIFOElement;

          // get FIFO front element
          FIFOElement = FIFO_.front ();
          FIFO_.pop_front ();

          // update iterator variables
          this->currentNode_ = FIFOElement.node;
          this->currentOctreeKey_ = FIFOElement.key;
          this->currentOctreeDepth_ = FIFOElement.depth;
        }
        else
        {
          // last node reached
          this->currentNode_ = NULL;
        }
      }

      return (*this);
    }
  }
}

#endif
