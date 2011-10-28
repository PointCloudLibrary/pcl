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

#include "pcl/common/common.h"

namespace pcl
{
  namespace octree
  {

    using namespace std;

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename OctreeT>
      OctreeNodeIterator<DataT, LeafT, OctreeT>::OctreeNodeIterator (const OctreeT& octree_arg) :
        octree_ (octree_arg), currentNode_ (NULL), currentChildIdx_ (0)
      {

        // allocate stack
        stack_ .reserve (octree_.getTreeDepth ());

        // initialize iterator
        reset ();

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename OctreeT>
      OctreeNodeIterator<DataT, LeafT, OctreeT>::~OctreeNodeIterator ()
      {

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename OctreeT>
      void
      OctreeNodeIterator<DataT, LeafT, OctreeT>::reset ()
      {
        // initialize iterator globals
        currentNode_ = (OctreeNode*)octree_.getRootNode ();
        currentChildIdx_ = 0;
        currentOctreeDepth_ = 0;

        // reset octree key
        currentOctreeKey_.x = currentOctreeKey_.y = currentOctreeKey_.z = 0;

        // empty stack
        stack_.clear ();
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename OctreeT>
      void
      OctreeNodeIterator<DataT, LeafT, OctreeT>::skipChildVoxels ()
      {
        if (currentNode_)
        {
          // make sure, we are not at the root node
          if (stack_.size () > 0)
          {
            // pop stack
            std::pair<OctreeNode const*, unsigned char>& stackEntry = stack_.back ();
            stack_.pop_back ();

            // assign parent node and child index
            currentNode_ = stackEntry.first;
            currentChildIdx_ = stackEntry.second;

            // update octree key
            currentOctreeKey_.x = (currentOctreeKey_.x >> 1);
            currentOctreeKey_.y = (currentOctreeKey_.y >> 1);
            currentOctreeKey_.z = (currentOctreeKey_.z >> 1);

            // update octree depth
            currentOctreeDepth_--;

          }
          else
          {
            // we are at root node level - finish
            currentNode_ = NULL;
          }

        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename OctreeT>
      OctreeNodeIterator<DataT, LeafT, OctreeT>&
      OctreeNodeIterator<DataT, LeafT, OctreeT>::operator++ ()
      {
        if (currentNode_)
        {

          bool bTreeUp = false;
          const OctreeNode* itNode = NULL;

          if (currentNode_->getNodeType () == BRANCH_NODE)
          {
            // current node is a branch node
            const OctreeBranch* currentBranch = (const OctreeBranch*)currentNode_;

            // find next existing child node
            while ((currentChildIdx_ < 8) && !(itNode = octree_.getBranchChild (*currentBranch, currentChildIdx_)))
            {
              currentChildIdx_++;
            };

            if (currentChildIdx_ == 8)
            {
              // all childs traversed -> back to parent node
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
              std::pair<OctreeNode const*, unsigned char>& stackEntry = stack_.back ();
              stack_.pop_back ();

              // assign parent node and child index
              currentNode_ = stackEntry.first;
              currentChildIdx_ = stackEntry.second;

              // update octree key
              currentOctreeKey_.x = (currentOctreeKey_.x >> 1);
              currentOctreeKey_.y = (currentOctreeKey_.y >> 1);
              currentOctreeKey_.z = (currentOctreeKey_.z >> 1);

              // update octree depth
              currentOctreeDepth_--;
            }
            else
            {
              // root level -> finish
              currentNode_ = NULL;
            }

          }
          else
          {
            // traverse child node

            // new stack entry
            std::pair<OctreeNode const*, unsigned char> newStackEntry;

            // assign current node and child index to stack entry
            newStackEntry.first = currentNode_;
            newStackEntry.second = currentChildIdx_ + 1;

            // push stack entry to stack
            stack_.push_back (newStackEntry);

            // update octree key
            currentOctreeKey_.x = (currentOctreeKey_.x << 1) | (!!(currentChildIdx_ & (1 << 2)));
            currentOctreeKey_.y = (currentOctreeKey_.y << 1) | (!!(currentChildIdx_ & (1 << 1)));
            currentOctreeKey_.z = (currentOctreeKey_.z << 1) | (!!(currentChildIdx_ & (1 << 0)));

            // update octree depth
            currentOctreeDepth_++;

            // traverse to child node
            currentChildIdx_ = 0;
            currentNode_ = itNode;
          }
        }

        return (*this);
      }
  }
}

#endif
