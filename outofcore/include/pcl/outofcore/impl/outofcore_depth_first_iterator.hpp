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

#ifndef PCL_OUTOFCORE_DEPTH_FIRST_ITERATOR_IMPL_H_
#define PCL_OUTOFCORE_DEPTH_FIRST_ITERATOR_IMPL_H_

namespace pcl
{
  namespace outofcore
  {

    template<typename PointT, typename ContainerT> 
    OutofcoreDepthFirstIterator<PointT, ContainerT>::OutofcoreDepthFirstIterator (OutofcoreOctreeBase<ContainerT, PointT>& octree_arg) 
    : OutofcoreIteratorBase<PointT, ContainerT> (octree_arg)
    , currentChildIdx_ (0)
    , stack_ (0)
    {
      stack_.reserve (this->octree_.getTreeDepth ());
      OutofcoreIteratorBase<PointT,ContainerT>::reset ();
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT, typename ContainerT> 
    OutofcoreDepthFirstIterator<PointT, ContainerT>::~OutofcoreDepthFirstIterator ()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT, typename ContainerT> 
    OutofcoreDepthFirstIterator<PointT, ContainerT>& 
    OutofcoreDepthFirstIterator<PointT, ContainerT>::operator++ ()
    {
      //when currentNode_ is 0, skip incrementing because it is already at the end
      if (this->currentNode_)
      {
        bool bTreeUp = false;
        OutofcoreOctreeBaseNode<ContainerT, PointT>* itNode = nullptr;

        if (this->currentNode_->getNodeType () == pcl::octree::BRANCH_NODE)
        {
          BranchNode* currentBranch = static_cast<BranchNode*> (this->currentNode_);
          
          if (currentChildIdx_ < 8)
          {
            itNode = this->octree_.getBranchChildPtr (*currentBranch, currentChildIdx_);

            //keep looking for a valid child until we've run out of children or a valid one is found
            while ((currentChildIdx_ < 7) && !(itNode))
            {
              //find next existing child node
              currentChildIdx_++;
              itNode = this->octree_.getBranchChildPtr (*currentBranch, currentChildIdx_);
            }
            //if no valid one was found, set flag to move back up the tree to the parent node
            if (!itNode)
            {
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
          bTreeUp = true;
        }
        
        if (bTreeUp)
        {
          if (!stack_.empty ())
          {
            std::pair<OutofcoreOctreeBaseNode<ContainerT, PointT>*, unsigned char>& stackEntry = stack_.back ();
            stack_.pop_back ();
              
            this->currentNode_ = stackEntry.first;
            currentChildIdx_ = stackEntry.second;
              
            //don't do anything with the keys here...
            this->currentOctreeDepth_--;
          }
          else
          {
            this->currentNode_ = nullptr;
          }
            
        }
        else
        {
          std::pair<OutofcoreOctreeBaseNode<ContainerT, PointT>*, unsigned char> newStackEntry;
          newStackEntry.first = this->currentNode_;
          newStackEntry.second = static_cast<unsigned char> (currentChildIdx_+1);
            
          stack_.push_back (newStackEntry);
            
          //don't do anything with the keys here...
            
          this->currentOctreeDepth_++;
          currentChildIdx_= 0;
          this->currentNode_ = itNode;
        }
      }
        
      return (*this);
    }

    ////////////////////////////////////////////////////////////////////////////////

  }//namesapce pcl
}//namespace outofcore

#endif //PCL_OUTOFCORE_DEPTH_FIRST_ITERATOR_IMPL_H_

