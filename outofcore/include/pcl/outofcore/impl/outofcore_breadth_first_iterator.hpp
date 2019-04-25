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
 * $Id: outofcore_depth_first_iterator.hpp 7938 2012-11-14 06:27:39Z jrosen $
 */

#ifndef PCL_OUTOFCORE_BREADTH_FIRST_ITERATOR_IMPL_H_
#define PCL_OUTOFCORE_BREADTH_FIRST_ITERATOR_IMPL_H_

namespace pcl
{
  namespace outofcore
  {

    template<typename PointT, typename ContainerT> 
    OutofcoreBreadthFirstIterator<PointT, ContainerT>::OutofcoreBreadthFirstIterator (OutofcoreOctreeBase<ContainerT, PointT>& octree_arg)
    : OutofcoreIteratorBase<PointT, ContainerT> (octree_arg)
    {
      reset();
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT, typename ContainerT> 
    OutofcoreBreadthFirstIterator<PointT, ContainerT>::~OutofcoreBreadthFirstIterator ()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT, typename ContainerT>
    OutofcoreBreadthFirstIterator<PointT, ContainerT>&
    OutofcoreBreadthFirstIterator<PointT, ContainerT>::operator++ ()
    {
      if (!FIFO_.empty ())
      {
        // Get the first entry from the FIFO queue
        OctreeDiskNode *node = FIFO_.front ();
        FIFO_.pop_front ();

        // If not skipping children, not at the max specified depth and we're a branch then iterate over children
        if (!skip_child_voxels_ && node->getDepth () < this->max_depth_ && node->getNodeType () == pcl::octree::BRANCH_NODE)
        {
          // Get the branch node
          BranchNode* branch = static_cast<BranchNode*> (node);
          OctreeDiskNode* child = nullptr;

          // Iterate over the branches children
          for (unsigned char child_idx = 0; child_idx < 8 ; child_idx++)
          {
            // If child/index exists add it to FIFO queue
            child = this->octree_.getBranchChildPtr (*branch, child_idx);
            if (child)
            {
              FIFO_.push_back (child);
            }
          }
        }
      }

      // Reset skipped children
      skip_child_voxels_ = false;

      // If there's a queue, set the current node to the first entry
      if (!FIFO_.empty ())
      {
        this->currentNode_ = FIFO_.front ();
      }
      else
      {
        this->currentNode_ = nullptr;
      }

      return (*this);
    }

    ////////////////////////////////////////////////////////////////////////////////

  }//namesapce pcl
}//namespace outofcore

#endif //PCL_OUTOFCORE_BREADTH_FIRST_ITERATOR_IMPL_H_

