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
 * $Id: outofcore_depth_first_iterator.h 7938 2012-11-14 06:27:39Z jrosen $
 */

#ifndef PCL_OUTOFCORE_BREADTH_FIRST_ITERATOR_H_
#define PCL_OUTOFCORE_BREADTH_FIRST_ITERATOR_H_

#include <pcl/outofcore/outofcore_iterator_base.h>
namespace pcl
{
  namespace outofcore
  {

    /** \class OutofcoreBreadthFirstIterator
     *
     *  \ingroup outofcore
     *  \author Justin Rosen (jmylesrosen@gmail.com)
     *  \note Code adapted from \ref octree_iterator.h in Module \ref pcl::octree written by Julius Kammerl
     */
    template<typename PointT=pcl::PointXYZ, typename ContainerT=OutofcoreOctreeDiskContainer<pcl::PointXYZ> >
    class OutofcoreBreadthFirstIterator : public OutofcoreIteratorBase<PointT, ContainerT>
    {
      public:
        typedef typename pcl::outofcore::OutofcoreOctreeBase<ContainerT, PointT> OctreeDisk;
        typedef typename pcl::outofcore::OutofcoreOctreeBaseNode<ContainerT, PointT> OctreeDiskNode;

        typedef typename pcl::outofcore::OutofcoreOctreeBaseNode<ContainerT, PointT> LeafNode;
        typedef typename pcl::outofcore::OutofcoreOctreeBaseNode<ContainerT, PointT> BranchNode;


        explicit
        OutofcoreBreadthFirstIterator (OctreeDisk& octree_arg);

        virtual
        ~OutofcoreBreadthFirstIterator ();
      
        OutofcoreBreadthFirstIterator&
        operator++ ();
      
        inline OutofcoreBreadthFirstIterator
        operator++ (int)
        {
          OutofcoreBreadthFirstIterator _Tmp = *this;
          ++*this;
          return (_Tmp);
        }

        virtual inline void
        reset ()
        {
          OutofcoreIteratorBase<PointT, ContainerT>::reset();

          // Clear the FIFO queue and add the root as the first node
          FIFO_.clear ();
          FIFO_.push_back(this->currentNode_);

          // Don't skip children
          skip_child_voxels_ = false;
        }
      
        void
        skipChildVoxels ()
        {
          skip_child_voxels_ = true;
        }
      
      protected:
        /** FIFO list */
        std::deque<OctreeDiskNode*> FIFO_;
        bool skip_child_voxels_;
    };
  }
}

#endif //PCL_OUTOFCORE_BREADTH_FIRST_ITERATOR_H_
