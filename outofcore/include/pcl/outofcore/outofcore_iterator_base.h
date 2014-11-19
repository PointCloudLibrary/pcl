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

#ifndef PCL_OUTOFCORE_ITERATOR_BASE_H_
#define PCL_OUTOFCORE_ITERATOR_BASE_H_

#include <iterator>

#include <pcl/point_types.h>

#include <pcl/outofcore/octree_base.h>
#include <pcl/outofcore/octree_base_node.h>
#include <pcl/outofcore/octree_disk_container.h>

namespace pcl
{
  namespace outofcore
  {
    /** \brief Abstract octree iterator class
     *  \note This class is based on the octree_iterator written by Julius Kammerl adapted to the outofcore octree. The interface is very similar, but it does \b not inherit the \ref pcl::octree iterator base.
     *  \ingroup outofcore
     *  \author Stephen Fox (foxstephend@gmail.com)
     */
    template<typename PointT, typename ContainerT>
    class OutofcoreIteratorBase : public std::iterator<std::forward_iterator_tag,     /*iterator type*/
                                                      const OutofcoreOctreeBaseNode<ContainerT, PointT>,
                                                       void,  /*no defined distance between iterator*/
                                                       const OutofcoreOctreeBaseNode<ContainerT, PointT>*,/*Pointer type*/
                                                       const OutofcoreOctreeBaseNode<ContainerT, PointT>&>/*Reference type*/
    {
      public:
        typedef typename pcl::outofcore::OutofcoreOctreeBase<ContainerT, PointT> OctreeDisk;
        typedef typename pcl::outofcore::OutofcoreOctreeBaseNode<ContainerT, PointT> OctreeDiskNode;
        
        typedef typename pcl::outofcore::OutofcoreOctreeBase<ContainerT, PointT>::BranchNode BranchNode;
        typedef typename pcl::outofcore::OutofcoreOctreeBase<ContainerT, PointT>::LeafNode LeafNode;

        typedef typename OctreeDisk::OutofcoreNodeType OutofcoreNodeType;

        explicit
        OutofcoreIteratorBase (OctreeDisk& octree_arg) 
          : octree_ (octree_arg), currentNode_ (NULL)
        {
          reset ();
        }
        
        virtual
        ~OutofcoreIteratorBase ()
        {
        }

        OutofcoreIteratorBase (const OutofcoreIteratorBase& src)
          : octree_ (src.octree_), currentNode_ (src.currentNode_)
        {
        }

        inline OutofcoreIteratorBase&
        operator = (const OutofcoreIteratorBase& src)
        {
          octree_ = src.octree_;
          currentNode_ = src.currentNode_;
          currentOctreeDepth_ = src.currentOctreeDepth_;
        }
        
        
        inline OutofcoreNodeType*
        operator* () const
        {
          return (this->getCurrentOctreeNode ());
        }

        virtual inline OutofcoreNodeType*
        getCurrentOctreeNode () const
        {
          return (currentNode_);
        }
        
        virtual inline void
        reset ()
        {
          currentNode_ = static_cast<OctreeDiskNode*> (octree_.getRootNode ());
          currentOctreeDepth_ = 0;
          max_depth_ = static_cast<unsigned int> (octree_.getDepth ());
        }

        inline void
        setMaxDepth (unsigned int max_depth)
        {
          if (max_depth > static_cast<unsigned int> (octree_.getDepth ()))
          {
            max_depth = static_cast<unsigned int> (octree_.getDepth ());
          }

          max_depth_ = max_depth;
        }

      protected:
        OctreeDisk& octree_;
        OctreeDiskNode* currentNode_;
        unsigned int currentOctreeDepth_;
        unsigned int max_depth_;
    };


#if 0
    class PCL_EXPORTS OutofcoreBreadthFirstIterator : public OutofcoreIteratorBase
    {
      



    };
    
    class PCL_EXPORTS OutofcoreLeafIterator : public OutofcoreIteratorBase
    {



    };
#endif
  }
}

#endif //PCL_OUTOFCORE_ITERATOR_BASE_H_
