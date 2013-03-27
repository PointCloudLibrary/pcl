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
 */

#ifndef PCL_OCTREE_NODE_POOL_H
#define PCL_OCTREE_NODE_POOL_H

#include <vector>

#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree node pool
     * \note Used to reduce memory allocation and class instantiation events when generating octrees at high rate
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename NodeT>
      class OctreeNodePool
      {
      public:
        /** \brief Empty constructor. */
        OctreeNodePool () :
            nodePool_ ()
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeNodePool ()
        {
          deletePool ();
        }

        /** \brief Push node to pool
        *  \param node_arg: add this node to the pool
        *  */
        inline
        void
        pushNode (NodeT* node_arg)
        {
          nodePool_.push_back (node_arg);
        }

        /** \brief Pop node from pool - Allocates new nodes if pool is empty
        *  \return Pointer to octree node
        *  */
        inline NodeT*
        popNode ()
        {

          NodeT* newLeafNode;

          if (!nodePool_.size ())
          {
            // leaf pool is empty
            // we need to create a new octree leaf class
            newLeafNode = new NodeT ();
          }
          else
          {
            // reuse leaf node from branch pool
            newLeafNode = nodePool_.back ();
            nodePool_.pop_back ();
            newLeafNode->reset ();
          }

          return newLeafNode;
        }


        /** \brief Delete all nodes in pool
        *  */
        void
        deletePool ()
        {
          // delete all branch instances from branch pool
          while (!nodePool_.empty ())
          {
            delete (nodePool_.back ());
            nodePool_.pop_back ();
          }
        }

      protected:
        std::vector<NodeT*> nodePool_;
      };

  }
}

#endif
