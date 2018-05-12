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

#ifndef PCL_OCTREE_NODE_H
#define PCL_OCTREE_NODE_H

#include <cstddef>

#include <assert.h>

#include <Eigen/Core>

#include <pcl/pcl_macros.h>

#include "octree_container.h"

namespace pcl
{
  namespace octree
  {

    // enum of node types within the octree
    enum node_type_t
    {
      BRANCH_NODE, LEAF_NODE
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree node class
     * \note Every octree node should implement the getNodeType () method
     * \author Julius Kammerl (julius@kammerl.de)
     */
    class PCL_EXPORTS OctreeNode
    {
    public:

      OctreeNode ()
      {
      }

      virtual
      ~OctreeNode ()
      {
      }
      /** \brief Pure virtual method for receiving the type of octree node (branch or leaf)  */
      virtual node_type_t
      getNodeType () const = 0;

      /** \brief Pure virtual method to perform a deep copy of the octree */
      virtual OctreeNode*
      deepCopy () const = 0;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree leaf class
     * \note Octree leafs may collect data of type DataT
     * \author Julius Kammerl (julius@kammerl.de)
     */

    template<typename ContainerT>
      class OctreeLeafNode : public OctreeNode
      {
      public:

        /** \brief Empty constructor. */
        OctreeLeafNode () :
            OctreeNode ()
        {
        }

        /** \brief Copy constructor. */
        OctreeLeafNode (const OctreeLeafNode& source) :
            OctreeNode ()
        {
          container_ = source.container_;
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeLeafNode ()
        {
        }

        /** \brief Method to perform a deep copy of the octree */
        virtual OctreeLeafNode<ContainerT>*
        deepCopy () const
        {
          return new OctreeLeafNode<ContainerT> (*this);
        }

        /** \brief Get the type of octree node. Returns LEAVE_NODE type */
        virtual node_type_t
        getNodeType () const
        {
          return LEAF_NODE;
        }

        /** \brief Get const pointer to container */
        const ContainerT*
        operator->() const
        {
          return &container_;
        }

        /** \brief Get pointer to container */
        ContainerT*
        operator-> ()
        {
          return &container_;
        }

        /** \brief Get const reference to container */
        const ContainerT&
        operator* () const
        {
          return container_;
        }

        /** \brief Get reference to container */
        ContainerT&
        operator* ()
        {
          return container_;
        }

        /** \brief Get const reference to container */
        const ContainerT&
        getContainer () const
        {
          return container_;
        }

        /** \brief Get reference to container */
        ContainerT&
        getContainer ()
        {
          return container_;
        }

        /** \brief Get const pointer to container */
        const ContainerT*
        getContainerPtr() const
        {
          return &container_;
        }

        /** \brief Get pointer to container */
        ContainerT*
        getContainerPtr ()
        {
          return &container_;
        }

      protected:
        ContainerT container_;
        
      public:
        //Type ContainerT may have fixed-size Eigen objects inside
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree branch class
     * \note Octree branch classes may collect data of type DataT
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename ContainerT>
      class OctreeBranchNode : public OctreeNode
      {
      public:

        /** \brief Empty constructor. */
        OctreeBranchNode () :
            OctreeNode()
        {
          // reset pointer to child node vectors
          memset (child_node_array_, 0, sizeof(child_node_array_));
        }

        /** \brief Empty constructor. */
        OctreeBranchNode (const OctreeBranchNode& source) :
            OctreeNode()
        {
          unsigned char i;

          memset (child_node_array_, 0, sizeof(child_node_array_));

          for (i = 0; i < 8; ++i)
            if (source.child_node_array_[i])
              child_node_array_[i] = source.child_node_array_[i]->deepCopy ();
        }

        /** \brief Copy operator. */
        inline OctreeBranchNode&
        operator = (const OctreeBranchNode &source)
        {
          unsigned char i;

          memset (child_node_array_, 0, sizeof(child_node_array_));

          for (i = 0; i < 8; ++i)
            if (source.child_node_array_[i])
              child_node_array_[i] = source.child_node_array_[i]->deepCopy ();
          return (*this);
        }

        /** \brief Octree deep copy method */
        virtual OctreeBranchNode*
        deepCopy () const
        {
          return (new OctreeBranchNode<ContainerT> (*this));
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeBranchNode ()
        {
        }

        /** \brief Access operator.
         *  \param child_idx_arg: index to child node
         *  \return OctreeNode pointer
         * */
        inline OctreeNode*&
        operator[] (unsigned char child_idx_arg)
        {
          assert(child_idx_arg < 8);
          return child_node_array_[child_idx_arg];
        }

        /** \brief Get pointer to child
         *  \param child_idx_arg: index to child node
         *  \return OctreeNode pointer
         * */
        inline OctreeNode*
        getChildPtr (unsigned char child_idx_arg) const
        {
          assert(child_idx_arg < 8);
          return child_node_array_[child_idx_arg];
        }

        /** \brief Get pointer to child
         *  \return OctreeNode pointer
         * */
        inline void setChildPtr (OctreeNode* child, unsigned char index)
        {
          assert(index < 8);
          child_node_array_[index] = child;
        }


        /** \brief Check if branch is pointing to a particular child node
         *  \param child_idx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool hasChild (unsigned char child_idx_arg) const
        {
          return (child_node_array_[child_idx_arg] != 0);
        }


        /** \brief Check if branch can be pruned
         *  \note if all children are leaf nodes AND contain identical containers, branch can be pruned
         *  \return "true" if branch can be pruned; "false" otherwise
         * */
    /*    inline bool isPrunable () const
        {
          const OctreeNode* firstChild = child_node_array_[0];
          if (!firstChild || firstChild->getNodeType()==BRANCH_NODE)
            return false;

          bool prunable = true;
          for (unsigned char i = 1; i < 8 && prunable; ++i)
          {
            const OctreeNode* child = child_node_array_[i];
            if ( (!child) ||
                 (child->getNodeType()==BRANCH_NODE) ||
                 ((*static_cast<const OctreeContainerBase*>(child)) == (*static_cast<const OctreeContainerBase*>(child)) ) )
              prunable = false;
          }

          return prunable;
        }*/

        /** \brief Get the type of octree node. Returns LEAVE_NODE type */
        virtual node_type_t
        getNodeType () const
        {
          return BRANCH_NODE;
        }

        // reset node
        void reset()
        {
          memset(child_node_array_, 0, sizeof(child_node_array_));
          container_.reset();
        }


        /** \brief Get const pointer to container */
        const ContainerT*
        operator->() const
        {
          return &container_;
        }

        /** \brief Get pointer to container */
        ContainerT*
        operator-> ()
        {
          return &container_;
        }

        /** \brief Get const reference to container */
        const ContainerT&
        operator* () const
        {
          return container_;
        }

        /** \brief Get reference to container */
        ContainerT&
        operator* ()
        {
          return container_;
        }

        /** \brief Get const reference to container */
        const ContainerT&
        getContainer () const
        {
          return container_;
        }

        /** \brief Get reference to container */
        ContainerT&
        getContainer ()
        {
          return container_;
        }

        /** \brief Get const pointer to container */
        const ContainerT*
        getContainerPtr() const
        {
          return &container_;
        }

        /** \brief Get pointer to container */
        ContainerT*
        getContainerPtr ()
        {
          return &container_;
        }

      protected:
        OctreeNode* child_node_array_[8];

        ContainerT container_;
      };
  }
}

#endif
