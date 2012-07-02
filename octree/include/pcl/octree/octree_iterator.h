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

#ifndef OCTREE_ITERATOR_H
#define OCTREE_ITERATOR_H

#include <cstddef>
#include <vector>
#include <deque>

#include "octree_nodes.h"
#include "octree_key.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iterator>

// Ignore warnings in the above headers
#ifdef __GNUC__
#pragma GCC system_header 
#endif

namespace pcl
{
  namespace octree
  {
    /** \brief @b Abstract octree iterator class
     * \note Octree iterator base class
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT, typename OctreeT>
      class OctreeIteratorBase : public std::iterator<std::forward_iterator_tag, const OctreeNode, void,
          const OctreeNode*, const OctreeNode&>
      {
      public:

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeIteratorBase (OctreeT& octree_arg) :
            octree_ (octree_arg), currentNode_ (NULL)
        {
          reset ();
        }

        /** \brief Copy constructor.
         * \param[in] src the iterator to copy into this
         */
        OctreeIteratorBase (const OctreeIteratorBase& src) :
            octree_ (src.octree_), currentNode_ (src.currentNode_), currentOctreeDepth_ (src.currentOctreeDepth_), currentOctreeKey_ (
                src.currentOctreeKey_)
        {
        }

        /** \brief Copy operator.
         * \param[in] src the iterator to copy into this
         */
        inline OctreeIteratorBase&
        operator = (const OctreeIteratorBase& src)
        {
          octree_ = src.octree_;
          currentNode_ = src.currentNode_;
          currentOctreeDepth_ = src.currentOctreeDepth_;
          currentOctreeKey_ = src.currentOctreeKey_;
          return (*this);
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeIteratorBase ()
        {
        }

        /** \brief initialize iterator globals */
        inline void
        reset ()
        {
          // initialize iterator globals
          currentNode_ = static_cast<OctreeNode*> (octree_.getRootNode ());
          currentOctreeDepth_ = 0;

          // reset octree key
          currentOctreeKey_.x = currentOctreeKey_.y = currentOctreeKey_.z = 0;
        }

        /** \brief Get octree key for the current iterator octree node
         * \return octree key of current node
         */
        inline const OctreeKey&
        getCurrentOctreeKey () const
        {
          return (currentOctreeKey_);
        }

        /** \brief Get the current depth level of octree
         * \return depth level
         */
        inline unsigned int
        getCurrentOctreeDepth () const
        {
          return (currentOctreeDepth_);
        }

        /** \brief Get the current octree node
         * \return pointer to current octree node
         */
        inline OctreeNode*
        getCurrentOctreeNode () const
        {
          return (currentNode_);
        }

        /** \brief *operator.
         * \return pointer to the current octree node
         */
        inline OctreeNode*
        operator* () const
        { // return designated object
          return (this->getCurrentOctreeNode ());
        }

        /** \brief check if current node is a branch node
         * \return true if current node is a branch node, false otherwise
         */
        inline bool
        isBranchNode () const
        {
          return (currentNode_->getNodeType () == BRANCH_NODE);
        }

        /** \brief check if current node is a branch node
         * \return true if current node is a branch node, false otherwise
         */
        inline bool
        isLeafNode () const
        {
          return (currentNode_->getNodeType () == LEAF_NODE);
        }

        /** \brief Get bit pattern of children configuration of current node
         * \return bit pattern (byte) describing the existence of 8 children of the current node
         */
        inline char
        getNodeConfiguration () const
        {
          char ret = 0;

          if (isBranchNode ())
          {

            // current node is a branch node
            const BranchNode* currentBranch = static_cast<const BranchNode*> (this->currentNode_);

            // get child configuration bit pattern
            ret = octree_.getBranchBitPattern (*currentBranch);

          }

          return (ret);
        }

        /** \brief Method for retrieving a single DataT element from the octree leaf node
         * \param[in] data_arg reference to return pointer of leaf node DataT element.
         */
        virtual void
        getData (DataT& data_arg) const
        {

          assert(this->currentNode_ );

          octree_.getDataFromOctreeNode(this->currentNode_, data_arg);
        }

        /** \brief Method for retrieving a vector of DataT elements from the octree laef node
         * \param[in] dataVector_arg reference to DataT vector that is extended with leaf node DataT elements.
         */
        virtual void
        getData (std::vector<DataT>& dataVector_arg) const
        {
          assert(this->currentNode_);

          octree_.getDataFromOctreeNode(this->currentNode_, dataVector_arg);
        }

        /** \brief Method for retrieving the size of the DataT vector from the octree laef node
         */
        virtual std::size_t
        getSize () const
        {
          assert(this->currentNode_);

          return octree_.getDataSizeFromOctreeNode(this->currentNode_);
        }

        /** \brief get a integer identifier for current node (note: identifier depends on tree depth).
         * \return node id.
         */
        virtual unsigned long
        getNodeID () const
        {
          unsigned long id = 0;

          if (this->currentNode_)
          {
            // calculate integer id with respect to octree key
            unsigned int depth = octree_.getTreeDepth ();
            id = currentOctreeKey_.x << (depth * 2) | currentOctreeKey_.y << (depth * 1)
                | currentOctreeKey_.z << (depth * 0);
          }

          return id;
        }

      protected:
        /** \brief Reference to octree class. */
        OctreeT& octree_;

        /** Pointer to current octree node. */
        OctreeNode* currentNode_;

        /** Depth level in the octree structure. */
        unsigned int currentOctreeDepth_;

        /** Octree key for current octree node. */
        OctreeKey currentOctreeKey_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree iterator class
     * \note This class implements a forward iterator for traversing octrees in a depth-first manner.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT, typename OctreeT>
      class OctreeDepthFirstIterator : public OctreeIteratorBase<DataT, OctreeT>
      {

      public:

        typedef typename OctreeIteratorBase<DataT, OctreeT>::LeafNode LeafNode;
        typedef typename OctreeIteratorBase<DataT, OctreeT>::BranchNode BranchNode;


        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeDepthFirstIterator (OctreeT& octree_arg);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeDepthFirstIterator ();


        /** \brief Reset the iterator to the root node of the octree
         */
        virtual void
        reset ();

        /** \brief Preincrement operator.
         * \note recursively step to next octree node
         */
        OctreeDepthFirstIterator&
        operator++ ();

        /** \brief postincrement operator.
         * \note recursively step to next octree node
         */
        inline OctreeDepthFirstIterator
        operator++ (int)
        {
          OctreeDepthFirstIterator _Tmp = *this;
          ++*this;
          return (_Tmp);
        }

        /** \brief Skip all child voxels of current node and return to parent node.
         */
        void
        skipChildVoxels ();

      protected:
        /** Child index at current octree node. */
        unsigned char currentChildIdx_;

        /** Stack structure. */
        std::vector<std::pair<OctreeNode*, unsigned char> > stack_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree iterator class
     * \note This class implements a forward iterator for traversing octrees in a breadth-first manner.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT, typename OctreeT>
      class OctreeBreadthFirstIterator : public OctreeIteratorBase<DataT, OctreeT>
      {
        // public typedefs
        typedef typename OctreeIteratorBase<DataT, OctreeT>::BranchNode BranchNode;
        typedef typename OctreeIteratorBase<DataT, OctreeT>::LeafNode LeafNode;

        struct FIFOElement
        {
          OctreeNode* node;
          OctreeKey key;
          unsigned int depth;
        };

      public:
        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeBreadthFirstIterator (OctreeT& octree_arg);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeBreadthFirstIterator ();

        /** \brief Reset the iterator to the root node of the octree
         */
        void
        reset ();

        /** \brief Preincrement operator.
         * \note step to next octree node
         */
        OctreeBreadthFirstIterator&
        operator++ ();

        /** \brief postincrement operator.
         * \note step to next octree node
         */
        inline OctreeBreadthFirstIterator
        operator++ (int)
        {
          OctreeBreadthFirstIterator _Tmp = *this;
          ++*this;
          return (_Tmp);
        }

      protected:
        /** \brief Add children of node to FIFO
         * \param[in] node: node with children to be added to FIFO
         */
        void
        addChildNodesToFIFO (const OctreeNode* node);

        /** FIFO list */
        std::deque<FIFOElement> FIFO_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Octree leaf node iterator class
     * \note This class implements a forward iterator for traversing the leaf nodes of an octree data structure.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename OctreeT>
      class OctreeLeafNodeIterator : public OctreeDepthFirstIterator<DataT, OctreeT>
      {
        typedef typename OctreeDepthFirstIterator<DataT, OctreeT>::BranchNode BranchNode;
        typedef typename OctreeDepthFirstIterator<DataT, OctreeT>::LeafNode LeafNode;

      public:
        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeLeafNodeIterator (OctreeT& octree_arg) :
            OctreeDepthFirstIterator<DataT, OctreeT> (octree_arg)
        {
          reset ();
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeLeafNodeIterator ()
        {
        }

        /** \brief Reset the iterator to the root node of the octree
         */
        inline void
        reset ()
        {
          OctreeDepthFirstIterator<DataT, OctreeT>::reset ();
        }

        /** \brief Preincrement operator.
         * \note recursively step to next octree leaf node
         */
        inline OctreeLeafNodeIterator&
        operator++ ()
        {
          do
          {
            OctreeDepthFirstIterator<DataT, OctreeT>::operator++ ();
          } while ((this->currentNode_) && (this->currentNode_->getNodeType () != LEAF_NODE));

          return (*this);
        }

        /** \brief postincrement operator.
         * \note step to next octree node
         */
        inline OctreeLeafNodeIterator
        operator++ (int)
        {
          OctreeLeafNodeIterator _Tmp = *this;
          ++*this;
          return (_Tmp);
        }

        /** \brief *operator.
         * \return pointer to the current octree leaf node
         */
        OctreeNode*
        operator* () const
        {
          // return designated object
          OctreeNode* ret = 0;

          if (this->currentNode_ && (this->currentNode_->getNodeType () == LEAF_NODE))
            ret = this->currentNode_;
          return (ret);
        }
      };

  }
}

#endif

