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

#ifndef PCL_OCTREE_ITERATOR_H
#define PCL_OCTREE_ITERATOR_H

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

    // Octree iterator state pushed on stack/list
    struct IteratorState{
        OctreeNode* node_;
        OctreeKey key_;
        unsigned char depth_;
    };


    /** \brief @b Abstract octree iterator class
     * \note Octree iterator base class
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename OctreeT>
      class OctreeIteratorBase : public std::iterator<std::forward_iterator_tag, const OctreeNode, void,
          const OctreeNode*, const OctreeNode&>
      {
      public:

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        typedef typename OctreeT::LeafContainer LeafContainer;
        typedef typename OctreeT::BranchContainer BranchContainer;

        /** \brief Empty constructor.
         */
        explicit
        OctreeIteratorBase (unsigned int maxDepth_arg = 0) :
            octree_ (0), currentState_(0), maxOctreeDepth_(maxDepth_arg)
        {
          this->reset ();
        }

        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeIteratorBase (OctreeT* octree_arg, unsigned int maxDepth_arg = 0) :
            octree_ (octree_arg), currentState_(0), maxOctreeDepth_(maxDepth_arg)
        {
          this->reset ();
        }

        /** \brief Copy constructor.
         * \param[in] src the iterator to copy into this
         */
        OctreeIteratorBase (const OctreeIteratorBase& src, unsigned int maxDepth_arg = 0) :
            octree_ (src.octree_), currentState_(0), maxOctreeDepth_(maxDepth_arg)
        {
          this->reset ();
        }

        /** \brief Copy operator.
         * \param[in] src the iterator to copy into this
         */
        inline OctreeIteratorBase&
        operator = (const OctreeIteratorBase& src)
        {
          octree_ = src.octree_;
          currentState_ = src.currentState_;
          maxOctreeDepth_ = src.maxOctreeDepth_;
          return (*this);
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeIteratorBase ()
        {
        }

        /** \brief Equal comparison operator
         * \param[in] OctreeIteratorBase to compare with
         */
        bool operator==(const OctreeIteratorBase& other) const
        {
          return (( octree_ ==other.octree_) &&
                  ( currentState_ == other.currentState_) &&
                  ( maxOctreeDepth_ == other.maxOctreeDepth_) );
        }

        /** \brief Inequal comparison operator
         * \param[in] OctreeIteratorBase to compare with
         */
        bool operator!=(const OctreeIteratorBase& other) const
        {
          return (( octree_ !=other.octree_) &&
                  ( currentState_ != other.currentState_) &&
                  ( maxOctreeDepth_ != other.maxOctreeDepth_) );
        }

        /** \brief Reset iterator */
        inline void reset ()
        {
          currentState_ = 0;
          if (octree_ && (!maxOctreeDepth_))
          {
            maxOctreeDepth_ = octree_->getTreeDepth();
          }
        }

        /** \brief Get octree key for the current iterator octree node
         * \return octree key of current node
         */
        inline const OctreeKey&
        getCurrentOctreeKey () const
        {
          assert(octree_!=0);
          assert(currentState_!=0);

          return (currentState_->key_);
        }

        /** \brief Get the current depth level of octree
         * \return depth level
         */
        inline unsigned int
        getCurrentOctreeDepth () const
        {
          assert(octree_!=0);
          assert(currentState_!=0);

          return (currentState_->depth_);
        }

        /** \brief Get the current octree node
         * \return pointer to current octree node
         */
        inline OctreeNode*
        getCurrentOctreeNode () const
        {
          assert(octree_!=0);
          assert(currentState_!=0);

          return (currentState_->node_);
        }


        /** \brief check if current node is a branch node
         * \return true if current node is a branch node, false otherwise
         */
        inline bool
        isBranchNode () const
        {
          assert(octree_!=0);
          assert(currentState_!=0);

          return (currentState_->node_->getNodeType () == BRANCH_NODE);
        }

        /** \brief check if current node is a branch node
         * \return true if current node is a branch node, false otherwise
         */
        inline bool
        isLeafNode () const
        {
          assert(octree_!=0);
          assert(currentState_!=0);

          return (currentState_->node_->getNodeType () == LEAF_NODE);
        }

        /** \brief *operator.
         * \return pointer to the current octree node
         */
        inline OctreeNode*
        operator* () const
        { // return designated object
          if (octree_ && currentState_)
          {
            return (currentState_->node_);
          } else
          {
            return 0;
          }
        }

        /** \brief Get bit pattern of children configuration of current node
         * \return bit pattern (byte) describing the existence of 8 children of the current node
         */
        inline char
        getNodeConfiguration () const
        {
          char ret = 0;

          assert(octree_!=0);
          assert(currentState_!=0);

          if (isBranchNode ())
          {

            // current node is a branch node
            const BranchNode* currentBranch = static_cast<const BranchNode*> (currentState_->node_);

            // get child configuration bit pattern
            ret = octree_->getBranchBitPattern (*currentBranch);

          }

          return (ret);
        }

        /** \brief Method for retrieving a single leaf container from the octree leaf node
         * \return Reference to container class of leaf node.
         */
        const LeafContainer&
        getLeafContainer () const
        {
          assert(octree_!=0);
          assert(currentState_!=0);
          assert(this->isLeafNode());

          LeafNode* leaf_node = static_cast<LeafNode*>(currentState_->node_);

          return leaf_node->getContainer();
        }

        /** \brief Method for retrieving a single leaf container from the octree leaf node
         * \return Reference to container class of leaf node.
         */
        LeafContainer&
        getLeafContainer ()
        {
          assert(octree_!=0);
          assert(currentState_!=0);
          assert(this->isLeafNode());

          LeafNode* leaf_node = static_cast<LeafNode*>(currentState_->node_);

          return leaf_node->getContainer();
        }

        /** \brief Method for retrieving the container from an octree branch node
         * \return BranchContainer.
         */
        const BranchContainer&
        getBranchContainer () const
        {
          assert(octree_!=0);
          assert(currentState_!=0);
          assert(this->isBranchNode());

          BranchNode* branch_node = static_cast<BranchNode*>(currentState_->node_);

          return branch_node->getContainer();
        }

        /** \brief Method for retrieving the container from an octree branch node
         * \return BranchContainer.
         */
        BranchContainer&
        getBranchContainer ()
        {
          assert(octree_!=0);
          assert(currentState_!=0);
          assert(this->isBranchNode());

          BranchNode* branch_node = static_cast<BranchNode*>(currentState_->node_);

          return branch_node->getContainer();
        }

        /** \brief get a integer identifier for current node (note: identifier depends on tree depth).
         * \return node id.
         */
        virtual unsigned long
        getNodeID () const
        {
          unsigned long id = 0;

          assert(octree_!=0);
          assert(currentState_!=0);

          if (currentState_)
          {
            const OctreeKey& key = getCurrentOctreeKey();
            // calculate integer id with respect to octree key
            unsigned int depth = octree_->getTreeDepth ();
            id = key.x << (depth * 2) | key.y << (depth * 1) | key.z << (depth * 0);
          }

          return id;
        }

      protected:
        /** \brief Reference to octree class. */
        OctreeT* octree_;

        /** \brief Pointer to current iterator state. */
        IteratorState* currentState_;

        /** \brief Maximum octree depth */
        unsigned int maxOctreeDepth_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree iterator class
     * \note This class implements a forward iterator for traversing octrees in a depth-first manner.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename OctreeT>
      class OctreeDepthFirstIterator : public OctreeIteratorBase<OctreeT>
      {

      public:

        typedef typename OctreeIteratorBase<OctreeT>::LeafNode LeafNode;
        typedef typename OctreeIteratorBase<OctreeT>::BranchNode BranchNode;

        /** \brief Empty constructor.
         */
        explicit
        OctreeDepthFirstIterator (unsigned int maxDepth_arg = 0);

        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeDepthFirstIterator (OctreeT* octree_arg, unsigned int maxDepth_arg = 0);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeDepthFirstIterator ();

        /** \brief Copy operator.
         * \param[in] src the iterator to copy into this
         */
        inline OctreeDepthFirstIterator&
        operator = (const OctreeDepthFirstIterator& src)
        {

          OctreeIteratorBase<OctreeT>::operator=(src);

          stack_ = src.stack_;

          if (stack_.size())
          {
            this->currentState_ = &stack_.back();
          } else
          {
            this->currentState_ = 0;
          }

          return (*this);
        }

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
        /** Stack structure. */
        std::vector<IteratorState> stack_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree iterator class
     * \note This class implements a forward iterator for traversing octrees in a breadth-first manner.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename OctreeT>
      class OctreeBreadthFirstIterator : public OctreeIteratorBase<OctreeT>
      {
        // public typedefs
        typedef typename OctreeIteratorBase<OctreeT>::BranchNode BranchNode;
        typedef typename OctreeIteratorBase<OctreeT>::LeafNode LeafNode;


      public:
        /** \brief Empty constructor.
         */
        explicit
        OctreeBreadthFirstIterator (unsigned int maxDepth_arg = 0);

        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeBreadthFirstIterator (OctreeT* octree_arg, unsigned int maxDepth_arg = 0);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeBreadthFirstIterator ();

        /** \brief Copy operator.
         * \param[in] src the iterator to copy into this
         */
        inline OctreeBreadthFirstIterator&
        operator = (const OctreeBreadthFirstIterator& src)
        {

          OctreeIteratorBase<OctreeT>::operator=(src);

          FIFO_ = src.FIFO_;

          if (FIFO_.size())
          {
            this->currentState_ = &FIFO_.front();
          } else
          {
            this->currentState_ = 0;
          }

          return (*this);
        }

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
        /** FIFO list */
        std::deque<IteratorState> FIFO_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Octree leaf node iterator class
     * \note This class implements a forward iterator for traversing the leaf nodes of an octree data structure.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename OctreeT>
      class OctreeLeafNodeIterator : public OctreeDepthFirstIterator<OctreeT>
      {
        typedef typename OctreeDepthFirstIterator<OctreeT>::BranchNode BranchNode;
        typedef typename OctreeDepthFirstIterator<OctreeT>::LeafNode LeafNode;

      public:
        /** \brief Empty constructor.
         */
        explicit
        OctreeLeafNodeIterator (unsigned int maxDepth_arg = 0) :
            OctreeDepthFirstIterator<OctreeT> (maxDepth_arg)
        {
          reset ();
        }

        /** \brief Constructor.
         * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
         */
        explicit
        OctreeLeafNodeIterator (OctreeT* octree_arg, unsigned int maxDepth_arg = 0) :
            OctreeDepthFirstIterator<OctreeT> (octree_arg, maxDepth_arg)
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
          OctreeDepthFirstIterator<OctreeT>::reset ();
          this->operator++ ();
        }

        /** \brief Preincrement operator.
         * \note recursively step to next octree leaf node
         */
        inline OctreeLeafNodeIterator&
        operator++ ()
        {
          do
          {
            OctreeDepthFirstIterator<OctreeT>::operator++ ();
          } while ((this->currentState_) && (this->currentState_->node_->getNodeType () != LEAF_NODE));

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

          if (this->currentState_ && (this->currentState_->node_->getNodeType () == LEAF_NODE))
            ret = this->currentState_->node_;
          return (ret);
        }
      };

  }
}

#endif

