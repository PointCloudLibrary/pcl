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

#ifndef PCL_OCTREE_TREE_2BUF_BASE_H
#define PCL_OCTREE_TREE_2BUF_BASE_H

#include <vector>

#include "octree_nodes.h"
#include "octree_container.h"
#include "octree_key.h"
#include "octree_iterator.h"

#include <stdio.h>
#include <string.h>

namespace pcl
{
  namespace octree
  {

    template<typename ContainerT>
    class BufferedBranchNode : public OctreeNode
    {

      public:
        /** \brief Empty constructor. */
        BufferedBranchNode () : OctreeNode()
        {
          reset ();
        }

        /** \brief Copy constructor. */
        BufferedBranchNode (const BufferedBranchNode& source) : OctreeNode()
        {
          *this = source;
        }

        /** \brief Copy operator. */
        inline BufferedBranchNode&
        operator = (const BufferedBranchNode &source_arg)
        {

          unsigned char i, b;

          memset (childNodeArray_, 0, sizeof(childNodeArray_));

          for (b = 0; b < 2; ++b)
          for (i = 0; i < 8; ++i)
            if (source_arg.childNodeArray_[b][i])
              childNodeArray_[b][i] = source_arg.childNodeArray_[b][i]->deepCopy ();

          return (*this);

        }

        /** \brief Empty constructor. */
        virtual ~BufferedBranchNode ()
        {
        }

        /** \brief Method to perform a deep copy of the octree */
        virtual BufferedBranchNode*
        deepCopy () const
        {
          return new BufferedBranchNode (*this);
        }

        /** \brief Get child pointer in current branch node
         *  \param buffer_arg: buffer selector
         *  \param index_arg: index of child in node
         *  \return pointer to child node
         * */
        inline OctreeNode*
        getChildPtr (unsigned char buffer_arg, unsigned char index_arg) const
        {
          assert( (buffer_arg<2) && (index_arg<8));
          return childNodeArray_[buffer_arg][index_arg];
        }

        /** \brief Set child pointer in current branch node
         *  \param buffer_arg: buffer selector
         *  \param index_arg: index of child in node
         *  \param newNode_arg: pointer to new child node
         * */
        inline void setChildPtr (unsigned char buffer_arg, unsigned char index_arg,
            OctreeNode* newNode_arg)
        {
          assert( (buffer_arg<2) && (index_arg<8));
          childNodeArray_[buffer_arg][index_arg] = newNode_arg;
        }

        /** \brief Check if branch is pointing to a particular child node
         *  \param buffer_arg: buffer selector
         *  \param index_arg: index of child in node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool hasChild (unsigned char buffer_arg, unsigned char index_arg) const
        {
          assert( (buffer_arg<2) && (index_arg<8));
          return (childNodeArray_[buffer_arg][index_arg] != 0);
        }

        /** \brief Get the type of octree node. Returns LEAVE_NODE type */
        virtual node_type_t getNodeType () const
        {
          return BRANCH_NODE;
        }

        /** \brief Reset branch node container for every branch buffer. */
        inline void reset ()
        {
          memset (&childNodeArray_[0][0], 0, sizeof(OctreeNode*) * 8 * 2);
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

        OctreeNode* childNodeArray_[2][8];
    };

    /** \brief @b Octree double buffer class
     *
     * \note This octree implementation keeps two separate octree structures
     * in memory. This enables to create octree structures at high rate due to
     * an advanced memory management.
     *
     * \note Furthermore, it allows for detecting and differentially compare the adjacent octree structures.
     * \note The tree depth defines the maximum amount of octree voxels / leaf nodes (should be initially defined).
     * \note All leaf nodes are addressed by integer indices.
     * \note Note: The tree depth equates to the bit length of the voxel indices.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename LeafContainerT = int,
             typename BranchContainerT = OctreeContainerEmpty >
    class Octree2BufBase
    {

      public:

        typedef Octree2BufBase<LeafContainerT, BranchContainerT> OctreeT;

        // iterators are friends
        friend class OctreeIteratorBase<OctreeT> ;
        friend class OctreeDepthFirstIterator<OctreeT> ;
        friend class OctreeBreadthFirstIterator<OctreeT> ;
        friend class OctreeLeafNodeIterator<OctreeT> ;

        typedef BufferedBranchNode<BranchContainerT> BranchNode;
        typedef OctreeLeafNode<LeafContainerT> LeafNode;

        typedef BranchContainerT BranchContainer;
        typedef LeafContainerT LeafContainer;

        // Octree default iterators
        typedef OctreeDepthFirstIterator<OctreeT> Iterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstIterator;
        Iterator begin(unsigned int maxDepth_arg = 0) {return Iterator(this, maxDepth_arg);};
        const Iterator end() {return Iterator();};

        // Octree leaf node iterators
        typedef OctreeLeafNodeIterator<OctreeT> LeafNodeIterator;
        typedef const OctreeLeafNodeIterator<OctreeT> ConstLeafNodeIterator;
        LeafNodeIterator leaf_begin(unsigned int maxDepth_arg = 0) {return LeafNodeIterator(this, maxDepth_arg);};
        const LeafNodeIterator leaf_end() {return LeafNodeIterator();};

        // Octree depth-first iterators
        typedef OctreeDepthFirstIterator<OctreeT> DepthFirstIterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstDepthFirstIterator;
        DepthFirstIterator depth_begin(unsigned int maxDepth_arg = 0) {return DepthFirstIterator(this, maxDepth_arg);};
        const DepthFirstIterator depth_end() {return DepthFirstIterator();};

        // Octree breadth-first iterators
        typedef OctreeBreadthFirstIterator<OctreeT> BreadthFirstIterator;
        typedef const OctreeBreadthFirstIterator<OctreeT> ConstBreadthFirstIterator;
        BreadthFirstIterator breadth_begin(unsigned int maxDepth_arg = 0) {return BreadthFirstIterator(this, maxDepth_arg);};
        const BreadthFirstIterator breadth_end() {return BreadthFirstIterator();};

        /** \brief Empty constructor. */
        Octree2BufBase ();

        /** \brief Empty deconstructor. */
        virtual
        ~Octree2BufBase ();

        /** \brief Copy constructor. */
        Octree2BufBase (const Octree2BufBase& source) :
            leafCount_ (source.leafCount_), branchCount_ (source.branchCount_),
            rootNode_ (new (BranchNode) (*(source.rootNode_))), depthMask_ (source.depthMask_),
            maxKey_ (source.maxKey_),bufferSelector_ (source.bufferSelector_),
            treeDirtyFlag_ (source.treeDirtyFlag_), octreeDepth_ (source.octreeDepth_),
            dynamic_depth_enabled_(source.dynamic_depth_enabled_)
        {
        }

        /** \brief Copy constructor. */
        inline Octree2BufBase&
        operator = (const Octree2BufBase& source)
        {
          leafCount_ = source.leafCount_;
          branchCount_ = source.branchCount_;
          rootNode_ = new (BranchNode) (* (source.rootNode_));
          depthMask_ = source.depthMask_;
          maxKey_ = source.maxKey_;
          bufferSelector_ = source.bufferSelector_;
          treeDirtyFlag_ = source.treeDirtyFlag_;
          octreeDepth_ = source.octreeDepth_;
          dynamic_depth_enabled_ = source.dynamic_depth_enabled_;
          return (*this);
        }

        /** \brief Set the maximum amount of voxels per dimension.
         *  \param maxVoxelIndex_arg: maximum amount of voxels per dimension
         * */
        void
        setMaxVoxelIndex (unsigned int maxVoxelIndex_arg);

        /** \brief Set the maximum depth of the octree.
         *  \param depth_arg: maximum depth of octree
         * */
        void
        setTreeDepth (unsigned int depth_arg);

        /** \brief Get the maximum depth of the octree.
         *  \return depth_arg: maximum depth of octree
         * */
        inline unsigned int getTreeDepth () const
        {
          return this->octreeDepth_;
        }

        /** \brief Create new leaf node at (idxX, idxY, idxZ).
         *  \note If leaf node already exist, this method returns the existing node
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \return pointer to new leaf node container.
         * */
        LeafContainerT*
        createLeaf (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg);

        /** \brief Find leaf node at (idxX, idxY, idxZ).
         *  \note If leaf node already exist, this method returns the existing node
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \return pointer to leaf node container if found, null pointer otherwise.
         * */
        LeafContainerT*
        findLeaf (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg);

        /** \brief Check for the existence of leaf node at (idxX, idxY, idxZ).
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \return "true" if leaf node search is successful, otherwise it returns "false".
         * */
        bool
        existLeaf (unsigned int idxX_arg, unsigned int idxY_arg,
            unsigned int idxZ_arg) const;

        /** \brief Remove leaf node at (idxX_arg, idxY_arg, idxZ_arg).
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         * */
        void
        removeLeaf (unsigned int idxX_arg, unsigned int idxY_arg,
            unsigned int idxZ_arg);

        /** \brief Return the amount of existing leafs in the octree.
         *  \return amount of registered leaf nodes.
         * */
        inline std::size_t getLeafCount () const
        {
          return (leafCount_);
        }

        /** \brief Return the amount of existing branches in the octree.
         *  \return amount of branch nodes.
         * */
        inline std::size_t getBranchCount () const
        {
          return (branchCount_);
        }

        /** \brief Delete the octree structure and its leaf nodes.
         *  \param freeMemory_arg: if "true", allocated octree nodes are deleted, otherwise they are pushed to the octree node pool
         * */
        void
        deleteTree ();

        /** \brief Delete octree structure of previous buffer. */
        inline void deletePreviousBuffer ()
        {
          treeCleanUpRecursive (rootNode_);
        }

        /** \brief Delete the octree structure in the current buffer. */
        inline void deleteCurrentBuffer ()
        {
          bufferSelector_ = !bufferSelector_;
          treeCleanUpRecursive (rootNode_);
          leafCount_ = 0;
        }

        /** \brief Switch buffers and reset current octree structure. */
        void
        switchBuffers ();

        /** \brief Serialize octree into a binary output vector describing its branch node structure.
         *  \param binaryTreeOut_arg: reference to output vector for writing binary tree structure.
         *  \param doXOREncoding_arg: select if binary tree structure should be generated based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg,
            bool doXOREncoding_arg = false);

        /** \brief Serialize octree into a binary output vector describing its branch node structure and and push all DataT elements stored in the octree to a vector.
         * \param binaryTreeOut_arg: reference to output vector for writing binary tree structure.
         * \param dataVector_arg: pointer to all LeafContainerT objects in the octree
         * \param doXOREncoding_arg: select if binary tree structure should be generated based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg, std::vector<LeafContainerT*>& dataVector_arg,
                       bool doXOREncoding_arg = false);

        /** \brief Outputs a vector of all DataT elements that are stored within the octree leaf nodes.
         *  \param dataVector_arg: vector of pointers to all LeafContainerT objects in the octree
         * */
        void
        serializeLeafs (std::vector<LeafContainerT*>& dataVector_arg);

        /** \brief Outputs a vector of all DataT elements from leaf nodes, that do not exist in the previous octree buffer.
         *  \param dataVector_arg: vector of pointers to all LeafContainerT objects in the octree
         * */
        void
        serializeNewLeafs (std::vector<LeafContainerT*>& dataVector_arg);

        /** \brief Deserialize a binary octree description vector and create a corresponding octree structure. Leaf nodes are initialized with getDataTByKey(..).
         *  \param binaryTreeIn_arg: reference to input vector for reading binary tree structure.
         *  \param doXORDecoding_arg: select if binary tree structure is based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg,
            bool doXORDecoding_arg = false);

        /** \brief Deserialize a binary octree description and create a corresponding octree structure. Leaf nodes are initialized with DataT elements from the dataVector.
         *  \param binaryTreeIn_arg: reference to inpvectoream for reading binary tree structure.
         *  \param dataVector_arg: vector of pointers to all LeafContainerT objects in the octree
         *  \param doXORDecoding_arg: select if binary tree structure is based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg,
            std::vector<LeafContainerT*>& dataVector_arg, bool doXORDecoding_arg = false);

      protected:

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Retrieve root node */
        OctreeNode*
        getRootNode () const
        {
          return (this->rootNode_);
        }

        /** \brief Find leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf container. If leaf node is not found, this pointer returns 0.
         * */
        inline LeafContainerT*
        findLeaf (const OctreeKey& key_arg) const
        {
          LeafContainerT* result = 0;
          findLeafRecursive (key_arg, depthMask_, rootNode_, result);
          return result;
        }

        /** \brief Create a leaf node.
         *  \note If the leaf node at the given octree node does not exist, it will be created and added to the tree.
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to an existing or created leaf container.
         * */
        inline LeafContainerT*
        createLeaf (const OctreeKey& key_arg)
        {
          LeafNode* leaf_node;
          BranchNode* leaf_node_parent;

          createLeafRecursive (key_arg, depthMask_ ,rootNode_, leaf_node, leaf_node_parent, false);

          LeafContainerT* ret = leaf_node->getContainerPtr();

          return ret;
        }

        /** \brief Check for leaf not existance in the octree
         *  \param key_arg: octree key addressing a leaf node.
         *  \return "true" if leaf node is found; "false" otherwise
         * */
        inline bool existLeaf (const OctreeKey& key_arg) const
        {
          return (findLeaf(key_arg) != 0);
        }

        /** \brief Remove leaf node from octree
         *  \param key_arg: octree key addressing a leaf node.
         * */
        inline void removeLeaf (const OctreeKey& key_arg)
        {
          if (key_arg <= maxKey_)
          {
            deleteLeafRecursive (key_arg, depthMask_, rootNode_);

            // we changed the octree structure -> dirty
            treeDirtyFlag_ = true;
          }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Branch node accessor inline functions
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Check if branch is pointing to a particular child node
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool
        branchHasChild (const BranchNode& branch_arg, unsigned char childIdx_arg) const
        {
          // test occupancyByte for child existence
          return (branch_arg.getChildPtr(bufferSelector_, childIdx_arg) != 0);
        }

        /** \brief Retrieve a child node pointer for child node at childIdx.
         * \param branch_arg: reference to octree branch class
         * \param childIdx_arg: index to child node
         * \return pointer to octree child node class
         */
        inline OctreeNode*
        getBranchChildPtr (const BranchNode& branch_arg,
            unsigned char childIdx_arg) const
        {
          return branch_arg.getChildPtr(bufferSelector_, childIdx_arg);
        }

        /** \brief Assign new child node to branch
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newChild_arg: pointer to new child node
         * */
        inline void setBranchChildPtr (BranchNode& branch_arg,
            unsigned char childIdx_arg, OctreeNode* newChild_arg)
        {
          branch_arg.setChildPtr(bufferSelector_, childIdx_arg, newChild_arg);
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers for current buffer
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node information
         * */
        inline char getBranchBitPattern (const BranchNode& branch_arg) const
        {
          unsigned char i;
          char nodeBits;

          // create bit pattern
          nodeBits = 0;
          for (i = 0; i < 8; i++)
          {
            const OctreeNode* child = branch_arg.getChildPtr(bufferSelector_, i);
            nodeBits |= static_cast<char> ( (!!child) << i);
          }

          return (nodeBits);
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers in specific buffer
         *  \param branch_arg: reference to octree branch class
         *  \param bufferSelector_arg: buffer selector
         *  \return a single byte with 8 bits of child node information
         * */
        inline char getBranchBitPattern (const BranchNode& branch_arg,
            unsigned char bufferSelector_arg) const
        {
          unsigned char i;
          char nodeBits;

          // create bit pattern
          nodeBits = 0;
          for (i = 0; i < 8; i++)
          {
            const OctreeNode* child = branch_arg.getChildPtr(bufferSelector_arg, i);
            nodeBits |= static_cast<char> ( (!!child) << i);
          }

          return (nodeBits);
        }

        /** \brief Generate XOR bit pattern reflecting differences between the two octree buffers
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node XOR difference information
         * */
        inline char getBranchXORBitPattern (
            const BranchNode& branch_arg) const
        {
          unsigned char i;
          char nodeBits[2];

          // create bit pattern for both buffers
          nodeBits[0] = nodeBits[1] = 0;

          for (i = 0; i < 8; i++)
          {
            const OctreeNode* childA = branch_arg.getChildPtr(0, i);
            const OctreeNode* childB = branch_arg.getChildPtr(1, i);

            nodeBits[0] |= static_cast<char> ( (!!childA) << i);
            nodeBits[1] |= static_cast<char> ( (!!childB) << i);
          }

          return nodeBits[0] ^ nodeBits[1];
        }

        /** \brief Test if branch changed between previous and current buffer
         *  \param branch_arg: reference to octree branch class
         *  \return "true", if child node information differs between current and previous octree buffer
         * */
        inline bool hasBranchChanges (const BranchNode& branch_arg) const
        {
          return (getBranchXORBitPattern (branch_arg) > 0);
        }

        /** \brief Delete child node and all its subchilds from octree in specific buffer
         *  \param branch_arg: reference to octree branch class
         *  \param bufferSelector_arg: buffer selector
         *  \param childIdx_arg: index to child node
         * */
        inline void deleteBranchChild (BranchNode& branch_arg,
            unsigned char bufferSelector_arg,
            unsigned char childIdx_arg)
        {
          if (branch_arg.hasChild(bufferSelector_arg, childIdx_arg))
          {
            OctreeNode* branchChild = branch_arg.getChildPtr(bufferSelector_arg, childIdx_arg);

            switch (branchChild->getNodeType ())
            {
              case BRANCH_NODE:
              {
                // free child branch recursively
                deleteBranch (*static_cast<BranchNode*> (branchChild));

                // delete unused branch
                delete (branchChild);
                break;
              }

              case LEAF_NODE:
              {
                // push unused leaf to branch pool
                delete (branchChild);
                break;
              }
              default:
                break;
            }

            // set branch child pointer to 0
            branch_arg.setChildPtr(bufferSelector_arg, childIdx_arg, 0);
          }
        }

        /** \brief Delete child node and all its subchilds from octree in current buffer
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         * */
        inline void deleteBranchChild (BranchNode& branch_arg,  unsigned char childIdx_arg)
        {
          deleteBranchChild(branch_arg, bufferSelector_, childIdx_arg);
        }

        /** \brief Delete branch and all its subchilds from octree (both buffers)
         *  \param branch_arg: reference to octree branch class
         * */
        inline void deleteBranch (BranchNode& branch_arg)
        {
          char i;

          // delete all branch node children
          for (i = 0; i < 8; i++)
          {

            if (branch_arg.getChildPtr(0, i) == branch_arg.getChildPtr(1, i))
            {
              // reference was copied - there is only one child instance to be deleted
              deleteBranchChild (branch_arg, 0, i);

              // remove pointers from both buffers
              branch_arg.setChildPtr(0, i, 0);
              branch_arg.setChildPtr(1, i, 0);
            }
            else
            {
              deleteBranchChild (branch_arg, 0, i);
              deleteBranchChild (branch_arg, 1, i);
            }
          }
        }

        /** \brief Fetch and add a new branch child to a branch class in current buffer
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return pointer of new branch child to this reference
         * */
        inline  BranchNode* createBranchChild (BranchNode& branch_arg,
            unsigned char childIdx_arg)
        {
          BranchNode* newBranchChild = new BranchNode();

          branch_arg.setChildPtr (bufferSelector_, childIdx_arg,
              static_cast<OctreeNode*> (newBranchChild));

          return newBranchChild;
        }

        /** \brief Fetch and add a new leaf child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return pointer of new leaf child to this reference
         * */
        inline LeafNode* createLeafChild (BranchNode& branch_arg,
            unsigned char childIdx_arg)
        {
          LeafNode* newLeafChild = new LeafNode();

          branch_arg.setChildPtr(bufferSelector_, childIdx_arg, newLeafChild);

          return newLeafChild;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive octree methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Create a leaf node at octree key. If leaf node does already exist, it is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param returnLeaf_arg: return pointer to leaf container
         *  \param leafParent_arg: return pointer to parent of leaf node
         *  \param branchReset_arg: Reset pointer array of current branch
         *  \return depth mask at which leaf node was created
         **/
        unsigned int
        createLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg,
                             LeafNode*& returnLeaf_arg, BranchNode*& leafParent_arg, bool branchReset_arg = false);


        /** \brief Recursively search for a given leaf node and return a pointer.
         *  \note  If leaf node does not exist, a 0 pointer is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param result_arg: pointer to leaf container class
         **/
        void
        findLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg, LeafContainerT*& result_arg) const;


        /** \brief Recursively search and delete leaf node
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param branch_arg: current branch node
         *  \return "true" if branch does not contain any childs; "false" otherwise. This indicates if current branch can be deleted.
         **/
        bool
        deleteLeafRecursive (const OctreeKey& key_arg,
            unsigned int depthMask_arg, BranchNode* branch_arg);

        /** \brief Recursively explore the octree and output binary octree description together with a vector of leaf node DataT content.
         *  \param binaryTreeOut_arg: binary output vector
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: vector to return pointers to all leaf container in the tree.
         *  \param doXOREncoding_arg: select if binary tree structure should be generated based on current octree (false) of based on a XOR comparison between current and previous octree
         *  \param newLeafsFilter_arg: execute callback only for leaf nodes that did not exist in preceding buffer
         **/
        void
        serializeTreeRecursive (BranchNode* branch_arg, OctreeKey& key_arg,
            std::vector<char>* binaryTreeOut_arg,
            typename std::vector<LeafContainerT*>* dataVector_arg, bool doXOREncoding_arg = false,
            bool newLeafsFilter_arg = false);

        /** \brief Rebuild an octree based on binary XOR octree description and DataT objects for leaf node initialization.
         *  \param binaryTreeIn_arg: iterator to input vector
         *  \param branch_arg: current branch node
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param key_arg: reference to an octree key
         *  \param dataVectorIterator_arg: iterator pointing to leaf containter pointers to be added to a leaf node
         *  \param dataVectorEndIterator_arg: iterator pointing to leaf containter pointers pointing to last object in input container.
         *  \param branchReset_arg: Reset pointer array of current branch
         *  \param doXORDecoding_arg: select if binary tree structure is based on current octree (false) of based on a XOR comparison between current and previous octree
         **/
        void
        deserializeTreeRecursive (BranchNode* branch_arg,
            unsigned int depthMask_arg, OctreeKey& key_arg,
            typename std::vector<char>::const_iterator& binaryTreeIT_arg,
            typename std::vector<char>::const_iterator& binaryTreeIT_End_arg,
            typename std::vector<LeafContainerT*>::const_iterator* dataVectorIterator_arg,
            typename std::vector<LeafContainerT*>::const_iterator* dataVectorEndIterator_arg,
            bool branchReset_arg = false, bool doXORDecoding_arg = false);


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Serialization callbacks
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Callback executed for every leaf node data during serialization
         **/
        virtual void serializeTreeCallback (LeafContainerT &, const OctreeKey &)
        {

        }

        /** \brief Callback executed for every leaf node data during deserialization
         **/
        virtual void deserializeTreeCallback (LeafContainerT&, const OctreeKey&)
        {

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Helpers
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Recursively explore the octree and remove unused branch and leaf nodes
         *  \param branch_arg: current branch node
         **/
        void
        treeCleanUpRecursive (BranchNode* branch_arg);

        /** \brief Helper function to calculate the binary logarithm
         * \param n_arg: some value
         * \return binary logarithm (log2) of argument n_arg
         */
        inline double Log2 (double n_arg)
        {
          return log (n_arg) / log (2.0);
        }

        /** \brief Test if octree is able to dynamically change its depth. This is required for adaptive bounding box adjustment.
         *  \return "false" - not resizeable due to XOR serialization
         **/
        inline bool octreeCanResize ()
        {
          return (false);
        }

        /** \brief Prints binary representation of a byte - used for debugging
         *  \param data_arg - byte to be printed to stdout
         **/
        inline void printBinary (char data_arg)
        {
          unsigned char mask = 1;  // Bit mask

          // Extract the bits
          for (int i = 0; i < 8; i++)
          {
            // Mask each bit in the byte and print it
            std::cout << ((data_arg & (mask << i)) ? "1" : "0");
          }
          std::cout << std::endl;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Amount of leaf nodes   **/
        std::size_t leafCount_;

        /** \brief Amount of branch nodes   **/
        std::size_t branchCount_;

        /** \brief Pointer to root branch node of octree   **/
        BranchNode* rootNode_;

        /** \brief Depth mask based on octree depth   **/
        unsigned int depthMask_;

        /** \brief key range */
        OctreeKey maxKey_;

        /** \brief Currently active octree buffer  **/
        unsigned char bufferSelector_;

        // flags indicating if unused branches and leafs might exist in previous buffer
        bool treeDirtyFlag_;

        /** \brief Octree depth */
        unsigned int octreeDepth_;

        /** \brief Enable dynamic_depth
         *  \note Note that this parameter is ignored in octree2buf! */
        bool dynamic_depth_enabled_;

    };
  }
}

//#include "impl/octree2buf_base.hpp"

#endif

