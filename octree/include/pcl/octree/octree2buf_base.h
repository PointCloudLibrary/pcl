/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef OCTREE_TREE_2BUF_BASE_H
#define OCTREE_TREE_2BUF_BASE_H

#include <ostream>
#include <istream>
#include <vector>

#include "octree_nodes.h"

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree double buffer class
     *  \note This octree implementation keeps two separate octree structures in memory. This enables to create octree structures at high rate due to an advanced memory management.
     *  \note Furthermore, it allows for detecting and differentially compare the adjacent octree structures.
     *  \note The tree depth defines the maximum amount of octree voxels / leaf nodes (should be initially defined).
     *  \note All leaf nodes are addressed by integer indices.
     *  \note Note: The tree depth equates to the bit length of the voxel indices.
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT = OctreeLeafDataT<DataT> >
      class Octree2BufBase
      {

      public:

        /** \brief Empty constructor. */
        Octree2BufBase ();

        /** \brief Empty deconstructor. */
        virtual
        ~Octree2BufBase ();

        /** \brief Set the maximum depth of the octree.
         *  \param depth_arg: maximum depth of octree
         * */
        void
        setTreeDepth (unsigned int depth_arg);

        /** \brief Add a const DataT element to leaf node at (idxX, idxY, idxZ). If leaf node does not exist, it is added to the octree.
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \param data_arg: const reference to DataT object that is fed to the lead node.
         * */
        void
        add (const unsigned int idxX_arg, const unsigned int idxY_arg, const unsigned int idxZ_arg,
             const DataT& data_arg);

        /** \brief Retrieve a DataT element from leaf node at (idxX, idxY, idxZ). It returns false if leaf node does not exist.
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \param data_arg: reference to DataT object that contains content of leaf node if search was successful.
         *  \return "true" if leaf node search is successful, otherwise it returns "false".
         * */
        bool
            get (const unsigned int idxX_arg, const unsigned int idxY_arg, const unsigned int idxZ_arg, DataT& data_arg) const ;

        /** \brief Check for the existence of leaf node at (idxX, idxY, idxZ).
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxY_arg: index of leaf node in the Z axis.
         *  \return "true" if leaf node search is successful, otherwise it returns "false".
         * */
        bool
        existLeaf (const unsigned int idxX_arg, const unsigned int idxY_arg, const unsigned int idxZ_arg) const ;

        /** \brief Remove leaf node at (idxX_arg, idxY_arg, idxZ_arg).
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         * */
        void
        removeLeaf (const unsigned int idxX_arg, const unsigned int idxY_arg, const unsigned int idxZ_arg);

        /** \brief Return the amount of existing leafs in the octree.
         *  \return amount of registered leaf nodes.
         * */
        inline unsigned int
        getLeafCount () const
        {
          return leafCount_;
        }

        /** \brief Delete the octree structure and its leaf nodes. */
        void
        deleteTree ();

        /** \brief Switch buffers and reset current octree structure. */
        void
        switchBuffers ();

        /** \brief Serialize octree into a binary output stream describing its branch node structure.
         *  \param binaryTreeOut_arg: reference to output stream for writing binary tree structure.
         * */
        void
        serializeTree (std::ostream& binaryTreeOut_arg);

        /** \brief Serialize octree into a binary output stream describing its branch node structure and and push all DataT elements stored in the octree to a vector.
         * \param binaryTreeOut_arg: reference to output stream for writing binary tree structure.
         * \param dataVector_arg: reference of DataT vector that receives a copy of all DataT objects in the octree
         * */
        void
        serializeTree (std::ostream& binaryTreeOut_arg, std::vector<DataT>& dataVector_arg);

        /** \brief Outputs a vector of all DataT elements that are stored within the octree leaf nodes.
         *  \param dataVector_arg: reference to DataT vector that receives a copy of all DataT objects in the octree.
         * */
        void
        serializeLeafs (std::vector<DataT>& dataVector_arg);

        /** \brief Outputs a vector of all DataT elements from leaf nodes, that do not exist in the previous octree buffer.
         *  \param dataVector_arg: reference to DataT vector that receives a copy of all DataT objects in the octree.
         *  \param minPointsPerLeaf_arg: minimum amount of points required within leaf node to become serialized.
         * */
        void
        serializeNewLeafs (std::vector<DataT>& dataVector_arg, const int minPointsPerLeaf_arg = 0);

        /** \brief Deserialize a binary octree description stream and create a corresponding octree structure. Leaf nodes are kept empty and are not initialized with DataT elements.
         *  \param binaryTreeIn_arg: reference to input stream for reading binary tree structure.
         * */
        void
        deserializeTree (std::istream& binaryTreeIn_arg);

        /** \brief Deserialize a binary octree description and create a corresponding octree structure. Leaf nodes are initialized with DataT elements from the dataVector.
         *  \param binaryTreeIn_arg: reference to input stream for reading binary tree structure.
         *  \param dataVector_arg: reference to DataT vector that provides DataT objects for initializing leaf nodes.
         * */
        void
        deserializeTree (std::istream& binaryTreeIn_arg, std::vector<DataT>& dataVector_arg);

      protected:

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Octree key class
         *  \brief Octree keys contain integer indices for each coordinate axis in order to address an octree leaf node.
         *  \author Julius Kammerl (julius@kammerl.de)
         */
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        class OctreeKey
        {
        public:

          /** \brief Operator== for comparing octree keys with each other.
           *  \return "true" if leaf node indices are identical; "false" otherwise.
           * */
          bool
          operator == (const OctreeKey& b) const
          {
            return ((b.x == this->x) && (b.y == this->y) && (b.z == this->z));
          }

          // Indices addressing a voxel at (X, Y, Z)
          unsigned int x;unsigned int y;unsigned int z;
        };

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Octree branch class.
         * \note It stores 8 pointers to its child nodes.
         * \author Julius Kammerl (julius@kammerl.de)
         */
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        class OctreeBranch : public OctreeNode
        {

          // Octree2BufBase is a friend!
          friend class Octree2BufBase;

        public:

          /** \brief Constructor for initializing child node pointer array. */
          OctreeBranch ()
          {
            memset (this->subNodes_, 0, sizeof(this->subNodes_));
          }

          /** \brief Empty deconstructor. */
          virtual
          ~OctreeBranch ()
          {
          }

          /** \brief Get the type of octree node. Returns BRANCH_NODE type
           *  \return Returns BRANCH_NODE type.
           * */
          virtual node_type_t
          getNodeType () const
          {
            return BRANCH_NODE;
          }

        private:

          /** \brief Child node pointer array of size 2x8 for both octree structures.  */
          const OctreeNode * subNodes_[2][8];

        };

        typedef LeafT OctreeLeaf;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Virtual method for generating an octree key for a given DataT object.
         *  \param data_arg: reference to DataT object
         *  \param key_arg: write generated octree key to this octree key reference
         *  \return "true" if octree could be generated based on DataT object; "false" otherwise
         * */
        virtual bool
        genOctreeKey (const DataT& data_arg, OctreeKey & key_arg) const
        {
          // this class cannot relate DataT objects to octree keys
          return false;
        }

        /** \brief Virtual method for initializing new leaf node during deserialization (in case no DataT information is provided)
         *  \param key_arg: write generated octree key to this octree key reference
         *  \param data_arg: generated DataT object
         *  \return "true" if DataT object could be generated; "false" otherwise
         *  */
        virtual bool
        getDataTByKey (const OctreeKey & key_arg, DataT& data_arg) const
        {
          // this class cannot relate DataT objects to octree keys
          return false;
        }

        /** \brief Generate an octree key
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \param key_arg: write new octree key to this reference.
         * */
        inline void
        genOctreeKeyByIntIdx (const unsigned int idxX_arg, const unsigned int idxY_arg, const unsigned int idxZ_arg,
                              OctreeKey & key_arg) const
        {
          // copy data to octree key class
          key_arg.x = idxX_arg;
          key_arg.y = idxY_arg;
          key_arg.z = idxZ_arg;
        }

        /** \brief Add DataT object to leaf node at octree key.
         *  \param key_arg: octree key addressing a leaf node.
         *  \param data_arg: DataT object to be added.
         * */
        inline void
        add (const OctreeKey& key_arg, const DataT& data_arg)
        {
          // request a (new) leaf from tree
          LeafT* leaf = getLeaf (key_arg);

          // assign data to leaf
          leaf->setData (data_arg);
        }

        /** \brief Find leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf node. If leaf node is not found, this pointer returns NULL.
         * */
        inline LeafT*
        findLeaf (const OctreeKey& key_arg) const
        {
          return findLeafRecursive (key_arg, depthMask_, rootNode_);
        }

        /** \brief Get a leaf node from octree.
         *  \note If the leaf node at the given octree node does not exist, it will be created and added to the tree.
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to an existing or created leaf node.
         * */
        inline LeafT*
        getLeaf (const OctreeKey& key_arg)
        {
          LeafT* result;

          result = getLeafRecursive (key_arg, depthMask_, rootNode_, resetTree_);

          // getLeafRecursive has changed the octree -> clean-up/tree-reset might be required
          resetTree_ = false;
          treeDirtyFlag_ = true;

          return result;
        }

        /** \brief Check for leaf not existance in the octree
         *  \param key_arg: octree key addressing a leaf node.
         *  \return "true" if leaf node is found; "false" otherwise
         * */
        inline bool
        existLeaf (const OctreeKey& key_arg) const
        {
          return (findLeafRecursive (key_arg, depthMask_, rootNode_) != NULL);
        }

        /** \brief Remove leaf node from octree
         *  \param key_arg: octree key addressing a leaf node.
         * */
        inline void
        removeLeaf (const OctreeKey& key_arg)
        {
          deleteLeafRecursive (key_arg, depthMask_, rootNode_);

          // we changed the octree structure -> dirty
          treeDirtyFlag_ = true;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Branch node accessor inline functions
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /** \brief Retrieve a child node pointer for child node at childIdx for current octree buffer.
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return pointer to octree child node class
         * */
        inline const OctreeNode*
        getBranchChild (const OctreeBranch& branch_arg, const unsigned char childIdx_arg) const
        {
          return branch_arg.subNodes_[bufferSelector_][childIdx_arg];
        }

        /** \brief Retrieve a child node pointer for child node at childIdx in specific octree buffer.
         *  \param branch_arg: reference to octree branch class
         *  \param bufferSelector_arg: buffer selector
         *  \param childIdx_arg: index to child node
         *  \return pointer to octree child node class
         * */
        inline const OctreeNode*
        getBranchChild (const OctreeBranch& branch_arg, const unsigned char bufferSelector_arg,
                        const unsigned char childIdx_arg) const
        {
          return branch_arg.subNodes_[bufferSelector_arg][childIdx_arg];
        }

        /** \brief Check if branch is pointing to a particular child node for current octree buffer
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool
        branchHasChild (const OctreeBranch& branch_arg, const unsigned char childIdx_arg) const
        {
          return (branch_arg.subNodes_[bufferSelector_][childIdx_arg] != NULL);
        }

        /** \brief Check if branch is pointing to a particular child node in specific octree buffer
         *  \param branch_arg: reference to octree branch class
         *  \param bufferSelector_arg: buffer selector
         *  \param childIdx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool
        branchHasChild (const OctreeBranch& branch_arg, const unsigned char bufferSelector_arg,
                        const unsigned char childIdx_arg) const
        {
          return (branch_arg.subNodes_[bufferSelector_arg][childIdx_arg] != NULL);

        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers for current buffer
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node information
         * */
        inline char
        getBranchBitPattern (const OctreeBranch& branch_arg) const
        {
          unsigned char i;
          char nodeBits;

          // create bit pattern
          nodeBits = 0;
          for (i = 0; i < 8; i++)
          {
            nodeBits |= (!!branch_arg.subNodes_[bufferSelector_][i]) << i;
          }

          return nodeBits;
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers in specific buffer
         *  \param branch_arg: reference to octree branch class
         *  \param bufferSelector_arg: buffer selector
         *  \return a single byte with 8 bits of child node information
         * */
        inline char
        getBranchBitPattern (const OctreeBranch& branch_arg, const unsigned char bufferSelector_arg) const
        {
          unsigned char i;
          char nodeBits;

          // create bit pattern
          nodeBits = 0;
          for (i = 0; i < 8; i++)
          {
            nodeBits |= (!!branch_arg.subNodes_[bufferSelector_arg][i]) << i;
          }

          return nodeBits;
        }

        /** \brief Generate XOR bit pattern reflecting differences between the two octree buffers
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node XOR difference information
         * */
        inline char
        getBranchXORBitPattern (const OctreeBranch& branch_arg) const
        {
          unsigned char i;
          char nodeBits[2];

          // create bit pattern for both buffers
          nodeBits[0] = nodeBits[1] = 0;

          for (i = 0; i < 8; i++)
          {
            nodeBits[0] |= (!!branch_arg.subNodes_[0][i]) << i;
            nodeBits[1] |= (!!branch_arg.subNodes_[1][i]) << i;
          }

          return nodeBits[0] ^ nodeBits[1];
        }

        /** \brief Test if branch changed between previous and current buffer
         *  \param branch_arg: reference to octree branch class
         *  \return "true", if child node information differs between current and previous octree buffer
         * */
        inline bool
        hasBranchChanges (const OctreeBranch& branch_arg) const
        {
          return (getBranchXORBitPattern (branch_arg) > 0);
        }

        /** \brief Assign new child node to branch for current buffer
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newChild_arg: pointer to new child node
         * */
        inline void
        setBranchChild (OctreeBranch& branch_arg, const unsigned char childIdx_arg, const OctreeNode * newChild_arg)
        {
          branch_arg.subNodes_[bufferSelector_][childIdx_arg] = newChild_arg;
        }

        /** \brief Assign new child node to branch in specific buffer
         *  \param branch_arg: reference to octree branch class
         *  \param bufferSelector_arg: buffer selector
         *  \param childIdx_arg: index to child node
         *  \param newChild_arg: pointer to new child node
         * */
        inline void
        setBranchChild (OctreeBranch& branch_arg, const unsigned char bufferSelector_arg,
                        const unsigned char childIdx_arg, const OctreeNode * newChild_arg)
        {
          branch_arg.subNodes_[bufferSelector_arg][childIdx_arg] = newChild_arg;
        }

        /** \brief Delete child node and all its subchilds from octree in current buffer
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         * */
        inline void
        deleteBranchChild (OctreeBranch& branch_arg, const unsigned char childIdx_arg)
        {
          if (branchHasChild (branch_arg, childIdx_arg))
          {
            const OctreeNode* branchChild;
            branchChild = getBranchChild (branch_arg, childIdx_arg);

            switch (branchChild->getNodeType ())
            {
              case BRANCH_NODE:
                // free child branch recursively
                deleteBranch (*(OctreeBranch*)branchChild);

                // push unused branch to branch pool
                unusedBranchesPool_.push_back ((OctreeBranch*)branchChild);
                break;

              case LEAVE_NODE:

                // push unused leaf to branch pool
                unusedLeafsPool_.push_back ((OctreeLeaf*)branchChild);
                break;
            }

            // set branch child pointer to NULL
            setBranchChild (branch_arg, childIdx_arg, NULL);
          }
        }

        /** \brief Delete child node and all its subchilds from octree in specific buffer
         *  \param branch_arg: reference to octree branch class
         *  \param bufferSelector_arg: buffer selector
         *  \param childIdx_arg: index to child node
         * */
        inline void
        deleteBranchChild (OctreeBranch& branch_arg, const unsigned char bufferSelector_arg,
                           const unsigned char childIdx_arg)
        {
          if (branchHasChild (branch_arg, bufferSelector_arg, childIdx_arg))
          {
            const OctreeNode* branchChild;
            branchChild = getBranchChild (branch_arg, bufferSelector_arg, childIdx_arg);

            switch (branchChild->getNodeType ())
            {
              case BRANCH_NODE:
                // free child branch recursively
                deleteBranch (*(OctreeBranch*)branchChild);

                // push unused branch to branch pool
                unusedBranchesPool_.push_back ((OctreeBranch*)branchChild);
                break;

              case LEAVE_NODE:

                // push unused leaf to branch pool
                unusedLeafsPool_.push_back ((OctreeLeaf*)branchChild);
                break;
            }

            // set branch child pointer to NULL
            setBranchChild (branch_arg, bufferSelector_arg, childIdx_arg, NULL);

          }
        }

        /** \brief Delete branch and all its subchilds from octree (both buffers)
         *  \param branch_arg: reference to octree branch class
         * */
        inline void
        deleteBranch (OctreeBranch& branch_arg)
        {
          char i;

          // delete all branch node children
          for (i = 0; i < 8; i++)
          {

            if (getBranchChild (branch_arg, 0, i) == getBranchChild (branch_arg, 1, i))
            {
              // reference was copied - there is only one child instance to be deleted
              deleteBranchChild (branch_arg, 0, i);

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
         *  \param newBranchChild_arg: write a pointer of new branch child to this reference
         * */
        inline void
        createBranchChild (OctreeBranch& branch_arg, const unsigned char childIdx_arg,
                           OctreeBranch*& newBranchChild_arg)
        {
          if (!unusedBranchesPool_.size ())
          {
            newBranchChild_arg = (OctreeBranch*)new OctreeBranch ();
          }
          else
          {
            newBranchChild_arg = unusedBranchesPool_.back ();
            unusedBranchesPool_.pop_back ();
            branchReset (*newBranchChild_arg);
          }

          setBranchChild (branch_arg, childIdx_arg, (OctreeNode*)newBranchChild_arg);
        }

        /** \brief Return a new branch class and receive a pointer to it
         *  \param newBranchChild_arg: writes a pointer of new branch child to this reference
         * */
        inline void
        createBranch (OctreeBranch*& newBranchChild_arg)
        {

          if (!unusedBranchesPool_.size ())
          {
            // branch pool is empty
            // we need to create a new octree branch class
            newBranchChild_arg = (OctreeBranch*)new OctreeBranch ();
          }
          else
          {
            // reuse branch from branch pool
            newBranchChild_arg = unusedBranchesPool_.back ();
            unusedBranchesPool_.pop_back ();
            branchReset (*newBranchChild_arg);
          }
        }

        /** \brief Fetch and add a new leaf child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newBranchChild_arg: writes a pointer of new leaf child to this reference
         * */
        inline void
        createLeafChild (OctreeBranch& branch_arg, const unsigned char childIdx_arg, OctreeLeaf*& newLeafChild_arg)
        {
          if (!unusedLeafsPool_.size ())
          {
            // leaf pool is empty
            // we need to create a new octree leaf class
            newLeafChild_arg = (OctreeLeaf*)new OctreeLeaf ();
          }
          else
          {
            // reuse leaf node from branch pool
            newLeafChild_arg = unusedLeafsPool_.back ();
            unusedLeafsPool_.pop_back ();
          }

          newLeafChild_arg->reset ();

          setBranchChild (branch_arg, childIdx_arg, (OctreeNode*)newLeafChild_arg);
        }

        /** \brief Reset branch class
         *  \param branch_arg: reference to octree branch class
         * */
        inline void
        branchReset (OctreeBranch& branch_arg)
        {
          memset (branch_arg.subNodes_, 0, sizeof(branch_arg.subNodes_));
        }

        /** \brief Delete all branch nodes and leaf nodes from octree node pools
         * */
        inline void
        poolCleanUp ()
        {
          // delete all branch instances from branch pool
          while (!unusedBranchesPool_.empty ())
          {
            delete (unusedBranchesPool_.back ());
            unusedBranchesPool_.pop_back ();
          }

          // delete all leaf instances from leaf pool
          while (!unusedLeafsPool_.empty ())
          {
            delete (unusedLeafsPool_.back ());
            unusedLeafsPool_.pop_back ();
          }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive octree methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /** \brief Recursivly search for a leaf node at octree key. If leaf node does not exist, it will be created.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param branchReset_arg: Reset pointer array of current branch
         *  \return pointer to leaf node class
         **/
        LeafT*
        getLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg, OctreeBranch* branch_arg,
                          bool branchReset_arg);

        /** \brief Recursivly search for a given leaf node and return a pointer.
         *  \note  If leaf node does not exist, a NULL pointer is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \return pointer to leaf node class. Returns NULL if leaf node is not found.
         **/
        LeafT*
        findLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg, OctreeBranch* branch_arg) const;

        /** \brief Recursivly search and delete leaf node
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param branch_arg: current branch node
         *  \return "true" if branch does not contain any childs; "false" otherwise. This indicates if current branch can be deleted.
         **/
        bool
        deleteLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg, OctreeBranch* branch_arg);

        /** \brief Recursivly explore the octree and output binary octree description
         *  \param binaryTreeOut_arg: reference to output stream
         *  \param branch_arg: current branch node
         **/
        void
        serializeTreeRecursive (std::ostream& binaryTreeOut_arg, OctreeBranch* branch_arg);

        /** \brief Recursivly explore the octree and output binary octree description together with a vector of leaf node DataT content.
         *  \param binaryTreeOut_arg: reference to output stream
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: writes DataT content to this DataT vector reference.
         **/
        void
        serializeTreeRecursive (std::ostream& binaryTreeOut_arg, OctreeBranch* branch_arg, OctreeKey& key_arg,
                                std::vector<DataT>& dataVector_arg);

        /** \brief Recursivly explore the octree and output DataT objects to DataT vector.
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: DataT objects from leaf nodes are written to this DataT vector reference.
         **/
        void
            serializeLeafsRecursive (OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                     std::vector<DataT>& dataVector_arg);

        /** \brief Recursivly explore the octree and output DataT objects of leafs that do not exist in previous buffer.
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: DataT objects from leaf nodes are written to this DataT vector reference.
         *  \param minPointsPerLeaf_arg: minimum amount of points required within leaf node to become serialized.
         **/
        void
        serializeNewLeafsRecursive (OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                    std::vector<DataT>& dataVector_arg, const int minPointsPerLeaf_arg = 0);

        /** \brief Rebuild an octree based on binary XOR octree description.
         *  \param binaryTreeIn_arg: reference to input stream
         *  \param branch_arg: current branch node
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         **/
        void
        deserializeTreeRecursive (std::istream& binaryTreeIn_arg, OctreeBranch* branch_arg,
                                  const unsigned int depthMask_arg, const OctreeKey& key_arg);

        /** \brief Rebuild an octree based on binary XOR octree description and DataT objects for leaf node initialization.
         *  \param binaryTreeIn_arg: reference to input stream
         *  \param branch_arg: current branch node
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param dataVectorIterator_arg: iterator pointing to current DataT object to be added to a leaf node
         *  \param dataVectorEndIterator_arg: iterator pointing to last object in DataT input vector.
         **/
        void
        deserializeTreeRecursive (std::istream& binaryTreeIn_arg, OctreeBranch* branch_arg,
                                  const unsigned int depthMask_arg, const OctreeKey& key,
                                  typename std::vector<DataT>::iterator& dataVectorIterator_arg,
                                  typename std::vector<DataT>::const_iterator& dataVectorEndIterator_arg);

        /** \brief Recursivly explore the octree and remove unused branch and leaf nodes
         *  \param branch_arg: current branch node
         **/
        void
        treeCleanUpRecursive (OctreeBranch* branch_arg);

        /** \brief Test if octree is able to dynamically change its depth. This is required for adaptive bounding box adjustment.
         *  \return "false" - not resizeable due to XOR serialization
         **/
        inline const bool
        octreeCanResize ()
        {
          return false;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /** \brief Amount of leaf nodes   **/
        unsigned int leafCount_;

        /** \brief Pointer to root branch node of octree   **/
        OctreeBranch* rootNode_;

        /** \brief Depth mask based on octree depth   **/
        unsigned int depthMask_;

        /** \brief Vector pools of unused branch nodes   **/
        std::vector<OctreeBranch*> unusedBranchesPool_;

        /** \brief Vector pools of unused leaf nodes   **/
        std::vector<LeafT*> unusedLeafsPool_;

        /** \brief Currently active octree buffer  **/
        unsigned char bufferSelector_;

        // flags indicating if unused branches and leafs might exist in previous buffer
        bool resetTree_;
        bool treeDirtyFlag_;

      };
  }
}

//#include "impl/octree2buf_base.hpp"

#endif

