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

#ifndef OCTREE_LOWMEM_TREE_BASE_H
#define OCTREE_LOWMEM_TREE_BASE_H

#include <cstddef>
#include <vector>

#include "octree_nodes.h"

#include "octree_iterator.h"

namespace pcl
{
  namespace octree
  {
    /** \brief Octree class
      *  \note The tree depth defines the maximum amount of octree voxels / leaf nodes (should be initially defined).
      *  \note All leaf nodes are addressed by integer indices.
      *  \note Note: The tree depth equates to the bit length of the voxel indices.
      *  \ingroup octree
      *  \author Julius Kammerl (julius@kammerl.de)
      */
    template<typename DataT, typename LeafT = OctreeLeafDataT<DataT> >
    class OctreeLowMemBase
    {
      // iterators are friends
      friend class OctreeIteratorBase<DataT, LeafT, OctreeLowMemBase> ;
      friend class OctreeDepthFirstIterator<DataT, LeafT, OctreeLowMemBase> ;
      friend class OctreeBreadthFirstIterator<DataT, LeafT, OctreeLowMemBase> ;
      friend class OctreeLeafNodeIterator<DataT, LeafT, OctreeLowMemBase> ;

      public:

        // Octree iterators
        typedef OctreeDepthFirstIterator<DataT, LeafT, OctreeLowMemBase> Iterator;
        typedef const OctreeDepthFirstIterator<DataT, LeafT, OctreeLowMemBase> ConstIterator;

        typedef OctreeLeafNodeIterator<DataT, LeafT, OctreeLowMemBase> LeafNodeIterator;
        typedef const OctreeLeafNodeIterator<DataT, LeafT, OctreeLowMemBase> ConstLeafNodeIterator;

        typedef OctreeDepthFirstIterator<DataT, LeafT, OctreeLowMemBase> DepthFirstIterator;
        typedef const OctreeDepthFirstIterator<DataT, LeafT, OctreeLowMemBase> ConstDepthFirstIterator;
        typedef OctreeBreadthFirstIterator<DataT, LeafT, OctreeLowMemBase> BreadthFirstIterator;
        typedef const OctreeBreadthFirstIterator<DataT, LeafT, OctreeLowMemBase> ConstBreadthFirstIterator;

        /** \brief Empty constructor. */
        OctreeLowMemBase ();

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeLowMemBase ();

        /** \brief copy constructor. */
        OctreeLowMemBase (const OctreeLowMemBase& source) : 
          leafCount_ (source.leafCount_),
        	branchCount_ (source.branchCount_),
        	objectCount_ (source.objectCount_),
        	rootNode_ (new (OctreeBranch) (*(source.rootNode_))),
        	depthMask_ (source.depthMask_),
       	  octreeDepth_ (source.octreeDepth_),
          unusedBranchesPool_ (), unusedLeafsPool_ ()
        {
        }

        /** \brief Copy operator. */
        inline OctreeLowMemBase&
        operator = (const OctreeLowMemBase &source)
        {
          *this = source;
          return (*this);
        }

        /** \brief Set the maximum amount of voxels per dimension.
          * \param[in] maxVoxelIndex_arg maximum amount of voxels per dimension
          */
        void
        setMaxVoxelIndex (unsigned int maxVoxelIndex_arg);

        /** \brief Set the maximum depth of the octree.
          *  \param[in] depth_arg maximum depth of octree
          */
        void
        setTreeDepth (unsigned int depth_arg);

        /** \brief Get the maximum depth of the octree.
          * \return maximum depth of octree
          */
        inline unsigned int
        getTreeDepth () const
        {
          return (this->octreeDepth_);
        }

        /** \brief Add a const DataT element to leaf node at (idxX, idxY, idxZ). If leaf node does not exist, it is created and added to the octree.
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \param data_arg: const reference to DataT object to be added.
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
         *  \param idxZ_arg: index of leaf node in the Z axis.
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

        /** \brief Return the amount of existing branches in the octree.
         *  \return amount of branch nodes.
         * */
        inline unsigned int
        getBranchCount () const
        {
          return branchCount_;
        }

        /** \brief Delete the octree structure and its leaf nodes.
         * */
        void
        deleteTree ( );

        /** \brief Serialize octree into a binary output vector describing its branch node structure.
         *  \param binaryTreeOut_arg: reference to output vector for writing binary tree structure.
         *  \param doXOREncoding_arg: dummy argument to be consistent with the octree compression interface. As only a single octree is managed in memory, no XOR encoding is performed.
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg, bool doXOREncoding_arg = false);

        /** \brief Serialize octree into a binary output vector describing its branch node structure and and push all DataT elements stored in the octree to a vector.
         * \param binaryTreeOut_arg: reference to output vector for writing binary tree structure.
         * \param dataVector_arg: reference of DataT vector that receives a copy of all DataT objects in the octree
         * \param doXOREncoding_arg: dummy argument to be consistent with the octree compression interface. As only a single octree is managed in memory, no XOR encoding is performed.
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg, std::vector<DataT>& dataVector_arg,
                       bool doXOREncoding_arg = false);

        /** \brief Outputs a vector of all DataT elements that are stored within the octree leaf nodes.
         *  \param dataVector_arg: reference to DataT vector that receives a copy of all DataT objects in the octree.
         * */
        void
        serializeLeafs (std::vector<DataT>& dataVector_arg);

        /** \brief Deserialize a binary octree description vector and create a corresponding octree structure. Leaf nodes are initialized with getDataTByKey(..).
         *  \param binaryTreeIn_arg: reference to input vector for reading binary tree structure.
         *  \param doXORDecoding_arg: dummy argument to be consistent with the octree compression interface. As only a single octree is managed in memory, no XOR encoding is performed.
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg, bool doXORDecoding_arg = false);

        /** \brief Deserialize a binary octree description and create a corresponding octree structure. Leaf nodes are initialized with DataT elements from the dataVector.
         *  \param binaryTreeIn_arg: reference to inpvectoream for reading binary tree structure.
         *  \param dataVector_arg: reference to DataT vector that provides DataT objects for initializing leaf nodes.
         *  \param doXORDecoding_arg: dummy argument to be consistent with the octree compression interface. As only a single octree is managed in memory, no XOR encoding is performed.
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg, std::vector<DataT>& dataVector_arg,
                         bool doXORDecoding_arg = false);

        /** \brief Deserialize a binary octree description vector and create a corresponding octree structure. Leaf nodes are initialized with getDataTByKey(..). Generated DataT objects are copied to output vector.
         *  \param binaryTreeIn_arg: reference to input vector for reading binary tree structure.
         *  \param dataVector_arg: reference to DataT vector that receives a copy of generated DataT objects.
         * */
        void
        deserializeTreeAndOutputLeafData (std::vector<char>& binaryTreeIn_arg, std::vector<DataT>& dataVector_arg);

        /** \brief Switch buffers interface for compression. As this octree implementation manages only a single buffer, it just deletes the current octree. */
        void
        switchBuffers ()
        {
          this->deleteTree ();
        }

      protected:

        /** \brief @b Octree key class
          *  \note Octree keys contain integer indices for each coordinate axis in order to address an octree leaf node.
          *  \author Julius Kammerl (julius@kammerl.de)
          */
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
            unsigned int x;
            unsigned int y;
            unsigned int z;
        };

        /** \brief @b Octree branch class.
          * \note It stores 8 pointers to its child nodes.
          * \author Julius Kammerl (julius@kammerl.de)
          */
        class OctreeBranch : public OctreeNode
        {
          // OctreeBase is a friend!
          friend class OctreeLowMemBase;

          public:
            /** \brief Constructor for initializing child node pointer array. */
            OctreeBranch () : occupancyByte_ (0), subNodes_ ()
            {
              this->reset ();
            }

            /** \brief Copy constructor. */
            OctreeBranch (const OctreeBranch &source) : occupancyByte_ (0), subNodes_ ()
            {
              *this = source;
            }

            /** \brief Copy operator. */
            inline OctreeBranch&
            operator = (const OctreeBranch &source)
            {
              *this = source;
              return (*this);
            }

            /** \brief Reset Octree Branch */
            void
            reset ()
            {
              // if pointers to childs exist, we delete them
              if (occupancyByte_)
                delete[] subNodes_;

              // reset occupancy byte
              occupancyByte_ = 0;
            }

            /** \brief Empty deconstructor. */
            virtual
            ~OctreeBranch ()
            {
            }

            /** \brief deep copy function */
            virtual OctreeNode *
            deepCopy () const
            {
              return ((OctreeNode*) new OctreeBranch (*this));
            }

            /** \brief Get the type of octree node. Returns BRANCH_NODE type
              * \return Returns BRANCH_NODE type.
              */
            virtual node_type_t
            getNodeType () const
            {
              return (BRANCH_NODE);
            }

          private:
            unsigned char occupancyByte_;
            OctreeNode** subNodes_;
        };

        typedef LeafT OctreeLeaf;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Retrieve root node */
        const OctreeNode*
        getRootNode () const
        {
          return this->rootNode_;
        }

        /** \brief Virtual method for generating an octree key for a given DataT object.
         *  \param data_arg: reference to DataT object
         *  \param key_arg: write generated octree key to this octree key reference
         *  \return "true" if octree could be generated based on DataT object; "false" otherwise
         * */
        virtual bool
        genOctreeKeyForDataT (const DataT &data_arg, OctreeKey &key_arg) const
        {
          // Silence compiler warnings
          (void)data_arg;
          (void)key_arg;
          // this class cannot relate DataT objects to octree keys
          return (false);
        }

        /** \brief Virtual method for initializing new leaf node during deserialization (in case no DataT information is provided)
          * \param[in] key_arg write generated octree key to this octree key reference
          * \param[out] data_arg generated DataT object
          * \return "true" if DataT object could be generated; "false" otherwise
          */
        virtual bool
        genDataTByOctreeKey (const OctreeKey &key_arg, DataT &data_arg) const
        {
          // Silence compiler warnings
          (void)key_arg;
          (void)data_arg;
          // this class cannot relate DataT objects to octree keys
          return (false);
        }

        /** \brief Generate an octree key
          * \param[in] idxX_arg index of leaf node in the X axis.
          * \param[in] idxY_arg index of leaf node in the Y axis.
          * \param[in] idxZ_arg index of leaf node in the Z axis.
          * \param[out] key_arg write new octree key to this reference.
          */
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
          if (leaf)
          {
            leaf->setData (data_arg);
            objectCount_++;
          }
        }

        /** \brief Find leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf node. If leaf node is not found, this pointer returns 0.
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
          return getLeafRecursive (key_arg, depthMask_, rootNode_);
        }

        /** \brief Check for existance of a leaf node in the octree
         *  \param key_arg: octree key addressing a leaf node.
         *  \return "true" if leaf node is found; "false" otherwise
         * */
        inline bool
        existLeaf (const OctreeKey& key_arg) const
        {
          return (findLeafRecursive (key_arg, depthMask_, rootNode_) != 0);
        }

        /** \brief Remove leaf node from octree
         *  \param key_arg: octree key addressing a leaf node.
         * */
        inline void
        removeLeaf (const OctreeKey& key_arg)
        {
          deleteLeafRecursive (key_arg, depthMask_, rootNode_);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Branch node accessor inline functions
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Retrieve a child node pointer for child node at childIdx.
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return pointer to octree child node class
         * */
        inline const OctreeNode*
        getBranchChild (const OctreeBranch& branch_arg, const unsigned char childIdx_arg) const
        {
          // check if branch child exists according to occupancyBtye
          if (branch_arg.occupancyByte_ & (1 << childIdx_arg))
          {
            unsigned int c, idx;

            // retrieve array index of child pointer
            idx = 0;
            for (c = 0; c < childIdx_arg; c++)
            {
              if (branch_arg.occupancyByte_ & (1 << c))
                idx++;
            }

            return branch_arg.subNodes_[idx];

          }

          return 0;
        }

        /** \brief Check if branch is pointing to a particular child node
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool
        branchHasChild (const OctreeBranch& branch_arg, const unsigned char childIdx_arg) const
        {
          // test occupancyByte for child existence
          return ((branch_arg.occupancyByte_ & (1 << childIdx_arg)) != 0);
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node information
         * */
        inline char
        getBranchBitPattern (const OctreeBranch& branch_arg) const
        {
          return branch_arg.occupancyByte_;
        }

        /** \brief Assign new child node to branch
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newChild_arg: pointer to new child node
         * */
        inline void
        setBranchChild (OctreeBranch& branch_arg, const unsigned char childIdx_arg, OctreeNode * newChild_arg)
        {

          if (!(branch_arg.occupancyByte_ & (1 << childIdx_arg)))
          {
            // branch child does not exists
            unsigned int c, idx, total;

            // retrieve total amount of child pointers and retrieve index of new child pointer
            idx = 0;
            total = 0;
            for (c = 0; c < 8; c++)
            {
              if (branch_arg.occupancyByte_ & (1 << c))
              {
                if (c < childIdx_arg)
                  idx++;

                total++;
              }
            }

            if (newChild_arg != 0)
            {
              // a new child pointer is to be added to child array

              // allocate new pointer array
              OctreeNode** newNodes = new OctreeNode*[total + 1];

              // copy pointers to new pointer array
              memcpy (&newNodes[0], &branch_arg.subNodes_[0], idx * sizeof(OctreeNode*));
              newNodes[idx] = newChild_arg;
              memcpy (&newNodes[idx + 1], &branch_arg.subNodes_[idx], (total - idx) * sizeof(OctreeNode*));

              // delete old pointer array
              if (total > 0)
                delete[] branch_arg.subNodes_;

              // set new child pointer array
              branch_arg.subNodes_ = newNodes;

              // update occupancyByte
              branch_arg.occupancyByte_ |= (1 << childIdx_arg);
            }

          }
          else
          {
            // child pointer already exists
            unsigned int c, idx, total;

            // pointer set to 0 -> delete child pointer
            if (newChild_arg == 0)
            {

              // retrieve total amount of child pointers and detect index of child pointer to be removed
              idx = 0;
              total = 0;
              for (c = 0; c < 8; c++)
              {
                if (branch_arg.occupancyByte_ & (1 << c))
                {
                  if (c < childIdx_arg)
                    idx++;

                  total++;
                }
              }

              if (total == 1)
              {
                // if only a single pointer exists, simply delete the data structure
                delete[] branch_arg.subNodes_;

                branch_arg.subNodes_ = 0;
                branch_arg.occupancyByte_ = 0;
              }
              else
              {
                // allocate new pointer array with reduced size
                OctreeNode** newNodes = new OctreeNode*[total - 1];

                // copy old pointer references to new array
                memcpy (&newNodes[0], &branch_arg.subNodes_[0], idx * sizeof(OctreeNode*));
                memcpy (&newNodes[idx], &branch_arg.subNodes_[idx + 1], (total - (idx + 1)) * sizeof(OctreeNode*));

                // delete previous pointer array and update "branch_arg.subNodes_"
                delete[] branch_arg.subNodes_;
                branch_arg.subNodes_ = newNodes;

                // update occupancyByte by removing the corresponding bit
                branch_arg.occupancyByte_ &= ~(1 << childIdx_arg);

              }

            }
            else
            {
              // update existing child pointer

              // retrieve index of child pointer to be updated
              idx = 0;
              for (c = 0; c < childIdx_arg; c++)
              {
                if (branch_arg.occupancyByte_ & (1 << c))
                {
                  idx++;
                }
              }

              // update pointer
              branch_arg.subNodes_[idx] = newChild_arg;
            }

          }

        }

        /** \brief Delete child node and all its subchilds from octree
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
                break;
              case LEAF_NODE:
                break;
              default:
		break;

            }

            // delete branch child
            delete (branchChild);

            // set branch child pointer to 0
            setBranchChild (branch_arg, childIdx_arg, 0);
          }
        }

        /** \brief Delete branch and all its subchilds from octree
         *  \param branch_arg: reference to octree branch class
         * */
        inline void
        deleteBranch (OctreeBranch& branch_arg)
        {
          char i;

          // delete all branch node children
          for (i = 0; i < 8; i++)
          {
            deleteBranchChild (branch_arg, i);
          }
        }

        /** \brief Create a new branch class and receive a pointer to it
         *  \param newBranchChild_arg: writes a pointer of new branch child to this reference
         * */
        inline void
        createBranch (OctreeBranch*& newBranchChild_arg)
        {

          // we need to create a new octree branch class
          newBranchChild_arg = (OctreeBranch*)new OctreeBranch ();

        }

        /** \brief Create and add a new branch child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newBranchChild_arg: write a pointer of new branch child to this reference
         * */
        inline void
        createBranchChild (OctreeBranch& branch_arg, const unsigned char childIdx_arg,
                           OctreeBranch*& newBranchChild_arg)
        {
          createBranch (newBranchChild_arg);
          setBranchChild (branch_arg, childIdx_arg, (OctreeNode*)newBranchChild_arg);
        }

        /** \brief Create and add a new leaf child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newLeafChild_arg: writes a pointer of new leaf child to this reference
         * */
        inline void
        createLeafChild (OctreeBranch& branch_arg, const unsigned char childIdx_arg, OctreeLeaf*& newLeafChild_arg)
        {

          // we need to create a new octree leaf class
          newLeafChild_arg = (OctreeLeaf*)new OctreeLeaf ();
          newLeafChild_arg->reset ();

          setBranchChild (branch_arg, childIdx_arg, (OctreeNode*)newLeafChild_arg);

        }

        /** \brief Reset branch class
         *  \param branch_arg: reference to octree branch class
         * */
        inline void
        branchReset (OctreeBranch& branch_arg)
        {
          branch_arg.reset ();
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive octree methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /** \brief Recursively search for a leaf node at octree key. If leaf node does not exist, it will be created.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \return pointer to leaf node class
         **/
        LeafT*
        getLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg, OctreeBranch* branch_arg);

        /** \brief Recursively search for a given leaf node and return a pointer.
         *  \note  If leaf node does not exist, a 0 pointer is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \return pointer to leaf node class. Returns 0 if leaf node is not found.
         **/
        LeafT*
        findLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg, OctreeBranch* branch_arg) const;

        /** \brief Recursively search and delete leaf node
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param branch_arg: current branch node
         *  \return "true" if branch does not contain any childs; "false" otherwise. This indicates if current branch can be deleted, too.
         **/
        bool
        deleteLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg, OctreeBranch* branch_arg);

        /** \brief Recursively explore the octree and output binary octree description
         *  \param binaryTreeOut_arg: binary output vector
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         **/
        void
        serializeTreeRecursive (std::vector<char>& binaryTreeOut_arg, const OctreeBranch* branch_arg, const OctreeKey& key_arg);

        /** \brief Recursively explore the octree and output binary octree description together with a vector of leaf node DataT content.
         *  \param binaryTreeOut_arg: binary output vector
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: writes DataT content to this DataT vector iterator.
         **/
        void
        serializeTreeRecursive (std::vector<char>& binaryTreeOut_arg, const OctreeBranch* branch_arg,
                                const OctreeKey& key_arg, typename std::vector<DataT>& dataVector_arg);

        /** \brief Recursively explore the octree and output DataT objects to DataT vector.
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: DataT objects from leaf nodes are written to this DataT vector .
         **/
        void
        serializeLeafsRecursive (const OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                 typename std::vector<DataT>& dataVector_arg);

        /** \brief Rebuild an octree based on binary octree description.
         *  \param binaryTreeIn_arg: iterator to input vector
         *  \param branch_arg: current branch node
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param key_arg: reference to an octree key
         **/
        void
        deserializeTreeRecursive (typename std::vector<char>::const_iterator& binaryTreeIn_arg,
                                  OctreeBranch* branch_arg, const unsigned int depthMask_arg, const OctreeKey& key_arg);

        /** \brief Rebuild an octree based on binary octree description and DataT objects for leaf node initialization.
         *  \param binaryTreeIn_arg: iterator to input vector
         *  \param branch_arg: current branch node
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param key_arg: reference to an octree key
         *  \param dataVectorIterator_arg: iterator pointing to current DataT object to be added to a leaf node
         *  \param dataVectorEndIterator_arg: iterator pointing to last object in DataT input vector.
         **/
        void
        deserializeTreeRecursive (typename std::vector<char>::const_iterator& binaryTreeIn_arg,
                                  OctreeBranch* branch_arg, const unsigned int depthMask_arg, const OctreeKey& key_arg,
                                  typename std::vector<DataT>::const_iterator& dataVectorIterator_arg,
                                  typename std::vector<DataT>::const_iterator& dataVectorEndIterator_arg);

        /** \brief Rebuild an octree based on binary octree description and output generated DataT objects.
         *  \param binaryTreeIn_arg: iterator to input vector
         *  \param branch_arg: current branch node
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: iterator of DataT vector that receives a copy of generated DataT objects.
         **/
        void
        deserializeTreeAndOutputLeafDataRecursive (typename std::vector<char>::const_iterator& binaryTreeIn_arg,
                                                   OctreeBranch* branch_arg, const unsigned int depthMask_arg,
                                                   const OctreeKey& key_arg,
                                                   typename std::vector<DataT>& dataVector_arg);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Serialization callbacks
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Decode leaf node data during serialization
         *  \param leaf_arg: reference to new leaf node
         *  \param key_arg: octree key of new leaf node
         **/
        virtual void
        serializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg);

        /** \brief Decode leaf node data during serialization
         *  \param leaf_arg: reference to new leaf node
         *  \param key_arg: octree key of new leaf node
         *  \param dataVector_arg: DataT objects from leaf are pushed to this DataT vector
         **/
        virtual void
        serializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg, std::vector<DataT>& dataVector_arg);

        /** \brief Initialize leaf nodes during deserialization
         *  \param leaf_arg: reference to new leaf node
         *  \param key_arg: octree key of new leaf node
         **/
        virtual void
        deserializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg);

        /** \brief Initialize leaf nodes during deserialization
         *  \param leaf_arg: reference to new leaf node
         *  \param key_arg: octree key of new leaf node
         *  \param dataVectorIterator_arg: iterator pointing to current DataT object to be added to the new leaf node
         *  \param dataVectorEndIterator_arg: iterator pointing to last object in DataT input vector.
         **/
        virtual void
        deserializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg,
                                 typename std::vector<DataT>::const_iterator& dataVectorIterator_arg,
                                 typename std::vector<DataT>::const_iterator& dataVectorEndIterator_arg);

        /** \brief Initialize leaf nodes during deserialization
         *  \param leaf_arg: reference to new leaf node
         *  \param key_arg: octree key of new leaf node
         *  \param dataVector_arg: generated DataT objects are pushed to this DataT vector
         **/
        virtual void
        deserializeTreeAndSerializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey & key_arg,
                                                 std::vector<DataT>& dataVector_arg);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Helpers
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Helper function to calculate the binary logarithm
         * \param n_arg: some value
         * \return binary logarithm (log2) of argument n_arg
         */
        inline double
        Log2 (double n_arg)
        {
           return log( n_arg ) / log( 2.0 );
        }

        /** \brief Test if octree is able to dynamically change its depth. This is required for adaptive bounding box adjustment.
         *  \return "true"
         **/
        inline bool
        octreeCanResize ()
        {
          return (true);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Amount of leaf nodes   **/
        unsigned int leafCount_;

        /** \brief Amount of branch nodes   **/
        unsigned int branchCount_;

        /** \brief Amount of objects assigned to leaf nodes   **/
        unsigned int objectCount_;

        /** \brief Pointer to root branch node of octree   **/
        OctreeBranch* rootNode_;

        /** \brief Depth mask based on octree depth   **/
        unsigned int depthMask_;

        /** \brief Octree depth */
        unsigned int octreeDepth_;

        /** \brief Vector pools of unused branch nodes   **/
        std::vector<OctreeBranch*> unusedBranchesPool_;

        /** \brief Vector pools of unused leaf nodes   **/
        std::vector<LeafT*> unusedLeafsPool_;

      };
  }
}

//#include "impl/octree_base.hpp"

#endif

