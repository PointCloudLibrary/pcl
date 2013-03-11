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

#ifndef PCL_OCTREE_TREE_BASE_H
#define PCL_OCTREE_TREE_BASE_H

#include <cstddef>
#include <vector>

#include "octree_nodes.h"
#include "octree_container.h"
#include "octree_key.h"
#include "octree_iterator.h"

// maximum depth of octree as we are using "unsigned int" octree keys / bit masks
#define OCT_MAXTREEDEPTH ( sizeof(size_t) * 8  )

namespace pcl
{
  namespace octree
  {
    /** \brief Octree class
      * \note The tree depth defines the maximum amount of octree voxels / leaf nodes (should be initially defined).
      * \note All leaf nodes are addressed by integer indices.
      * \note Note: The tree depth equates to the bit length of the voxel indices.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename LeafContainerT = int,
        typename BranchContainerT = OctreeContainerEmpty >
    class OctreeBase
    {

      public:

        typedef OctreeBase<LeafContainerT, BranchContainerT> OctreeT;

        typedef OctreeBranchNode<BranchContainerT> BranchNode;
        typedef OctreeLeafNode<LeafContainerT> LeafNode;

        typedef BranchContainerT BranchContainer;
        typedef LeafContainerT LeafContainer;

        // iterators are friends
        friend class OctreeIteratorBase<OctreeT> ;
        friend class OctreeDepthFirstIterator<OctreeT> ;
        friend class OctreeBreadthFirstIterator<OctreeT> ;
        friend class OctreeLeafNodeIterator<OctreeT> ;

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
        OctreeBase ();

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeBase ();

        /** \brief Copy constructor. */
        OctreeBase (const OctreeBase& source) :
          leafCount_ (source.leafCount_),
          branchCount_ (source.branchCount_),
          rootNode_ (new (BranchNode) (*(source.rootNode_))),
          depthMask_ (source.depthMask_),
          octreeDepth_ (source.octreeDepth_),
          dynamic_depth_enabled_(source.dynamic_depth_enabled_),
          maxKey_ (source.maxKey_)
        {
        }

        /** \brief Copy operator. */
        OctreeBase&
        operator = (const OctreeBase &source)
        {
          leafCount_ = source.leafCount_;
          branchCount_ = source.branchCount_;
          rootNode_ = new (BranchNode) (*(source.rootNode_));
          depthMask_ = source.depthMask_;
          maxKey_ = source.maxKey_;
          octreeDepth_ = source.octreeDepth_;
          return (*this);
        }

        /** \brief Set the maximum amount of voxels per dimension.
          * \param[in] maxVoxelIndex_arg maximum amount of voxels per dimension
          */
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
        unsigned int
        getTreeDepth () const
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
        existLeaf (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg) const ;

        /** \brief Remove leaf node at (idxX_arg, idxY_arg, idxZ_arg).
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         * */
        void
        removeLeaf (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg);

        /** \brief Return the amount of existing leafs in the octree.
         *  \return amount of registered leaf nodes.
         * */
        std::size_t
        getLeafCount () const
        {
          return leafCount_;
        }

        /** \brief Return the amount of existing branches in the octree.
         *  \return amount of branch nodes.
         * */
        std::size_t
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
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg);

        /** \brief Serialize octree into a binary output vector describing its branch node structure and push all LeafContainerT elements stored in the octree to a vector.
         * \param binaryTreeOut_arg: reference to output vector for writing binary tree structure.
         * \param dataVector_arg: pointer to all LeafContainerT objects in the octree
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg, std::vector<LeafContainerT*>& dataVector_arg);

        /** \brief Outputs a vector of all LeafContainerT elements that are stored within the octree leaf nodes.
         *  \param dataVector_arg: pointers to LeafContainerT vector that receives a copy of all LeafContainerT objects in the octree.
         * */
        void
        serializeLeafs (std::vector<LeafContainerT*>& dataVector_arg);

        /** \brief Deserialize a binary octree description vector and create a corresponding octree structure. Leaf nodes are initialized with getDataTByKey(..).
         *  \param binaryTreeIn_arg: reference to input vector for reading binary tree structure.
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg);

        /** \brief Deserialize a binary octree description and create a corresponding octree structure. Leaf nodes are initialized with LeafContainerT elements from the dataVector.
         *  \param binaryTreeIn_arg: reference to input vector for reading binary tree structure.
         *  \param dataVector_arg: pointer to container vector.
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg, std::vector<LeafContainerT*>& dataVector_arg);

      protected:
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Create a leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf node
         * */
        LeafContainerT*
        createLeaf (const OctreeKey& key_arg)
        {

          LeafNode* leaf_node;
          BranchNode* leaf_node_parent;

          createLeafRecursive (key_arg, depthMask_ ,rootNode_, leaf_node, leaf_node_parent);

          LeafContainerT* ret = leaf_node->getContainerPtr();

          return ret;
        }

        /** \brief Find leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf node. If leaf node is not found, this pointer returns 0.
         * */
        LeafContainerT*
        findLeaf (const OctreeKey& key_arg) const
        {
          LeafContainerT* result = 0;
          findLeafRecursive (key_arg, depthMask_, rootNode_, result);
          return result;
        }

        /** \brief Check for existance of a leaf node in the octree
         *  \param key_arg: octree key addressing a leaf node.
         *  \return "true" if leaf node is found; "false" otherwise
         * */
        bool
        existLeaf (const OctreeKey& key_arg) const
        {
          return (findLeaf(key_arg) != 0);
        }

        /** \brief Remove leaf node from octree
         *  \param key_arg: octree key addressing a leaf node.
         * */
        void
        removeLeaf (const OctreeKey& key_arg)
        {
          if (key_arg <= maxKey_)
            deleteLeafRecursive (key_arg, depthMask_, rootNode_);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Branch node accessor functions
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Retrieve root node */
        OctreeNode*
        getRootNode () const
        {
          return this->rootNode_;
        }

        /** \brief Check if branch is pointing to a particular child node
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        bool
        branchHasChild (const BranchNode& branch_arg, unsigned char childIdx_arg) const
        {
          // test occupancyByte for child existence
          return (branch_arg.getChildPtr(childIdx_arg) != 0);
        }

        /** \brief Retrieve a child node pointer for child node at childIdx.
         * \param branch_arg: reference to octree branch class
         * \param childIdx_arg: index to child node
         * \return pointer to octree child node class
         */
        OctreeNode*
        getBranchChildPtr (const BranchNode& branch_arg,
            unsigned char childIdx_arg) const
        {
          return branch_arg.getChildPtr(childIdx_arg);
        }

        /** \brief Assign new child node to branch
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newChild_arg: pointer to new child node
         * */
        void setBranchChildPtr (BranchNode& branch_arg,
            unsigned char childIdx_arg, OctreeNode* newChild_arg)
        {
          branch_arg[childIdx_arg] = newChild_arg;
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node information
         * */
        char
        getBranchBitPattern (const BranchNode& branch_arg) const
        {
          unsigned char i;
          char nodeBits;

          // create bit pattern
          nodeBits = 0;
          for (i = 0; i < 8; i++) {
            const OctreeNode* child = branch_arg.getChildPtr(i);
            nodeBits |= static_cast<char> ((!!child) << i);
          }

          return (nodeBits);
        }

        /** \brief Delete child node and all its subchilds from octree
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         * */
        void
        deleteBranchChild (BranchNode& branch_arg, unsigned char childIdx_arg)
        {
          if (branch_arg.hasChild(childIdx_arg))
          {
            OctreeNode* branchChild = branch_arg[childIdx_arg];
            
            switch (branchChild->getNodeType ())
            {
              case BRANCH_NODE:
              {
                // free child branch recursively
                deleteBranch (*static_cast<BranchNode*> (branchChild));
                // delete branch node
                delete branchChild;
              }
                break;

              case LEAF_NODE:
              {
                // delete leaf node
                delete branchChild;
                break;
              }
              default:
                break;
            }

            // set branch child pointer to 0
            branch_arg[childIdx_arg] = 0;
          }
        }

        /** \brief Delete branch and all its subchilds from octree
         *  \param branch_arg: reference to octree branch class
         * */
        void
        deleteBranch (BranchNode& branch_arg)
        {
          char i;

          // delete all branch node children
          for (i = 0; i < 8; i++)
            deleteBranchChild (branch_arg, i);
        }

        /** \brief Create and add a new branch child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return  newBranchChild_arg: pointer of new branch child to this reference
         * */
        BranchNode* createBranchChild (BranchNode& branch_arg,
            unsigned char childIdx_arg)
        {
          BranchNode* newBranchChild = new BranchNode();
          branch_arg[childIdx_arg] =
              static_cast<OctreeNode*> (newBranchChild);

          return newBranchChild;
        }

        /** \brief Create and add a new leaf child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \return newLeafChild_arg: pointer of new leaf child to this reference
         * */
        LeafNode*
        createLeafChild (BranchNode& branch_arg, unsigned char childIdx_arg)
        {
          LeafNode* newLeafChild = new LeafNode();

          branch_arg[childIdx_arg] = static_cast<OctreeNode*> (newLeafChild);

          return newLeafChild;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive octree methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Create a leaf node at octree key. If leaf node does already exist, it is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param data_arg data to be added
         *  \param branch_arg: current branch node
         *  \param returnLeaf_arg: return pointer to leaf node
         *  \param leafParent_arg: return pointer to parent of leaf node
         *  \return depth mask at which leaf node was created
         **/
        unsigned int
        createLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg, LeafNode*& returnLeaf_arg, BranchNode*& leafParent_arg);

        /** \brief Recursively search for a given leaf node and return a pointer.
         *  \note  If leaf node does not exist, a 0 pointer is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param result_arg: pointer to leaf node class
         **/
        void
        findLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg, LeafContainerT*& result_arg) const;

        /** \brief Recursively search and delete leaf node
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param branch_arg: current branch node
         *  \return "true" if branch does not contain any childs; "false" otherwise. This indicates if current branch can be deleted, too.
         **/
        bool
        deleteLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg);

        /** \brief Recursively explore the octree and output binary octree description together with a vector of leaf node LeafContainerTs.
          *  \param binaryTreeOut_arg: binary output vector
          *  \param branch_arg: current branch node
          *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: writes LeafContainerT pointers to this LeafContainerT* vector.
         **/
        void
        serializeTreeRecursive (const BranchNode* branch_arg, OctreeKey& key_arg,
            std::vector<char>* binaryTreeOut_arg,
            typename std::vector<LeafContainerT*>* dataVector_arg) const;

         /** \brief Rebuild an octree based on binary XOR octree description and LeafContainerT objects for leaf node initialization.
          *  \param binaryTreeIn_arg: iterator to input vector
          *  \param branch_arg: current branch node
          *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
          *  \param key_arg: reference to an octree key
          *  \param dataVectorIterator_arg: iterator pointing to current LeafContainerT object to be added to a leaf node
          *  \param dataVectorEndIterator_arg: iterator pointing to last object in LeafContainerT input vector.
          **/

         void
         deserializeTreeRecursive (BranchNode* branch_arg,
             unsigned int depthMask_arg, OctreeKey& key_arg,
             typename std::vector<char>::const_iterator& binaryTreeIT_arg,
             typename std::vector<char>::const_iterator& binaryTreeIT_End_arg,
             typename std::vector<LeafContainerT*>::const_iterator* dataVectorIterator_arg,
             typename std::vector<LeafContainerT*>::const_iterator* dataVectorEndIterator_arg);


         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Serialization callbacks
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Callback executed for every leaf node during serialization
         **/
        virtual void serializeTreeCallback (LeafContainerT&,
            const OctreeKey &) const
        {

        }

        /** \brief Callback executed for every leaf node during deserialization
         **/
        virtual void deserializeTreeCallback (LeafContainerT&, const OctreeKey&)
        {

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Helpers
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Helper function to calculate the binary logarithm
         * \param n_arg: some value
         * \return binary logarithm (log2) of argument n_arg
         */
        double
        Log2 (double n_arg)
        {
          return log( n_arg ) / log( 2.0 );
        }

        /** \brief Test if octree is able to dynamically change its depth. This is required for adaptive bounding box adjustment.
         *  \return "true"
         **/
        bool
        octreeCanResize ()
        {
          return (true);
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

        /** \brief Octree depth */
        unsigned int octreeDepth_;


        /** \brief Enable dynamic_depth
         *  \note Leaf nodes are kept as close as possible to the root node */
        bool dynamic_depth_enabled_;

        /** \brief key range */
        OctreeKey maxKey_;
    };
  }
}

//#include "impl/octree_base.hpp"

#endif

