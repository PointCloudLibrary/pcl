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

#ifndef OCTREE_TREE_BASE_H
#define OCTREE_TREE_BASE_H

#include <cstddef>
#include <vector>

#include "octree_nodes.h"
#include "octree_container.h"
#include "octree_key.h"
#include "octree_iterator.h"
#include "octree_node_pool.h"

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
    template<typename DataT, typename LeafT = OctreeContainerDataT<DataT>,
        typename BranchT = OctreeContainerEmpty<DataT> >
    class OctreeBase
    {

      public:

        typedef OctreeBase<DataT, OctreeContainerDataT<DataT>, OctreeContainerEmpty<DataT> > SingleObjLeafContainer;
        typedef OctreeBase<DataT, OctreeContainerDataTVector<DataT>, OctreeContainerEmpty<DataT> > MultipleObjsLeafContainer;

        typedef OctreeBase<DataT, LeafT, BranchT> OctreeT;

        // iterators are friends
        friend class OctreeIteratorBase<DataT, OctreeT> ;
        friend class OctreeDepthFirstIterator<DataT, OctreeT> ;
        friend class OctreeBreadthFirstIterator<DataT, OctreeT> ;
        friend class OctreeLeafNodeIterator<DataT, OctreeT> ;

        typedef OctreeBranchNode<BranchT> BranchNode;
        typedef OctreeLeafNode<LeafT> LeafNode;

        // Octree iterators
        typedef OctreeDepthFirstIterator<DataT, OctreeT> Iterator;
        typedef const OctreeDepthFirstIterator<DataT, OctreeT> ConstIterator;

        typedef OctreeLeafNodeIterator<DataT, OctreeT> LeafNodeIterator;
        typedef const OctreeLeafNodeIterator<DataT, OctreeT> ConstLeafNodeIterator;

        typedef OctreeDepthFirstIterator<DataT, OctreeT> DepthFirstIterator;
        typedef const OctreeDepthFirstIterator<DataT, OctreeT> ConstDepthFirstIterator;
        typedef OctreeBreadthFirstIterator<DataT, OctreeT> BreadthFirstIterator;
        typedef const OctreeBreadthFirstIterator<DataT, OctreeT> ConstBreadthFirstIterator;


        /** \brief Empty constructor. */
        OctreeBase ();

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeBase ();

        /** \brief Copy constructor. */
        OctreeBase (const OctreeBase& source) :
          leafCount_ (source.leafCount_),
          branchCount_ (source.branchCount_),
          objectCount_ (source.objectCount_),
          rootNode_ (new (BranchNode) (*(source.rootNode_))),
          depthMask_ (source.depthMask_),
          octreeDepth_ (source.octreeDepth_),
          maxKey_ (source.maxKey_),
          branchNodePool_ (),
          leafNodePool_ ()
        {
        }

        /** \brief Copy operator. */
        inline OctreeBase& 
        operator = (const OctreeBase &source)
        {
          leafCount_ = source.leafCount_;
          branchCount_ = source.branchCount_;
          objectCount_ = source.objectCount_;
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
        inline unsigned int
        getTreeDepth () const
        {
          return this->octreeDepth_;
        }

        /** \brief Enable dynamic octree structure
         *  \note Leaf nodes are kept as close to the root as possible and are only expanded if the number of DataT objects within a leaf node exceeds a fixed limit.
         *  \return maxObjsPerLeaf: maximum number of DataT objects per leaf
         * */
        inline void
        enableDynamicDepth ( size_t maxObjsPerLeaf )
        {
          assert(leafCount_==0);
          maxObjsPerLeaf_ = maxObjsPerLeaf;
        }

        /** \brief Add a const DataT element to leaf node at (idxX, idxY, idxZ). If leaf node does not exist, it is created and added to the octree.
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \param data_arg: const reference to DataT object to be added.
         * */
        void
        addData (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg,
             const DataT& data_arg);

        /** \brief Retrieve a DataT element from leaf node at (idxX, idxY, idxZ). It returns false if leaf node does not exist.
         *  \param idxX_arg: index of leaf node in the X axis.
         *  \param idxY_arg: index of leaf node in the Y axis.
         *  \param idxZ_arg: index of leaf node in the Z axis.
         *  \param data_arg: reference to DataT object that contains content of leaf node if search was successful.
         *  \return "true" if leaf node search is successful, otherwise it returns "false".
         * */
        bool
        getData (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg, DataT& data_arg) const ;

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
        inline std::size_t
        getLeafCount () const
        {
          return leafCount_;
        }

        /** \brief Return the amount of existing branches in the octree.
         *  \return amount of branch nodes.
         * */
        inline std::size_t
        getBranchCount () const
        {
          return branchCount_;
        }

        /** \brief Delete the octree structure and its leaf nodes.
         *  \param freeMemory_arg: if "true", allocated octree nodes are deleted, otherwise they are pushed to the octree node pool
         * */
        void
        deleteTree ( bool freeMemory_arg = true );

        /** \brief Serialize octree into a binary output vector describing its branch node structure.
         *  \param binaryTreeOut_arg: reference to output vector for writing binary tree structure.
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg);

        /** \brief Serialize octree into a binary output vector describing its branch node structure and push all DataT elements stored in the octree to a vector.
         * \param binaryTreeOut_arg: reference to output vector for writing binary tree structure.
         * \param dataVector_arg: reference of DataT vector that receives a copy of all DataT objects in the octree
         * */
        void
        serializeTree (std::vector<char>& binaryTreeOut_arg, std::vector<DataT>& dataVector_arg);

        /** \brief Outputs a vector of all DataT elements that are stored within the octree leaf nodes.
         *  \param dataVector_arg: reference to DataT vector that receives a copy of all DataT objects in the octree.
         * */
        void
        serializeLeafs (std::vector<DataT>& dataVector_arg);

        /** \brief Deserialize a binary octree description vector and create a corresponding octree structure. Leaf nodes are initialized with getDataTByKey(..).
         *  \param binaryTreeIn_arg: reference to input vector for reading binary tree structure.
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg);

        /** \brief Deserialize a binary octree description and create a corresponding octree structure. Leaf nodes are initialized with DataT elements from the dataVector.
         *  \param binaryTreeIn_arg: reference to input vector for reading binary tree structure.
         *  \param dataVector_arg: reference to DataT vector that provides DataT objects for initializing leaf nodes.
         * */
        void
        deserializeTree (std::vector<char>& binaryTreeIn_arg, std::vector<DataT>& dataVector_arg);

      protected:
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Virtual method for generating an octree key for a given DataT object.
          * \param[in] data_arg reference to DataT object
          * \param[in] key_arg write generated octree key to this octree key reference
          * \return "true" if octree could be generated based on DataT object; "false" otherwise
          */
        virtual bool
        genOctreeKeyForDataT (const DataT &, OctreeKey &) const
        {
          // this class cannot relate DataT objects to octree keys
          return (false);
        }

        /** \brief Virtual method for initializing new leaf node during deserialization (in case no DataT information is provided)
          * \param[in] key_arg write generated octree key to this octree key reference
          * \param[in] data_arg generated DataT object
          * \return "true" if DataT object could be generated; "false" otherwise
          */
        virtual bool
        genDataTByOctreeKey (const OctreeKey &, DataT &) const
        {
          // this class cannot relate DataT objects to octree keys
          return (false);
        }

        /** \brief Add DataT object to leaf node at octree key.
         *  \param key_arg: octree key addressing a leaf node.
         *  \param data_arg: DataT object to be added.
         * */
        inline void
        addData (const OctreeKey& key_arg, const DataT& data_arg)
        {
          addDataToLeafRecursive (key_arg, depthMask_,data_arg, rootNode_);
        }

        /** \brief Find leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf node. If leaf node is not found, this pointer returns 0.
         * */
        inline LeafNode*
        findLeaf (const OctreeKey& key_arg) const
        {
          LeafNode* result = 0;
          findLeafRecursive (key_arg, depthMask_, rootNode_, result);
          return result;
        }

        /** \brief Check for existance of a leaf node in the octree
         *  \param key_arg: octree key addressing a leaf node.
         *  \return "true" if leaf node is found; "false" otherwise
         * */
        inline bool
        existLeaf (const OctreeKey& key_arg) const
        {
          LeafNode* leafNode = 0;
          if (key_arg <= maxKey_)
            findLeafRecursive (key_arg, depthMask_, rootNode_, leafNode);
          return (leafNode != 0);
        }

        /** \brief Remove leaf node from octree
         *  \param key_arg: octree key addressing a leaf node.
         * */
        inline void
        removeLeaf (const OctreeKey& key_arg)
        {
          if (key_arg <= maxKey_)
            deleteLeafRecursive (key_arg, depthMask_, rootNode_);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Branch node accessor inline functions
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
        inline bool
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
        inline OctreeNode*
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
        inline void setBranchChildPtr (BranchNode& branch_arg,
            unsigned char childIdx_arg, OctreeNode* newChild_arg)
        {
          branch_arg[childIdx_arg] = newChild_arg;
        }

        /** \brief Get data from octree node
         *  \param node_arg: node in octree
         *  \param data_arg: obtain single DataT object from octree node
         * */
        inline void getDataFromOctreeNode (const OctreeNode* node_arg,
            DataT& data_arg)
        {
          if (node_arg->getNodeType () == LEAF_NODE)
          {
            const LeafT* leafContainer = dynamic_cast<const LeafT*> (node_arg);
            leafContainer->getData (data_arg);
          }
          else
          {
            const BranchT* branchContainer =
                dynamic_cast<const BranchT*> (node_arg);
            branchContainer->getData (data_arg);
          }
        }

        /** \brief Get data from octree node
         *  \param node_arg: node in octree
         *  \param data_arg: obtain vector of all DataT objects stored in octree node
         * */
        inline void getDataFromOctreeNode (const OctreeNode* node_arg,
            std::vector<DataT>& data_arg)
        {
          if (node_arg->getNodeType () == LEAF_NODE)
          {
            const LeafT* leafContainer = dynamic_cast<const LeafT*> (node_arg);
            leafContainer->getData (data_arg);
          }
          else
          {
            const BranchT* branchContainer =
                dynamic_cast<const BranchT*> (node_arg);
            branchContainer->getData (data_arg);
          }
        }

        /** \brief Get data size of octree node container
         *  \param node_arg: node in octree
         *  \return data_arg: returns number of DataT objects stored in node container
         * */
        inline size_t getDataSizeFromOctreeNode (const OctreeNode* node_arg)
        {
          size_t nodeSize;
          if (node_arg->getNodeType () == LEAF_NODE)
          {
            const LeafT* leafContainer = dynamic_cast<const LeafT*> (node_arg);
            nodeSize = leafContainer->getSize ();
          }
          else
          {
            const BranchT* branchContainer =
                dynamic_cast<const BranchT*> (node_arg);
            nodeSize = branchContainer->getSize ();
          }
          return nodeSize;
        }
        /** \brief Generate bit pattern reflecting the existence of child node pointers
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node information
         * */
        inline char
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
        inline void
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
                // push unused branch to branch pool
                branchNodePool_.pushNode (
                    static_cast<BranchNode*> (branchChild));
              }
                break;

              case LEAF_NODE:
              {
                // push unused leaf to branch pool
                leafNodePool_.pushNode (static_cast<LeafNode*> (branchChild));
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
        inline void
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
         *  \param newBranchChild_arg: write a pointer of new branch child to this reference
         * */
        inline void createBranchChild (BranchNode& branch_arg,
            unsigned char childIdx_arg, BranchNode*& newBranchChild_arg)
        {
          newBranchChild_arg = branchNodePool_.popNode ();
          branch_arg[childIdx_arg] =
              static_cast<OctreeNode*> (newBranchChild_arg);
        }

        /** \brief Create and add a new leaf child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param childIdx_arg: index to child node
         *  \param newLeafChild_arg: writes a pointer of new leaf child to this reference
         * */
        inline void
        createLeafChild (BranchNode& branch_arg, unsigned char childIdx_arg, LeafNode*& newLeafChild_arg)
        {
          newLeafChild_arg = leafNodePool_.popNode();

          branch_arg[childIdx_arg] = static_cast<OctreeNode*> (newLeafChild_arg);
        }

        /** \brief Reset branch class
         *  \param branch_arg: reference to octree branch class
         * */
        inline void
        branchReset (BranchNode& branch_arg)
        {
          branch_arg.reset ();
        }

        /** \brief Delete all branch nodes and leaf nodes from octree node pools
         * */
        inline void
        poolCleanUp ()
        {
          branchNodePool_.deletePool();
          leafNodePool_.deletePool();
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive octree methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Create a leaf node at octree key. If leaf node does already exist, it is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param data_arg data to be added
         *  \param branch_arg: current branch node
         **/
        void
        addDataToLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, const DataT& data_arg, BranchNode* branch_arg);

        /** \brief Recursively search for a given leaf node and return a pointer.
         *  \note  If leaf node does not exist, a 0 pointer is returned.
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param result_arg: pointer to leaf node class
         **/
        void
        findLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg, LeafNode*& result_arg) const;

        /** \brief Recursively search and delete leaf node
         *  \param key_arg: reference to an octree key
         *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param branch_arg: current branch node
         *  \return "true" if branch does not contain any childs; "false" otherwise. This indicates if current branch can be deleted, too.
         **/
        bool
        deleteLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg);

        /** \brief Recursively explore the octree and output binary octree description together with a vector of leaf node DataT content.
          *  \param binaryTreeOut_arg: binary output vector
          *  \param branch_arg: current branch node
          *  \param key_arg: reference to an octree key
         *  \param dataVector_arg: writes DataT content to this DataT vector.
         **/
        void
        serializeTreeRecursive (const BranchNode* branch_arg, OctreeKey& key_arg,
            std::vector<char>* binaryTreeOut_arg,
            typename std::vector<DataT>* dataVector_arg) const;

         /** \brief Rebuild an octree based on binary XOR octree description and DataT objects for leaf node initialization.
          *  \param binaryTreeIn_arg: iterator to input vector
          *  \param branch_arg: current branch node
          *  \param depthMask_arg: depth mask used for octree key analysis and branch depth indicator
          *  \param key_arg: reference to an octree key
          *  \param dataVectorIterator_arg: iterator pointing to current DataT object to be added to a leaf node
          *  \param dataVectorEndIterator_arg: iterator pointing to last object in DataT input vector.
          **/

         void
         deserializeTreeRecursive (BranchNode* branch_arg,
             unsigned int depthMask_arg, OctreeKey& key_arg,
             typename std::vector<char>::const_iterator& binaryTreeIT_arg,
             typename std::vector<char>::const_iterator& binaryTreeIT_End_arg,
             typename std::vector<DataT>::const_iterator* dataVectorIterator_arg,
             typename std::vector<DataT>::const_iterator* dataVectorEndIterator_arg);


         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Serialization callbacks
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Callback executed for every leaf node during serialization
         **/
        virtual void serializeTreeCallback (const LeafNode &,
            const OctreeKey &) const
        {

        }

        /** \brief Callback executed for every leaf node during deserialization
         **/
        virtual void deserializeTreeCallback (LeafNode&, const OctreeKey&)
        {

        }

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
        std::size_t leafCount_;

        /** \brief Amount of branch nodes   **/
        std::size_t branchCount_;

        /** \brief Amount of objects assigned to leaf nodes   **/
        std::size_t objectCount_;

        /** \brief Pointer to root branch node of octree   **/
        BranchNode* rootNode_;

        /** \brief Amount of DataT objects per leafNode before expanding branch
         *  \note zero indicates a fixed/maximum depth octree structure
         * **/
        std::size_t maxObjsPerLeaf_;

        /** \brief Depth mask based on octree depth   **/
        unsigned int depthMask_;

        /** \brief Octree depth */
        unsigned int octreeDepth_;

        /** \brief key range */
        OctreeKey maxKey_;

        /** \brief Pool of unused branch nodes   **/
        OctreeNodePool<BranchNode> branchNodePool_;

        /** \brief Pool of unused branch nodes   **/
        OctreeNodePool<LeafNode> leafNodePool_;
    };
  }
}

//#include "impl/octree_base.hpp"

#endif

