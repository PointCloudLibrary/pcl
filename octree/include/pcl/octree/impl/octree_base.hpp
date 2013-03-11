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

#ifndef PCL_OCTREE_BASE_HPP
#define PCL_OCTREE_BASE_HPP

#include <vector>

#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT>
    OctreeBase<LeafContainerT, BranchContainerT>::OctreeBase () :
      leafCount_ (0),
      branchCount_ (1),
      rootNode_ (new BranchNode ()),
      depthMask_ (0),
      octreeDepth_ (0),
      dynamic_depth_enabled_(false),
      maxKey_ ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT>
    OctreeBase<LeafContainerT, BranchContainerT>::~OctreeBase ()
    {
      // deallocate tree structure
      deleteTree ();
      delete (rootNode_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::setMaxVoxelIndex (unsigned int maxVoxelIndex_arg)
    {
      unsigned int treeDepth;

      assert (maxVoxelIndex_arg>0);

      // tree depth == amount of bits of maxVoxels
      treeDepth = std::max ((std::min (static_cast<unsigned int> (OCT_MAXTREEDEPTH), 
                                       static_cast<unsigned int> (std::ceil (Log2 (maxVoxelIndex_arg))))),
                                       static_cast<unsigned int> (0));

      // define depthMask_ by setting a single bit to 1 at bit position == tree depth
      depthMask_ = (1 << (treeDepth - 1));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT>
    void
    OctreeBase<LeafContainerT, BranchContainerT>::setTreeDepth (unsigned int depth_arg)
    {
      assert(depth_arg>0);

      // set octree depth
      octreeDepth_ = depth_arg;

      // define depthMask_ by setting a single bit to 1 at bit position == tree depth
      depthMask_ = (1 << (depth_arg - 1));

      // define max. keys
      maxKey_.x = maxKey_.y = maxKey_.z = (1 << depth_arg) - 1;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
     template<typename LeafContainerT, typename BranchContainerT>  LeafContainerT*
     OctreeBase<LeafContainerT, BranchContainerT>::findLeaf (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg)
     {
       // generate key
       OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

       // check if key exist in octree
       return ( findLeaf (key));
     }

    //////////////////////////////////////////////////////////////////////////////////////////////
     template<typename LeafContainerT, typename BranchContainerT>  LeafContainerT*
     OctreeBase<LeafContainerT, BranchContainerT>::createLeaf (unsigned int idxX_arg, unsigned int idxY_arg, unsigned int idxZ_arg)
     {
       // generate key
       OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

       // check if key exist in octree
       return ( createLeaf (key));
     }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> bool
    OctreeBase<LeafContainerT, BranchContainerT>::existLeaf (unsigned int idxX_arg, unsigned int idxY_arg,
                                         unsigned int idxZ_arg) const
    {
      // generate key
      OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

      // check if key exist in octree
      return ( existLeaf (key));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::removeLeaf (unsigned int idxX_arg, unsigned int idxY_arg,
                                          unsigned int idxZ_arg)
    {
      // generate key
      OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

      // check if key exist in octree
      deleteLeafRecursive (key, depthMask_, rootNode_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::deleteTree ()
    {

      if (rootNode_)
      {
        // reset octree
        deleteBranch (*rootNode_);
        leafCount_ = 0;
        branchCount_ = 1;

      }

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::serializeTree (std::vector<char>& binaryTreeOut_arg)
    {

      OctreeKey newKey;

      // clear binary vector
      binaryTreeOut_arg.clear ();
      binaryTreeOut_arg.reserve (this->branchCount_);

      serializeTreeRecursive (rootNode_, newKey, &binaryTreeOut_arg, 0 );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::serializeTree (std::vector<char>& binaryTreeOut_arg, std::vector<LeafContainerT*>& dataVector_arg)
    {

      OctreeKey newKey;

      // clear output vectors
      binaryTreeOut_arg.clear ();
      dataVector_arg.clear ();

      dataVector_arg.reserve (this->leafCount_);
      binaryTreeOut_arg.reserve (this->branchCount_);

      serializeTreeRecursive (rootNode_, newKey, &binaryTreeOut_arg, &dataVector_arg );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::serializeLeafs (std::vector<LeafContainerT*>& dataVector_arg)
    {
      OctreeKey newKey;

      // clear output vector
      dataVector_arg.clear ();

      dataVector_arg.reserve(this->leafCount_);

      serializeTreeRecursive (rootNode_, newKey, 0, &dataVector_arg );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::deserializeTree (std::vector<char>& binaryTreeIn_arg)
    {
      OctreeKey newKey;

      // free existing tree before tree rebuild
      deleteTree ();

      //iterator for binary tree structure vector
      std::vector<char>::const_iterator binaryTreeVectorIterator = binaryTreeIn_arg.begin ();
      std::vector<char>::const_iterator binaryTreeVectorIteratorEnd = binaryTreeIn_arg.end ();

      deserializeTreeRecursive (rootNode_, depthMask_, newKey,
          binaryTreeVectorIterator, binaryTreeVectorIteratorEnd, 0, 0);

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::deserializeTree (std::vector<char>& binaryTreeIn_arg,
                                               std::vector<LeafContainerT*>& dataVector_arg)
    {
      OctreeKey newKey;

      // set data iterator to first element
      typename std::vector<LeafContainerT*>::const_iterator dataVectorIterator = dataVector_arg.begin ();

      // set data iterator to last element
      typename std::vector<LeafContainerT*>::const_iterator dataVectorEndIterator = dataVector_arg.end ();

      // free existing tree before tree rebuild
      deleteTree ();

      //iterator for binary tree structure vector
      std::vector<char>::const_iterator binaryTreeVectorIterator = binaryTreeIn_arg.begin ();
      std::vector<char>::const_iterator binaryTreeVectorIteratorEnd = binaryTreeIn_arg.end ();

      deserializeTreeRecursive (rootNode_, depthMask_, newKey,
          binaryTreeVectorIterator, binaryTreeVectorIteratorEnd, &dataVectorIterator, &dataVectorEndIterator);

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT>
      unsigned int
      OctreeBase<LeafContainerT, BranchContainerT>::createLeafRecursive (const OctreeKey& key_arg,
                                                                         unsigned int depthMask_arg,
                                                                         BranchNode* branch_arg,
                                                                         LeafNode*& returnLeaf_arg,
                                                                         BranchNode*& leafParent_arg)
      {
        // index to branch child
      unsigned char childIdx;

      // find branch child from key
      childIdx = key_arg.getChildIdxWithDepthMask(depthMask_arg);

      OctreeNode* childNode = (*branch_arg)[childIdx];

      if (!childNode)
      {
        if ((!dynamic_depth_enabled_) && (depthMask_arg > 1)) {
          // if required branch does not exist -> create it
          BranchNode* childBranch = createBranchChild (*branch_arg, childIdx);

          branchCount_++;

          // recursively proceed with indexed child branch
          return createLeafRecursive (key_arg, depthMask_arg / 2, childBranch, returnLeaf_arg, leafParent_arg);

        } else {
          // if leaf node at childIdx does not exist
          LeafNode* leaf_node = createLeafChild (*branch_arg, childIdx);
          returnLeaf_arg = leaf_node;
          leafParent_arg = branch_arg;
          leafCount_++;
        }
      } else {

        // Node exists already
        switch (childNode->getNodeType()) {
          case BRANCH_NODE:
            // recursively proceed with indexed child branch
            return createLeafRecursive (key_arg, depthMask_arg / 2, static_cast<BranchNode*> (childNode), returnLeaf_arg, leafParent_arg);
            break;

          case LEAF_NODE:
            LeafNode* childLeaf = static_cast<LeafNode*> (childNode);
            returnLeaf_arg = childLeaf;
            leafParent_arg = branch_arg;
            /*
            // get amount of objects in leaf container
            size_t leafObjCount = childLeaf->getSize ();

            if (! ( (!maxObjsPerLeaf_) || (!depthMask_arg) ) && (leafObjCount >= maxObjsPerLeaf_) )
            {
              // leaf node needs to be expanded

              // copy leaf data
              std::vector<DataT> leafData;
              leafData.reserve(leafObjCount);

              childLeaf->getData (leafData);

              // delete current leaf node
              deleteBranchChild(*branch_arg,childIdx);
              leafCount_ --;

              // create new branch node
              BranchNode* childBranch;
              createBranchChild (*branch_arg, childIdx, childBranch);
              branchCount_ ++;

              typename std::vector<DataT>::const_iterator lData = leafData.begin();
              typename std::vector<DataT>::const_iterator lDataEnd = leafData.end();

              // add data to new branch
              OctreeKey dataKey;

              while (lData!=lDataEnd) {
                // get data object
                const DataT& data = *lData;
                ++lData;

                // generate new key for data object
                if (this->genOctreeKeyForDataT(data, dataKey)) {
                  LeafNode* newLeaf;

                  createLeafRecursive (dataKey, depthMask_arg / 2, data, childBranch, newLeaf);
                  // add data to leaf
                  newLeaf->setData (data);
                  objectCount_++;
                }
              }

              // and return new leaf node
              createLeafRecursive (key_arg, depthMask_arg / 2, data_arg, childBranch, returnLeaf_arg);

              // correct object counter
              objectCount_ -= leafObjCount;
            }
            */
            break;
        }

      }

      return (depthMask_arg>>1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::findLeafRecursive (
        const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg, LeafContainerT*& result_arg) const
    {
      // index to branch child
      unsigned char childIdx;

      // find branch child from key
      childIdx = key_arg.getChildIdxWithDepthMask(depthMask_arg);

      OctreeNode* childNode = (*branch_arg)[childIdx];

      if (childNode) {
        switch (childNode->getNodeType()) {
          case BRANCH_NODE:
            // we have not reached maximum tree depth
            BranchNode* childBranch;
            childBranch = static_cast<BranchNode*> (childNode);

            findLeafRecursive (key_arg, depthMask_arg / 2, childBranch, result_arg);
            break;

          case LEAF_NODE:
            // return existing leaf node
            LeafNode* childLeaf;
            childLeaf =  static_cast<LeafNode*> (childNode);

            result_arg = childLeaf->getContainerPtr();
            break;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> bool
    OctreeBase<LeafContainerT, BranchContainerT>::deleteLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg,
                                                            BranchNode* branch_arg)
    {
      // index to branch child
      unsigned char childIdx;
      // indicates if branch is empty and can be safely removed
      bool bNoChilds;

      // find branch child from key
      childIdx = key_arg.getChildIdxWithDepthMask(depthMask_arg);

      OctreeNode* childNode = (*branch_arg)[childIdx];

      if (childNode) {
        switch (childNode->getNodeType()) {

          case BRANCH_NODE:
            BranchNode* childBranch;
            childBranch = static_cast<BranchNode*> (childNode);

            // recursively explore the indexed child branch
            bNoChilds = deleteLeafRecursive (key_arg, depthMask_arg / 2, childBranch);

            if (!bNoChilds)
            {
              // child branch does not own any sub-child nodes anymore -> delete child branch
              deleteBranchChild(*branch_arg, childIdx);
              branchCount_--;
            }
            break;

          case LEAF_NODE:
            // return existing leaf node

            // our child is a leaf node -> delete it
            deleteBranchChild (*branch_arg, childIdx);
            leafCount_--;
            break;
        }
      }

      // check if current branch still owns children
      bNoChilds = false;
      for (childIdx = 0; (!bNoChilds) && (childIdx < 8); childIdx++)
      {
        bNoChilds = branch_arg->hasChild(childIdx);
      }
      // return true if current branch can be deleted
      return (bNoChilds);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void OctreeBase<
        LeafContainerT, BranchContainerT>::serializeTreeRecursive (const BranchNode* branch_arg, OctreeKey& key_arg,
            std::vector<char>* binaryTreeOut_arg,
            typename std::vector<LeafContainerT*>* dataVector_arg) const
    {

      // child iterator
      unsigned char childIdx;
      char nodeBitPattern;

      // branch occupancy bit pattern
      nodeBitPattern = getBranchBitPattern (*branch_arg);

      // write bit pattern to output vector
      if (binaryTreeOut_arg)
        binaryTreeOut_arg->push_back (nodeBitPattern);

      // iterate over all children
      for (childIdx = 0; childIdx < 8; childIdx++)
      {

        // if child exist
        if (branch_arg->hasChild(childIdx))
        {
          // add current branch voxel to key
          key_arg.pushBranch(childIdx);

          OctreeNode *childNode = branch_arg->getChildPtr(childIdx);

          switch (childNode->getNodeType ())
          {
            case BRANCH_NODE:
            {
              // recursively proceed with indexed child branch
              serializeTreeRecursive (
                  static_cast<const BranchNode*> (childNode), key_arg,
                  binaryTreeOut_arg, dataVector_arg);
              break;
            }
            case LEAF_NODE:
            {
              LeafNode* childLeaf = static_cast<LeafNode*> (childNode);

              if (dataVector_arg)
                dataVector_arg->push_back(childLeaf->getContainerPtr());

              // we reached a leaf node -> execute serialization callback
              serializeTreeCallback (**childLeaf, key_arg);
              break;
            }
            default:
              break;
           }

          // pop current branch voxel from key
          key_arg.popBranch();
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeBase<LeafContainerT, BranchContainerT>::deserializeTreeRecursive (BranchNode* branch_arg,
        unsigned int depthMask_arg, OctreeKey& key_arg,
        typename std::vector<char>::const_iterator& binaryTreeIT_arg,
        typename std::vector<char>::const_iterator& binaryTreeIT_End_arg,
        typename std::vector<LeafContainerT*>::const_iterator* dataVectorIterator_arg,
        typename std::vector<LeafContainerT*>::const_iterator* dataVectorEndIterator_arg)
    {
      // child iterator
      unsigned char childIdx;
      char nodeBits;

      if (binaryTreeIT_arg != binaryTreeIT_End_arg ) {
        // read branch occupancy bit pattern from input vector
        nodeBits = (*binaryTreeIT_arg);
        binaryTreeIT_arg++;

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          // if occupancy bit for childIdx is set..
          if (nodeBits & (1 << childIdx))
          {
            // add current branch voxel to key
            key_arg.pushBranch(childIdx);

            if (depthMask_arg > 1)
            {
              // we have not reached maximum tree depth
              BranchNode * newBranch = createBranchChild (*branch_arg, childIdx);

              branchCount_++;

              // recursively proceed with new child branch
              deserializeTreeRecursive (newBranch, depthMask_arg / 2, key_arg, binaryTreeIT_arg,binaryTreeIT_End_arg,  dataVectorIterator_arg,
                                        dataVectorEndIterator_arg);
            }
            else
            {
              // we reached leaf node level

              LeafNode* childLeaf = createLeafChild (*branch_arg, childIdx);

              if (dataVectorIterator_arg
                  && (*dataVectorIterator_arg != *dataVectorEndIterator_arg))
              {
                LeafContainerT& container = **childLeaf;
                container =  ***dataVectorIterator_arg;
                ++*dataVectorIterator_arg;
              }

              leafCount_++;

              // execute deserialization callback
              deserializeTreeCallback (**childLeaf, key_arg);
            }

            // pop current branch voxel from key
            key_arg.popBranch();
          }
        }
      }
    }

  }
}

#define PCL_INSTANTIATE_OctreeBase(T) template class PCL_EXPORTS pcl::octree::OctreeBase<T>;

#endif

