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

#ifndef OCTREE_BASE_HPP
#define OCTREE_BASE_HPP

#include <vector>

#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

// maximum depth of octree as we are using "unsigned int" octree keys / bit masks
#define OCT_MAXTREEDEPTH ( sizeof(size_t) * 8  )

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT>
    OctreeBase<DataT, LeafT, BranchT>::OctreeBase () :
      leafCount_ (0),
      branchCount_ (1),
      objectCount_ (0),
      rootNode_ (new BranchNode ()),
      maxObjsPerLeaf_(0),
      depthMask_ (0),
      octreeDepth_ (0),
      maxKey_ (),
      branchNodePool_ (),
      leafNodePool_ ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT>
    OctreeBase<DataT, LeafT, BranchT>::~OctreeBase ()
    {
      // deallocate tree structure
      deleteTree ();
      delete (rootNode_);
      poolCleanUp ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::setMaxVoxelIndex (unsigned int maxVoxelIndex_arg)
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
    template<typename DataT, typename LeafT, typename BranchT>
    void
    OctreeBase<DataT, LeafT, BranchT>::setTreeDepth (unsigned int depth_arg)
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
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::addData (unsigned int idxX_arg, unsigned int idxY_arg,
                                   unsigned int idxZ_arg, const DataT& data_arg)
    {
      // generate key
      OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

      // add data_arg to octree
      addData (key, data_arg);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT>bool
    OctreeBase<DataT, LeafT, BranchT>::getData (unsigned int idxX_arg, unsigned int idxY_arg,
                                   unsigned int idxZ_arg, DataT& data_arg) const
    {
      // generate key
      OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

      // search for leaf at key
      LeafNode* leaf = findLeaf (key);
      if (leaf)
      {
        // if successful, decode data to data_arg
        leaf->getData (data_arg);
      }

      // returns true on success
      return (leaf != 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> bool
    OctreeBase<DataT, LeafT, BranchT>::existLeaf (unsigned int idxX_arg, unsigned int idxY_arg,
                                         unsigned int idxZ_arg) const
    {
      // generate key
      OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

      // check if key exist in octree
      return ( existLeaf (key));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::removeLeaf (unsigned int idxX_arg, unsigned int idxY_arg,
                                          unsigned int idxZ_arg)
    {
      // generate key
      OctreeKey key (idxX_arg, idxY_arg, idxZ_arg);

      // check if key exist in octree
      deleteLeafRecursive (key, depthMask_, rootNode_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::deleteTree (bool freeMemory_arg )
    {

      if (rootNode_)
      {
        // reset octree
        deleteBranch (*rootNode_);
        leafCount_ = 0;
        branchCount_ = 1;
        objectCount_ = 0;

      }

      // delete node pool
      if (freeMemory_arg)
        poolCleanUp ();

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::serializeTree (std::vector<char>& binaryTreeOut_arg)
    {
      // serialization requires fixed octree depth
      // maxObjsPerLeaf_>0 indicates a dynamic octree structure
      assert (!maxObjsPerLeaf_);

      OctreeKey newKey;

      // clear binary vector
      binaryTreeOut_arg.clear ();
      binaryTreeOut_arg.reserve (this->branchCount_);

      serializeTreeRecursive (rootNode_, newKey, &binaryTreeOut_arg, 0 );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::serializeTree (std::vector<char>& binaryTreeOut_arg, std::vector<DataT>& dataVector_arg)
    {
      // serialization requires fixed octree depth
      // maxObjsPerLeaf_>0 indicates a dynamic octree structure
      assert (!maxObjsPerLeaf_);

      OctreeKey newKey;

      // clear output vectors
      binaryTreeOut_arg.clear ();
      dataVector_arg.clear ();

      dataVector_arg.reserve (this->objectCount_);
      binaryTreeOut_arg.reserve (this->branchCount_);

      serializeTreeRecursive (rootNode_, newKey, &binaryTreeOut_arg, &dataVector_arg );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::serializeLeafs (std::vector<DataT>& dataVector_arg)
    {
      // serialization requires fixed octree depth
      // maxObjsPerLeaf_>0 indicates a dynamic octree structure
      assert (!maxObjsPerLeaf_);

      OctreeKey newKey;

      // clear output vector
      dataVector_arg.clear ();

      dataVector_arg.reserve(this->objectCount_);

      serializeTreeRecursive (rootNode_, newKey, 0, &dataVector_arg );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::deserializeTree (std::vector<char>& binaryTreeIn_arg)
    {
      // serialization requires fixed octree depth
      // maxObjsPerLeaf_>0 indicates a dynamic octree structure
      assert (!maxObjsPerLeaf_);

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
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::deserializeTree (std::vector<char>& binaryTreeIn_arg,
                                               std::vector<DataT>& dataVector_arg)
    {
      // serialization requires fixed octree depth
      // maxObjsPerLeaf_>0 indicates a dynamic octree structure
      assert (!maxObjsPerLeaf_);

      OctreeKey newKey;

      // set data iterator to first element
      typename std::vector<DataT>::const_iterator dataVectorIterator = dataVector_arg.begin ();

      // set data iterator to last element
      typename std::vector<DataT>::const_iterator dataVectorEndIterator = dataVector_arg.end ();

      // free existing tree before tree rebuild
      deleteTree ();

      //iterator for binary tree structure vector
      std::vector<char>::const_iterator binaryTreeVectorIterator = binaryTreeIn_arg.begin ();
      std::vector<char>::const_iterator binaryTreeVectorIteratorEnd = binaryTreeIn_arg.end ();

      deserializeTreeRecursive (rootNode_, depthMask_, newKey,
          binaryTreeVectorIterator, binaryTreeVectorIteratorEnd, &dataVectorIterator, &dataVectorEndIterator);

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void OctreeBase<
        DataT, LeafT, BranchT>::addDataToLeafRecursive (
        const OctreeKey& key_arg, unsigned int depthMask_arg,
        const DataT& data_arg, BranchNode* branch_arg)
    {
      // index to branch child
      unsigned char childIdx;

      // find branch child from key
      childIdx = key_arg.getChildIdxWithDepthMask(depthMask_arg);

      // add data to branch node container
      branch_arg->setData (data_arg);

      OctreeNode* childNode = (*branch_arg)[childIdx];

      if (!childNode)
      {
        if ((!maxObjsPerLeaf_) && (depthMask_arg > 1)) {
          // if required branch does not exist -> create it
          BranchNode* childBranch;
          createBranchChild (*branch_arg, childIdx, childBranch);

          branchCount_++;

          // recursively proceed with indexed child branch
          addDataToLeafRecursive (key_arg, depthMask_arg / 2, data_arg, childBranch);

        } else {
          LeafNode* childLeaf;

          // if leaf node at childIdx does not exist
          createLeafChild (*branch_arg, childIdx, childLeaf);
          leafCount_++;

          // add data to leaf
          childLeaf->setData (data_arg);
          objectCount_++;
        }
      } else {

        // Node exists already
        switch (childNode->getNodeType()) {
          case BRANCH_NODE:
            // recursively proceed with indexed child branch
            addDataToLeafRecursive (key_arg, depthMask_arg / 2, data_arg, static_cast<BranchNode*> (childNode));
            break;

          case LEAF_NODE:
            LeafNode* childLeaf = static_cast<LeafNode*> (childNode);

            if ( (!maxObjsPerLeaf_) || (!depthMask_arg) )
            {
              // add data to leaf
              childLeaf->setData (data_arg);
              objectCount_++;
            } else {
              size_t leafObjCount = childLeaf->getSize();

              if (leafObjCount<maxObjsPerLeaf_) {
                // add data to leaf
                childLeaf->setData (data_arg);
                objectCount_++;
              } else {
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
                  const DataT& data = *lData++;

                  // generate new key for data object
                  if (this->genOctreeKeyForDataT(data, dataKey)) {
                    addDataToLeafRecursive (dataKey, depthMask_arg / 2, data, childBranch);
                  }
                }

                // and add new DataT object
                addDataToLeafRecursive (key_arg, depthMask_arg / 2, data_arg, childBranch);

                // correct object counter
                objectCount_ -= leafObjCount;
              }
            }
            break;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::findLeafRecursive (
        const OctreeKey& key_arg, unsigned int depthMask_arg, BranchNode* branch_arg, LeafNode*& result_arg) const
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
            result_arg = childLeaf;
            break;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> bool
    OctreeBase<DataT, LeafT, BranchT>::deleteLeafRecursive (const OctreeKey& key_arg, unsigned int depthMask_arg,
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

      // check if current branch still owns childs
      bNoChilds = false;
      for (childIdx = 0; (!bNoChilds) && (childIdx < 8); childIdx++)
      {
        bNoChilds = branch_arg->hasChild(childIdx);
      }
      // return true if current branch can be deleted
      return (bNoChilds);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename BranchT> void OctreeBase<
        DataT, LeafT, BranchT>::serializeTreeRecursive (const BranchNode* branch_arg, OctreeKey& key_arg,
            std::vector<char>* binaryTreeOut_arg,
            typename std::vector<DataT>* dataVector_arg) const
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

          const OctreeNode *childNode = branch_arg->getChildPtr(childIdx);

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
              const LeafNode* childLeaf = static_cast<const LeafNode*> (childNode);

              if (dataVector_arg)
                childLeaf->getData (*dataVector_arg);

              // we reached a leaf node -> execute serialization callback
              serializeTreeCallback (*childLeaf, key_arg);
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
    template<typename DataT, typename LeafT, typename BranchT> void
    OctreeBase<DataT, LeafT, BranchT>::deserializeTreeRecursive (BranchNode* branch_arg,
        unsigned int depthMask_arg, OctreeKey& key_arg,
        typename std::vector<char>::const_iterator& binaryTreeIT_arg,
        typename std::vector<char>::const_iterator& binaryTreeIT_End_arg,
        typename std::vector<DataT>::const_iterator* dataVectorIterator_arg,
        typename std::vector<DataT>::const_iterator* dataVectorEndIterator_arg)
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
              BranchNode * newBranch;

              // create new child branch
              createBranchChild (*branch_arg, childIdx, newBranch);

              branchCount_++;

              // recursively proceed with new child branch
              deserializeTreeRecursive (newBranch, depthMask_arg / 2, key_arg, binaryTreeIT_arg,binaryTreeIT_End_arg,  dataVectorIterator_arg,
                                        dataVectorEndIterator_arg);
            }
            else
            {
              // we reached leaf node level

              LeafNode* childLeaf;

              // create leaf node
              createLeafChild (*branch_arg, childIdx, childLeaf);

              OctreeKey dataKey;
              bool bKeyBasedEncoding = false;

              if (dataVectorIterator_arg
                  && (*dataVectorIterator_arg != *dataVectorEndIterator_arg))
              {

                // add DataT objects to octree leaf as long as their key fit to voxel
                while ( ( (*dataVectorIterator_arg)
                    != (*dataVectorEndIterator_arg))
                    && (this->genOctreeKeyForDataT (**dataVectorIterator_arg,
                        dataKey) && (dataKey == key_arg)))
                {
                  childLeaf->setData (**dataVectorIterator_arg);
                  (*dataVectorIterator_arg)++;
                  bKeyBasedEncoding = true;
                  objectCount_++;
                }

                // add single DataT object to octree if key-based encoding is disabled
                if (!bKeyBasedEncoding)
                {
                  childLeaf->setData (**dataVectorIterator_arg);
                  (*dataVectorIterator_arg)++;
                  objectCount_++;
                }
              }

              leafCount_++;

              // execute deserialization callback
              deserializeTreeCallback (*childLeaf, key_arg);
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

