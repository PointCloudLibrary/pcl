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

#ifndef OCTREE_2BUF_BASE_HPP
#define OCTREE_2BUF_BASE_HPP

#include <list>
#include <sstream>
#include <iostream>

namespace pcl
{
  namespace octree
  {

    using namespace std;

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      Octree2BufBase<DataT, LeafT>::Octree2BufBase ()
      {

        // Initialization of globals
        rootNode_ = new OctreeBranch ();
        leafCount_ = 0;
        depthMask_ = 0;
        bufferSelector_ = 0;
        resetTree_ = false;
        treeDirtyFlag_ = false;
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      Octree2BufBase<DataT, LeafT>::~Octree2BufBase ()
      {

        // deallocate tree structure
        deleteTree ();
        delete (rootNode_);
        poolCleanUp ();
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::setTreeDepth (unsigned int depth_arg)
      {

        assert (depth_arg > 0);

        // define depthMask_ by setting a single bit to 1 at bit position == tree depth
        depthMask_ = (1 << (depth_arg - 1));
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::add (const unsigned int idxX_arg, const unsigned int idxY_arg,
                                         const unsigned int idxZ_arg, const DataT& data_arg)
      {
        OctreeKey key;

        // generate key
        genOctreeKeyByIntIdx (idxX_arg, idxY_arg, idxZ_arg, key);

        // add data_arg to octree
        add (key, data_arg);

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      bool
      Octree2BufBase<DataT, LeafT>::get (const unsigned int idxX_arg, const unsigned int idxY_arg,
                                         const unsigned int idxZ_arg, DataT& data_arg) const
      {
        OctreeKey key;

        // generate key
        genOctreeKeyByIntIdx (idxX_arg, idxY_arg, idxZ_arg, key);

        // search for leaf at key
        LeafT* leaf = findLeaf (key);
        if (leaf)
        {
          const DataT * dataPtr;
          // if successful, decode data to data_arg
          leaf->getData (dataPtr);
          if (dataPtr)
            data_arg = *dataPtr;
        }
        // returns true on success
        return (leaf != NULL);
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      bool
      Octree2BufBase<DataT, LeafT>::existLeaf (const unsigned int idxX_arg, const unsigned int idxY_arg,
                                               const unsigned int idxZ_arg) const
      {
        OctreeKey key;

        // generate key
        this->genOctreeKeyByIntIdx (idxX_arg, idxY_arg, idxZ_arg, key);

        // check if key exist in octree
        return existLeaf (key);
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::removeLeaf (const unsigned int idxX_arg, const unsigned int idxY_arg,
                                                const unsigned int idxZ_arg)
      {
        OctreeKey key;

        // generate key
        this->genOctreeKeyByIntIdx (idxX_arg, idxY_arg, idxZ_arg, key);

        // free voxel at key
        return this->removeLeaf (key);
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::deleteTree ()
      {

        if (rootNode_)
        {
          // reset octree
          deleteBranch (*rootNode_);
          leafCount_ = 0;
        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::switchBuffers ()
      {
        if (treeDirtyFlag_)
        {
          // make sure that all unused branch nodes from previous buffer are deleted
          treeCleanUpRecursive (rootNode_);
        }

        // switch butter selector
        bufferSelector_ = !bufferSelector_;

        // reset flags
        treeDirtyFlag_ = true;
        leafCount_ = 0;
        resetTree_ = true;
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeTree (std::ostream& binaryTreeOut_arg)
      {
        // clear binary stream
        binaryTreeOut_arg.clear ();

        serializeTreeRecursive (binaryTreeOut_arg, rootNode_);

        // serializeTreeRecursive cleans-up unused octree nodes in previous octree
        treeDirtyFlag_ = false;

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeTree (std::ostream& binaryTreeOut_arg, std::vector<DataT>& dataVector_arg)
      {
        OctreeKey newKey;
        newKey.x = newKey.y = newKey.z = 0;

        // clear output streams
        binaryTreeOut_arg.clear ();
        dataVector_arg.clear ();

        dataVector_arg.reserve (leafCount_);

        Octree2BufBase<DataT, LeafT>::serializeTreeRecursive (binaryTreeOut_arg, rootNode_, newKey, dataVector_arg);

        // serializeTreeRecursive cleans-up unused octree nodes in previous octree
        treeDirtyFlag_ = false;
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeLeafs (std::vector<DataT>& dataVector_arg)
      {
        OctreeKey newKey;
        newKey.x = newKey.y = newKey.z = 0;

        // clear output stream
        dataVector_arg.clear ();

        dataVector_arg.reserve (leafCount_);

        serializeLeafsRecursive (rootNode_, newKey, dataVector_arg);

        // serializeLeafsRecursive cleans-up unused octree nodes in previous octree
        treeDirtyFlag_ = false;
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::deserializeTree (std::istream& binaryTreeIn_arg)
      {
        OctreeKey newKey;
        newKey.x = newKey.y = newKey.z = 0;

        // we will rebuild an octree -> reset leafCount
        leafCount_ = 0;
        deserializeTreeRecursive (binaryTreeIn_arg, rootNode_, depthMask_, newKey);

        // deserializeTreeRecursive cleans-up unused octree nodes in previous octree
        treeDirtyFlag_ = false;

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::deserializeTree (std::istream& binaryTreeIn_arg, std::vector<DataT>& dataVector_arg)
      {
        OctreeKey newKey;
        newKey.x = newKey.y = newKey.z = 0;

        // set data iterator to first element
        typename std::vector<DataT>::iterator dataVectorIterator = dataVector_arg.begin ();

        // set data iterator to last element
        typename std::vector<DataT>::const_iterator dataVectorEndIterator = dataVector_arg.end ();

        // we will rebuild an octree -> reset leafCount
        leafCount_ = 0;

        deserializeTreeRecursive (binaryTreeIn_arg, rootNode_, depthMask_, newKey, dataVectorIterator,
                                  dataVectorEndIterator);

        // deserializeTreeRecursive cleans-up unused octree nodes in previous octree
        treeDirtyFlag_ = false;
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeNewLeafs (std::vector<DataT>& dataVector_arg)
      {
        OctreeKey newKey;
        newKey.x = newKey.y = newKey.z = 0;

        // clear output stream
        dataVector_arg.clear ();

        dataVector_arg.reserve (leafCount_);

        serializeNewLeafsRecursive (rootNode_, newKey, dataVector_arg);

        // serializeLeafsRecursive cleans-up unused octree nodes in previous octree
        treeDirtyFlag_ = false;
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      LeafT*
      Octree2BufBase<DataT, LeafT>::getLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg,
                                                      OctreeBranch* branch_arg, bool branchReset_arg)
      {

        // index to branch child
        unsigned char childIdx;
        LeafT* result = NULL;

        // branch reset -> this branch has been taken from previous buffer
        if (branchReset_arg)
        {
          // we can safely remove children references
          for (childIdx = 0; childIdx < 8; childIdx++)
          {
            setBranchChild (*branch_arg, childIdx, NULL);
          }

        }

        // find branch child from key
        childIdx = ((!!(key_arg.x & depthMask_arg)) << 2) | ((!!(key_arg.y & depthMask_arg)) << 1) | (!!(key_arg.z
            & depthMask_arg));

        if (depthMask_arg > 1)
        {
          // we have not reached maximum tree depth

          OctreeBranch* childBranch;
          bool doNodeReset;

          doNodeReset = false;

          // if required branch does not exist
          if (!branchHasChild (*branch_arg, childIdx))
          {

            // check if we find a branch node reference in previous buffer
            if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
            {

              // take child branch from previous buffer
              childBranch = (OctreeBranch*)getBranchChild (*branch_arg, !bufferSelector_, childIdx);
              setBranchChild (*branch_arg, bufferSelector_, childIdx, (OctreeNode*)childBranch);

              doNodeReset = true; // reset the branch pointer array of stolen child node

            }
            else
            {
              // if required branch does not exist -> create it
              createBranchChild (*branch_arg, childIdx, childBranch);
            }

          }
          else
          {
            // required branch node already exists - use it
            childBranch = (OctreeBranch*)getBranchChild (*branch_arg, childIdx);
          }

          // recursively proceed with indexed child branch
          result = getLeafRecursive (key_arg, depthMask_arg / 2, childBranch, doNodeReset);

        }
        else
        {
          // branch childs are leaf nodes

          OctreeLeaf* childLeaf;
          if (!branchHasChild (*branch_arg, childIdx))
          {
            // leaf node at childIdx does not exist

            // check if we can take copy a reference from previous buffer
            if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
            {

              // take child leaf node from previous buffer
              childLeaf = (OctreeLeaf*)getBranchChild (*branch_arg, !bufferSelector_, childIdx);
              setBranchChild (*branch_arg, bufferSelector_, childIdx, (OctreeNode*)childLeaf);

              childLeaf->reset ();

              leafCount_++;

            }
            else
            {

              // if required leaf does not exist -> create it
              createLeafChild (*branch_arg, childIdx, childLeaf);
              leafCount_++;
            }

            // return leaf node
            result = childLeaf;

          }
          else
          {

            // leaf node already exist
            childLeaf = (OctreeLeaf*)getBranchChild (*branch_arg, childIdx);

            // return leaf node
            result = childLeaf;

          }

        }

        return result;

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      LeafT*
      Octree2BufBase<DataT, LeafT>::findLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg,
                                                       OctreeBranch* branch_arg) const
      {
        // return leaf node
        unsigned char childIdx;
        LeafT* result = NULL;

        // find branch child from key
        childIdx = ((!!(key_arg.x & depthMask_arg)) << 2) | ((!!(key_arg.y & depthMask_arg)) << 1) | (!!(key_arg.z
            & depthMask_arg));

        if (depthMask_arg > 1)
        {
          // we have not reached maximum tree depth
          OctreeBranch* childBranch;
          childBranch = (OctreeBranch*)getBranchChild (*branch_arg, childIdx);

          if (childBranch)
            // recursively proceed with indexed child branch
            result = findLeafRecursive (key_arg, depthMask_arg / 2, childBranch);

        }
        else
        {
          // we reached leaf node level
          if (branchHasChild (*branch_arg, childIdx))
          {

            // return existing leaf node
            OctreeLeaf* childLeaf = (OctreeLeaf*)getBranchChild (*branch_arg, childIdx);
            result = childLeaf;
          }

        }

        return result;

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      bool
      Octree2BufBase<DataT, LeafT>::deleteLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg,
                                                         OctreeBranch* branch_arg)
      {
        // index to branch child
        unsigned char childIdx;
        // indicates if branch is empty and can be safely removed
        bool bNoChilds;

        // find branch child from key
        childIdx = ((!!(key_arg.x & depthMask_arg)) << 2) | ((!!(key_arg.y & depthMask_arg)) << 1) | (!!(key_arg.z
            & depthMask_arg));

        if (depthMask_arg > 1)
        {
          // we have not reached maximum tree depth

          OctreeBranch* childBranch;
          bool bBranchOccupied;

          // next branch child on our path through the tree
          childBranch = (OctreeBranch*)getBranchChild (*branch_arg, childIdx);

          if (childBranch)
          {
            // recursively explore the indexed child branch
            bBranchOccupied = deleteLeafRecursive (key_arg, depthMask_arg / 2, childBranch);

            if (!bBranchOccupied)
            {
              // child branch does not own any sub-child nodes anymore -> delete child branch
              deleteBranchChild (*branch_arg, childIdx);
            }
          }

        }
        else
        {

          // our child is a leaf node -> delete it
          deleteBranchChild (*branch_arg, childIdx);
          leafCount_--;
        }

        // check if current branch still owns childs
        bNoChilds = false;
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          bNoChilds = branchHasChild (*branch_arg, childIdx);
          if (bNoChilds)
            break;
        }

        // return true if current branch can be deleted
        return bNoChilds;

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeTreeRecursive (std::ostream& binaryTreeOut_arg, OctreeBranch* branch_arg)
      {

        // child iterator
        unsigned char childIdx;

        // bit pattern
        char nodeBitPatternCurrentBuffer;
        char nodeBitPatternLastBuffer;
        char nodeXORBitPattern;
        char unusedBranchesBits;

        // occupancy bit patterns of branch node  (current and previous octree buffer)
        nodeBitPatternCurrentBuffer = getBranchBitPattern (*branch_arg, bufferSelector_);
        nodeBitPatternLastBuffer = getBranchBitPattern (*branch_arg, !bufferSelector_);

        // XOR of current and previous occupancy bit patterns
        nodeXORBitPattern = nodeBitPatternCurrentBuffer ^ nodeBitPatternLastBuffer;

        // bit pattern indicating unused octree nodes in previous branch
        unusedBranchesBits = nodeXORBitPattern & nodeBitPatternLastBuffer;

        // write XOR bit pattern to output stream
        binaryTreeOut_arg.write (&nodeXORBitPattern, 1);

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {

          if (branchHasChild (*branch_arg, childIdx))
          {
            const OctreeNode * childNode;
            childNode = getBranchChild (*branch_arg, childIdx);

            switch (childNode->getNodeType ())
            {
              case BRANCH_NODE:

                // recursively proceed with indexed child branch
                serializeTreeRecursive (binaryTreeOut_arg, (OctreeBranch*)childNode);
                break;

              case LEAVE_NODE:
                // nothing to do
                break;
            }

          }

          // check for unused branches in previous buffer
          if (unusedBranchesBits & (1 << childIdx))
          {
            // delete branch, free memory
            deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
          }

        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeTreeRecursive (std::ostream& binaryTreeOut_arg, OctreeBranch* branch_arg,
                                                            OctreeKey& key_arg, std::vector<DataT>& dataVector_arg)
      {

        // child iterator
        unsigned char childIdx;

        // bit pattern
        char nodeBitPatternCurrentBuffer;
        char nodeBitPatternLastBuffer;
        char nodeXORBitPattern;
        char unusedBranchesBits;

        // occupancy bit patterns of branch node  (current and previous octree buffer)
        nodeBitPatternCurrentBuffer = getBranchBitPattern (*branch_arg, bufferSelector_);
        nodeBitPatternLastBuffer = getBranchBitPattern (*branch_arg, !bufferSelector_);

        // XOR of current and previous occupancy bit patterns
        nodeXORBitPattern = nodeBitPatternCurrentBuffer ^ nodeBitPatternLastBuffer;

        // bit pattern indicating unused octree nodes in previous branch
        unusedBranchesBits = nodeXORBitPattern & nodeBitPatternLastBuffer;

        // write XOR bit pattern to output stream
        binaryTreeOut_arg.write (&nodeXORBitPattern, 1);

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          if (branchHasChild (*branch_arg, childIdx))
          {

            // generate new key for current branch voxel
            OctreeKey newKey;
            newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
            newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
            newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));

            const OctreeNode * childNode;
            childNode = getBranchChild (*branch_arg, childIdx);

            switch (childNode->getNodeType ())
            {
              case BRANCH_NODE:
                // recursively proceed with indexed child branch
                serializeTreeRecursive (binaryTreeOut_arg, (OctreeBranch*)childNode, newKey, dataVector_arg);
                break;

              case LEAVE_NODE:
                OctreeLeaf* childLeaf = (OctreeLeaf*)childNode;

                // we reached a leaf node -> decode to dataVector_arg
                childLeaf->getData (dataVector_arg);

                break;
            }

          }

          // check for unused branches in previous buffer
          if (unusedBranchesBits & (1 << childIdx))
          {
            // delete branch, free memory
            deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
          }

        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeLeafsRecursive (OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                                             std::vector<DataT>& dataVector_arg)
      {
        // child iterator
        unsigned char childIdx;

        // bit pattern
        char nodeBitPatternLastBuffer;
        char nodeXORBitPattern;
        char unusedBranchesBits;

        // occupancy bit pattern of branch node  (previous octree buffer)
        nodeBitPatternLastBuffer = getBranchBitPattern (*branch_arg, !bufferSelector_);

        // XOR of current and previous occupancy bit patterns
        nodeXORBitPattern = getBranchXORBitPattern (*branch_arg);

        // bit pattern indicating unused octree nodes in previous branch
        unusedBranchesBits = nodeXORBitPattern & nodeBitPatternLastBuffer;

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          if (branchHasChild (*branch_arg, childIdx))
          {
            const OctreeNode * childNode;
            childNode = getBranchChild (*branch_arg, childIdx);

            // generate new key for current branch voxel
            OctreeKey newKey;
            newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
            newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
            newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));

            switch (childNode->getNodeType ())
            {
              case BRANCH_NODE:

                // recursively proceed with indexed child branch
                serializeLeafsRecursive ((OctreeBranch*)childNode, newKey, dataVector_arg);
                break;
              case LEAVE_NODE:
                OctreeLeaf* childLeaf = (OctreeLeaf*)childNode;

                // we reached a leaf node -> decode to dataVector_arg
                childLeaf->getData (dataVector_arg);
                break;
            }

          }

          // check for unused branches in previous buffer
          if (unusedBranchesBits & (1 << childIdx))
          {
            // delete branch, free memory
            deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
          }

        }
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::serializeNewLeafsRecursive (OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                                                std::vector<DataT>& dataVector_arg)
      {
        // child iterator
        unsigned char childIdx;

        // bit pattern
        char nodeBitPatternLastBuffer;
        char nodeXORBitPattern;
        char unusedBranchesBits;

        // occupancy bit pattern of branch node  (previous octree buffer)
        nodeBitPatternLastBuffer = getBranchBitPattern (*branch_arg, !bufferSelector_);

        // XOR of current and previous occupancy bit patterns
        nodeXORBitPattern = getBranchXORBitPattern (*branch_arg);

        // bit pattern indicating unused octree nodes in previous branch
        unusedBranchesBits = nodeXORBitPattern & nodeBitPatternLastBuffer;

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          if (branchHasChild (*branch_arg, childIdx))
          {
            const OctreeNode * childNode;
            childNode = getBranchChild (*branch_arg, childIdx);

            // generate new key for current branch voxel
            OctreeKey newKey;
            newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
            newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
            newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));

            switch (childNode->getNodeType ())
            {
              case BRANCH_NODE:

                // recursively proceed with indexed child branch
                serializeNewLeafsRecursive ((OctreeBranch*)childNode, newKey, dataVector_arg);
                break;
              case LEAVE_NODE:
                // check if leaf existed already in previous buffer
                if (!(nodeBitPatternLastBuffer & (1 << childIdx)))
                {
                  OctreeLeaf* childLeaf = (OctreeLeaf*)childNode;

                  // we reached a leaf node -> decode to dataVector_arg
                  childLeaf->getData (dataVector_arg);
                }
                break;
            }

          }

          // check for unused branches in previous buffer
          if (unusedBranchesBits & (1 << childIdx))
          {
            // delete branch, free memory
            deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
          }

        }
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::deserializeTreeRecursive (std::istream& binaryTreeIn_arg, OctreeBranch* branch_arg,
                                                              const unsigned int depthMask_arg, const OctreeKey& key_arg)
      {
        // child iterator
        unsigned char childIdx;

        // node bits
        char nodeBits;
        char recoveredNodeBits;

        // read branch occupancy bit pattern from stream
        binaryTreeIn_arg.read (&nodeBits, 1);

        // recover branch occupancy bit pattern
        recoveredNodeBits = getBranchBitPattern (*branch_arg, !bufferSelector_) ^ nodeBits;

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          // if occupancy bit for childIdx is set..
          if (recoveredNodeBits & (1 << childIdx))
          {

            // generate new key for current branch voxel
            OctreeKey newKey;
            newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
            newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
            newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));

            if (depthMask_arg > 1)
            {
              // we have not reached maximum tree depth
              OctreeBranch* childBranch;

              if (!branchHasChild (*branch_arg, childIdx))
              {

                // check if we find a branch node reference in previous buffer
                if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
                {
                  // take child branch from previous buffer
                  childBranch = (OctreeBranch*)getBranchChild (*branch_arg, !bufferSelector_, childIdx);
                  setBranchChild (*branch_arg, bufferSelector_, childIdx, (OctreeNode*)childBranch);

                }
                else
                {
                  // if required branch does not exist -> create it
                  createBranchChild (*branch_arg, childIdx, childBranch);
                }

              }
              else
              {
                // required branch node already exists - use it
                childBranch = (OctreeBranch*)getBranchChild (*branch_arg, childIdx);
              }

              // recursively proceed with indexed child branch
              deserializeTreeRecursive (binaryTreeIn_arg, childBranch, depthMask_arg / 2, newKey);

            }
            else
            {
              // branch childs are leaf nodes

              OctreeLeaf* childLeaf;
              DataT newDataT;

              // check if we can take copy a reference from previous buffer
              if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
              {
                // take child leaf node from previous buffer
                childLeaf = (OctreeLeaf*)getBranchChild (*branch_arg, !bufferSelector_, childIdx);
                setBranchChild (*branch_arg, bufferSelector_, childIdx, (OctreeNode*)childLeaf);

                // reset child leaf
                childLeaf->reset ();

              }
              else
              {
                // if required leaf does not exist -> create it
                createLeafChild (*branch_arg, childIdx, childLeaf);

              }

              // initialize new leaf child
              if (getDataTByKey (newKey, newDataT) )
                childLeaf->setData (newDataT);

              leafCount_++;

            }
          }
          else
          {

            // remove old branch pointer information in current branch
            setBranchChild (*branch_arg, bufferSelector_, childIdx, NULL);

            // remove unused branches in previous buffer
            deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
          }
        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::deserializeTreeRecursive (
                                                              std::istream& binaryTreeIn_arg,
                                                              OctreeBranch* branch_arg,
                                                              const unsigned int depthMask_arg,
                                                              const OctreeKey& key_arg,
                                                              typename std::vector<DataT>::iterator& dataVectorIterator_arg,
                                                              typename std::vector<DataT>::const_iterator& dataVectorEndIterator_arg)
      {
        // child iterator
        unsigned char childIdx;

        // node bits
        char nodeBits;
        char recoveredNodeBits;

        // read branch occupancy bit pattern from stream
        binaryTreeIn_arg.read (&nodeBits, 1);

        // recover branch occupancy bit pattern
        recoveredNodeBits = getBranchBitPattern (*branch_arg, !bufferSelector_) ^ nodeBits;

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          // if occupancy bit for childIdx is set..
          if (recoveredNodeBits & (1 << childIdx))
          {
            // generate new key for current branch voxel
            OctreeKey newKey;
            newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
            newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
            newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));

            if (depthMask_arg > 1)
            {
              // we have not reached maximum tree depth

              OctreeBranch* childBranch;

              // check if we find a branch node reference in previous buffer
              if (!branchHasChild (*branch_arg, childIdx))
              {

                if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
                {
                  // take child branch from previous buffer
                  childBranch = (OctreeBranch*)getBranchChild (*branch_arg, !bufferSelector_, childIdx);
                  setBranchChild (*branch_arg, bufferSelector_, childIdx, (OctreeNode*)childBranch);

                }
                else
                {
                  // if required branch does not exist -> create it
                  createBranchChild (*branch_arg, childIdx, childBranch);
                }

              }
              else
              {
                // required branch node already exists - use it
                childBranch = (OctreeBranch*)getBranchChild (*branch_arg, childIdx);
              }

              // recursively proceed with indexed child branch
              deserializeTreeRecursive (binaryTreeIn_arg, childBranch, depthMask_arg / 2, newKey,
                                        dataVectorIterator_arg, dataVectorEndIterator_arg);

            }
            else
            {
              // branch childs are leaf nodes

              OctreeLeaf* childLeaf;

              // check if we can take copy a reference pointer from previous buffer
              if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
              {
                // take child leaf node from previous buffer
                childLeaf = (OctreeLeaf*)getBranchChild (*branch_arg, !bufferSelector_, childIdx);
                setBranchChild (*branch_arg, bufferSelector_, childIdx, (OctreeNode*)childLeaf);

                // reset child leaf
                childLeaf->reset ();

              }
              else
              {
                // if required leaf does not exist -> create it
                createLeafChild (*branch_arg, childIdx, childLeaf);
              }

              leafCount_++;

              // feed new leaf node with data
              OctreeKey dataKey;

              bool bKeyBasedEncoding = false;
              while ((dataVectorIterator_arg != dataVectorEndIterator_arg)
                  && (this->genOctreeKey (*dataVectorIterator_arg, dataKey) && (dataKey == newKey)))
              {
                childLeaf->setData (*dataVectorIterator_arg);
                dataVectorIterator_arg++;
                bKeyBasedEncoding = true;
              }

              if (!bKeyBasedEncoding)
              {
                childLeaf->setData (*dataVectorIterator_arg);
                dataVectorIterator_arg++;
              }

            }
          }
          else
          {

            // remove old branch pointer information in current branch
            setBranchChild (*branch_arg, bufferSelector_, childIdx, NULL);

            // remove unused branches in previous buffer
            deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
          }
        }
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
      void
      Octree2BufBase<DataT, LeafT>::treeCleanUpRecursive (OctreeBranch* branch_arg)
      {

        // child iterator
        unsigned char childIdx;

        // bit pattern
        char nodeBitPatternLastBuffer;
        char nodeXORBitPattern;
        char unusedBranchesBits;

        // occupancy bit pattern of branch node  (previous octree buffer)
        nodeBitPatternLastBuffer = getBranchBitPattern (*branch_arg, !bufferSelector_);

        // XOR of current and previous occupancy bit patterns
        nodeXORBitPattern = getBranchXORBitPattern (*branch_arg);

        // bit pattern indicating unused octree nodes in previous branch
        unusedBranchesBits = nodeXORBitPattern & nodeBitPatternLastBuffer;

        // iterate over all children
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          if (branchHasChild (*branch_arg, childIdx))
          {
            const OctreeNode * childNode;
            childNode = getBranchChild (*branch_arg, childIdx);

            switch (childNode->getNodeType ())
            {
              case BRANCH_NODE:
                // recursively proceed with indexed child branch
                treeCleanUpRecursive ((OctreeBranch*)childNode);
                break;
              case LEAVE_NODE:
                // leaf level - nothing to do..
                break;
            }

          }

          // check for unused branches in previous buffer
          if (unusedBranchesBits & (1 << childIdx))
          {
            // delete branch, free memory
            deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
          }

        }
      }

  }
}

#define PCL_INSTANTIATE_Octree2BufBase(T) template class pcl::octree::Octree2BufBase<T>;

#endif

