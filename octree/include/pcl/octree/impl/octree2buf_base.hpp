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

#ifndef OCTREE_2BUF_BASE_HPP
#define OCTREE_2BUF_BASE_HPP

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT>
    Octree2BufBase<DataT, LeafT>::Octree2BufBase () :
      leafCount_ (0), 
      branchCount_ (1),
      objectCount_ (0), 
      rootNode_ (new OctreeBranch ()), 
      depthMask_ (0), 
      keyRange_ (),
      unusedBranchesPool_ (),
      unusedLeafsPool_ (),
      bufferSelector_ (0),
      resetTree_ (false), 
      treeDirtyFlag_ (false),
      octreeDepth_ (0)
    {
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
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::setMaxVoxelIndex (unsigned int maxVoxelIndex_arg)
    {
      unsigned int treeDepth;

      assert (maxVoxelIndex_arg > 0);

      // tree depth == amount of bits of maxVoxels
      treeDepth = std::max ((std::min (static_cast<unsigned int> (OCT_MAXTREEDEPTH), 
                                       static_cast<unsigned int> (std::ceil (Log2 (maxVoxelIndex_arg))))),
                                       static_cast<unsigned int> (0));

      // define depthMask_ by setting a single bit to 1 at bit position == tree depth
      depthMask_ = (1 << (treeDepth - 1));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::setTreeDepth (unsigned int depth_arg)
    {
      assert (depth_arg > 0);

      // set octree depth
      octreeDepth_ = depth_arg;

      // define depthMask_ by setting a single bit to 1 at bit position == tree depth
      depthMask_ = (1 << (depth_arg - 1));

      // define max. keys
      keyRange_.x = keyRange_.y = keyRange_.z = (1 << depth_arg) - 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::add (const unsigned int idxX_arg, const unsigned int idxY_arg,
                                       const unsigned int idxZ_arg, const DataT& data_arg)
    {
      OctreeKey key;

      // generate key
      genOctreeKeyByIntIdx (idxX_arg, idxY_arg, idxZ_arg, key);

      // add data_arg to octree
      add (key, data_arg);

      objectCount_++;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> bool
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
      return (leaf != 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> bool
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
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::removeLeaf (const unsigned int idxX_arg, const unsigned int idxY_arg,
                                              const unsigned int idxZ_arg)
    {
      OctreeKey key;

      // generate key
      this->genOctreeKeyByIntIdx (idxX_arg, idxY_arg, idxZ_arg, key);

      // free voxel at key
      return (this->removeLeaf (key));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deleteTree ( bool freeMemory_arg )
    {
      if (rootNode_)
      {
        // reset octree
        deleteBranch (*rootNode_);
        leafCount_ = 0;
        branchCount_ = 1;
        objectCount_ = 0;
        
        resetTree_ = false;
        treeDirtyFlag_ = false;
        depthMask_ = 0;
        octreeDepth_ = 0;
      }

      // delete node pool
      if (freeMemory_arg)
        poolCleanUp ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
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
      objectCount_ = 0;
      branchCount_ = 1;
      resetTree_ = true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeTree (std::vector<char>& binaryTreeOut_arg, bool doXOREncoding_arg)
    {
      OctreeKey newKey;
      newKey.x = newKey.y = newKey.z = 0;
      
      // clear binary vector
      binaryTreeOut_arg.clear ();
      binaryTreeOut_arg.reserve (this->branchCount_);

      serializeTreeRecursive (binaryTreeOut_arg, rootNode_, newKey, doXOREncoding_arg);

      // serializeTreeRecursive cleans-up unused octree nodes in previous octree
      treeDirtyFlag_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeTree (std::vector<char>& binaryTreeOut_arg,
                                                 std::vector<DataT>& dataVector_arg, bool doXOREncoding_arg)
    {
      OctreeKey newKey;
      newKey.x = newKey.y = newKey.z = 0;

      // clear output vectors
      binaryTreeOut_arg.clear ();
      dataVector_arg.clear ();

      dataVector_arg.reserve (objectCount_);
      binaryTreeOut_arg.reserve (this->branchCount_);

      Octree2BufBase<DataT, LeafT>::serializeTreeRecursive (binaryTreeOut_arg, rootNode_, newKey,
                                                            dataVector_arg, doXOREncoding_arg);

      // serializeTreeRecursive cleans-up unused octree nodes in previous octree
      treeDirtyFlag_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeLeafs (std::vector<DataT>& dataVector_arg)
    {
      OctreeKey newKey;
      newKey.x = newKey.y = newKey.z = 0;

      // clear output vector
      dataVector_arg.clear ();

      dataVector_arg.reserve (objectCount_);

      serializeLeafsRecursive (rootNode_, newKey, dataVector_arg);

      // serializeLeafsRecursive cleans-up unused octree nodes in previous octree
      treeDirtyFlag_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeTree (std::vector<char>& binaryTreeIn_arg, bool doXORDecoding_arg)
    {
      OctreeKey newKey;
      newKey.x = newKey.y = newKey.z = 0;

      // we will rebuild an octree -> reset leafCount
      leafCount_ = 0;

      // iterator for binary tree structure vector
      std::vector<char>::const_iterator binaryTreeVectorIterator = binaryTreeIn_arg.begin ();

      deserializeTreeRecursive (binaryTreeVectorIterator, rootNode_, depthMask_, newKey, resetTree_, doXORDecoding_arg);

      // we modified the octree structure -> clean-up/tree-reset might be required
      resetTree_ = false;
      treeDirtyFlag_ = true;

      objectCount_ = this->leafCount_;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeTree (std::vector<char>& binaryTreeIn_arg,
                                                   std::vector<DataT>& dataVector_arg, bool doXORDecoding_arg)
    {
      OctreeKey newKey;
      newKey.x = newKey.y = newKey.z = 0;

      // set data iterator to first element
      typename std::vector<DataT>::const_iterator dataVectorIterator = dataVector_arg.begin ();

      // set data iterator to last element
      typename std::vector<DataT>::const_iterator dataVectorEndIterator = dataVector_arg.end ();

      // we will rebuild an octree -> reset leafCount
      leafCount_ = 0;

      // iterator for binary tree structure vector
      std::vector<char>::const_iterator binaryTreeVectorIterator = binaryTreeIn_arg.begin ();

      deserializeTreeRecursive (binaryTreeVectorIterator, rootNode_, depthMask_, newKey, dataVectorIterator,
                                dataVectorEndIterator, resetTree_, doXORDecoding_arg);

      // we modified the octree structure -> clean-up/tree-reset might be required
      resetTree_ = false;
      treeDirtyFlag_ = true;

      objectCount_ = static_cast<unsigned int> (dataVector_arg.size ());
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeTreeAndOutputLeafData (std::vector<char>& binaryTreeIn_arg,
                                                                    std::vector<DataT>& dataVector_arg,
                                                                    bool doXORDecoding_arg)
    {
      OctreeKey newKey;
      newKey.x = newKey.y = newKey.z = 0;

      // free existing tree before tree rebuild
      deleteTree ();

      // iterator for binary tree structure vector
      std::vector<char>::const_iterator binaryTreeVectorIterator = binaryTreeIn_arg.begin ();

      deserializeTreeAndOutputLeafDataRecursive (binaryTreeVectorIterator, rootNode_, depthMask_, newKey,
                                                 dataVector_arg, resetTree_, doXORDecoding_arg);

      // we modified the octree structure -> clean-up/tree-reset might be required
      resetTree_ = false;
      treeDirtyFlag_ = true;

      objectCount_ = static_cast<unsigned int> (dataVector_arg.size ());
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeNewLeafs (std::vector<DataT>& dataVector_arg,
                                                     const int minPointsPerLeaf_arg)
    {
      OctreeKey newKey;
      newKey.x = newKey.y = newKey.z = 0;

      // clear output vector
      dataVector_arg.clear ();

      dataVector_arg.reserve (leafCount_);

      serializeNewLeafsRecursive (rootNode_, newKey, dataVector_arg, minPointsPerLeaf_arg);

      // serializeLeafsRecursive cleans-up unused octree nodes in previous octree
      treeDirtyFlag_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> LeafT*
    Octree2BufBase<DataT, LeafT>::getLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg,
                                                    OctreeBranch* branch_arg, bool branchReset_arg)
    {
      // index to branch child
      unsigned char childIdx;
      LeafT* result = 0;

      // branch reset -> this branch has been taken from previous buffer
      if (branchReset_arg)
      {
        // we can safely remove children references
        for (childIdx = 0; childIdx < 8; childIdx++)
          setBranchChild (*branch_arg, childIdx, 0);
      }

      // find branch child from key
      childIdx = static_cast<unsigned char> (
                ((!!(key_arg.x & depthMask_arg)) << 2) | 
                 ((!!(key_arg.y & depthMask_arg)) << 1) | 
                 (!!(key_arg.z & depthMask_arg)));

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
            childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
            setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childBranch));

            doNodeReset = true; // reset the branch pointer array of stolen child node

          }
          else
          {
            // if required branch does not exist -> create it
            createBranchChild (*branch_arg, childIdx, childBranch);
          }

          branchCount_++;
        }
        // required branch node already exists - use it
        else
          childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, childIdx));
        
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
            childLeaf = static_cast<OctreeLeaf*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
            setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childLeaf));
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
          childLeaf = static_cast<OctreeLeaf*> (getBranchChild (*branch_arg, childIdx));
          
          // return leaf node
          result = childLeaf;
        }
      }
      
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> LeafT*
    Octree2BufBase<DataT, LeafT>::findLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg,
                                                     OctreeBranch* branch_arg) const
    {
      // return leaf node
      unsigned char childIdx;
      LeafT* result = 0;

      // find branch child from key
      childIdx = static_cast<unsigned char> (
                ((!!(key_arg.x & depthMask_arg)) << 2) | 
                 ((!!(key_arg.y & depthMask_arg)) << 1) | 
                  (!!(key_arg.z & depthMask_arg)));

      if (depthMask_arg > 1)
      {
        // we have not reached maximum tree depth
        OctreeBranch* childBranch;
        childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, childIdx));
        
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
          OctreeLeaf* childLeaf = static_cast<OctreeLeaf*> (getBranchChild (*branch_arg, childIdx));
          result = childLeaf;
        }
      }    
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> bool
    Octree2BufBase<DataT, LeafT>::deleteLeafRecursive (const OctreeKey& key_arg, const unsigned int depthMask_arg,
                                                       OctreeBranch* branch_arg)
    {
      // index to branch child
      unsigned char childIdx;
      // indicates if branch is empty and can be safely removed
      bool bNoChilds;

      // find branch child from key
      childIdx = static_cast<unsigned char> (
                 ((!!(key_arg.x & depthMask_arg)) << 2) | 
                 ((!!(key_arg.y & depthMask_arg)) << 1) | 
                  (!!(key_arg.z & depthMask_arg)));

      if (depthMask_arg > 1)
      {
        // we have not reached maximum tree depth
        OctreeBranch* childBranch;
        bool bBranchOccupied;
        
        // next branch child on our path through the tree
        childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, childIdx));
        
        if (childBranch)
        {
          // recursively explore the indexed child branch
          bBranchOccupied = deleteLeafRecursive (key_arg, depthMask_arg / 2, childBranch);
          
          if (!bBranchOccupied)
          {
            // child branch does not own any sub-child nodes anymore -> delete child branch
            deleteBranchChild (*branch_arg, childIdx);
            branchCount_--;
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
      return (bNoChilds);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeTreeRecursive (std::vector<char>& binaryTreeOut_arg,
                                                          OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                                          bool doXOREncoding_arg)
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

      if (doXOREncoding_arg)
      {
        // write XOR bit pattern to output vector
        binaryTreeOut_arg.push_back (nodeXORBitPattern);
      }
      else
      {
        // write bit pattern of current buffer to output vector
        binaryTreeOut_arg.push_back (nodeBitPatternCurrentBuffer);
      }

      // iterate over all children
      for (childIdx = 0; childIdx < 8; childIdx++)
      {
        if (branchHasChild (*branch_arg, childIdx))
        {
          OctreeNode *childNode = getBranchChild (*branch_arg, childIdx);
          
          // generate new key for current branch voxel
          OctreeKey newKey;
          newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
          newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
          newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));
          
          switch (childNode->getNodeType ())
          {
            case BRANCH_NODE:
            {
              // recursively proceed with indexed child branch
              serializeTreeRecursive (binaryTreeOut_arg, static_cast<OctreeBranch*> (childNode), newKey, doXOREncoding_arg);
              break;
            }
            case LEAF_NODE:
            {
              OctreeLeaf* childLeaf = static_cast<OctreeLeaf*> (childNode);
              
              // we reached a leaf node -> execute serialization callback
              serializeLeafCallback (*childLeaf, newKey);
              break;
            }
            default:
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
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeTreeRecursive (std::vector<char>& binaryTreeOut_arg,
                                                          OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                                          typename std::vector<DataT>& dataVector_arg,
                                                          bool doXOREncoding_arg)
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

      if (doXOREncoding_arg)
      {
        // write XOR bit pattern to output vector
        binaryTreeOut_arg.push_back (nodeXORBitPattern);
      }
      else
      {
        // write bit pattern of current buffer to output vector
        binaryTreeOut_arg.push_back (nodeBitPatternCurrentBuffer);
      }

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
          
          OctreeNode *childNode = getBranchChild (*branch_arg, childIdx);
          
          switch (childNode->getNodeType ())
          {
            case BRANCH_NODE:
            {
              // recursively proceed with indexed child branch
              serializeTreeRecursive (binaryTreeOut_arg, static_cast<OctreeBranch*> (childNode), newKey, dataVector_arg,
                                      doXOREncoding_arg);
              break;
            }
            case LEAF_NODE:
            {
              OctreeLeaf* childLeaf = static_cast<OctreeLeaf*> (childNode);
              
              // we reached a leaf node -> execute serialization callback
              serializeLeafCallback (*childLeaf, newKey, dataVector_arg);
              break;
            }
            default:
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
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeLeafsRecursive (OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                                           typename std::vector<DataT>& dataVector_arg)
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
          OctreeNode *childNode = getBranchChild (*branch_arg, childIdx);
          
          // generate new key for current branch voxel
          OctreeKey newKey;
          newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
          newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
          newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));
          
          switch (childNode->getNodeType ())
          {
            case BRANCH_NODE:
            {
              // recursively proceed with indexed child branch
              serializeLeafsRecursive (static_cast<OctreeBranch*> (childNode), newKey, dataVector_arg);
              break;
            }
            case LEAF_NODE:
            {
              OctreeLeaf* childLeaf = static_cast<OctreeLeaf*> (childNode);
              
              // we reached a leaf node -> execute serialization callback
              serializeLeafCallback (*childLeaf, newKey, dataVector_arg);
              break;
            }
            default:
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
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeNewLeafsRecursive (OctreeBranch* branch_arg, const OctreeKey& key_arg,
                                                              std::vector<DataT>& dataVector_arg,
                                                              const int minPointsPerLeaf_arg)
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
          OctreeNode *childNode = getBranchChild (*branch_arg, childIdx);
          
          // generate new key for current branch voxel
          OctreeKey newKey;
          newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
          newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
          newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));
          
          switch (childNode->getNodeType ())
          {
            case BRANCH_NODE:
            {
              // recursively proceed with indexed child branch
              serializeNewLeafsRecursive (static_cast<OctreeBranch*> (childNode), newKey, dataVector_arg, minPointsPerLeaf_arg);
              break;
            }
            case LEAF_NODE:
            {
              // check if leaf existed already in previous buffer
              if (!(nodeBitPatternLastBuffer & (1 << childIdx)))
              {
                // we reached a leaf node
                OctreeLeaf* childLeaf = static_cast<OctreeLeaf*> (childNode);
                
                serializeNewLeafCallback (*childLeaf, newKey, minPointsPerLeaf_arg, dataVector_arg);
              }
              break;
            }
            default:
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
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeTreeRecursive (typename std::vector<char>::const_iterator& binaryTreeIn_arg,
                                                            OctreeBranch* branch_arg,
                                                            const unsigned int depthMask_arg,
                                                            const OctreeKey& key_arg, bool branchReset_arg,
                                                            bool doXORDecoding_arg)
    {
      // child iterator
      unsigned char childIdx;

      // node bits
      char nodeBits;
      char recoveredNodeBits;

      // branch reset -> this branch has been taken from previous buffer
      if (branchReset_arg)
      {
        // we can safely remove children references
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          setBranchChild (*branch_arg, childIdx, 0);
        }  
      }

      // read branch occupancy bit pattern from vector
      nodeBits = (*binaryTreeIn_arg);
      binaryTreeIn_arg++;

      // recover branch occupancy bit pattern
      if (doXORDecoding_arg)
      {
        recoveredNodeBits = getBranchBitPattern (*branch_arg, !bufferSelector_) ^ nodeBits;
      }
      else
      {
        recoveredNodeBits = nodeBits;
      }

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
          
          bool doNodeReset;
          
          doNodeReset = false;
          
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
                childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
                setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childBranch));
                doNodeReset = true;
              }
              else
              {
                // if required branch does not exist -> create it
                createBranchChild (*branch_arg, childIdx, childBranch);
              }
              
              branchCount_++;
              
            }
            else
            {
              // required branch node already exists - use it
              childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, childIdx));
            }
            // recursively proceed with indexed child branch
            deserializeTreeRecursive (binaryTreeIn_arg, childBranch, depthMask_arg / 2, newKey, doNodeReset, doXORDecoding_arg);
          }
          else
          {
            // branch childs are leaf nodes
            OctreeLeaf* childLeaf;
            
            // check if we can take copy a reference from previous buffer
            if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
            {
              // take child leaf node from previous buffer
              childLeaf = static_cast<OctreeLeaf*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
              setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childLeaf));
              
              // reset child leaf
              childLeaf->reset ();
            }
            else
            {
              // if required leaf does not exist -> create it
              createLeafChild (*branch_arg, childIdx, childLeaf);
            }
            
            // execute deserialization callback
            deserializeLeafCallback (*childLeaf, newKey);
            
            leafCount_++;
          }
        }
        else
        {
          // remove old branch pointer information in current branch
          setBranchChild (*branch_arg, bufferSelector_, childIdx, 0);
          
          // remove unused branches in previous buffer
          deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
        }
      } 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeTreeRecursive (typename std::vector<char>::const_iterator& binaryTreeIn_arg,
                                                            OctreeBranch* branch_arg,
                                                            const unsigned int depthMask_arg,
                                                            const OctreeKey& key_arg,
                                                            typename std::vector<DataT>::const_iterator& dataVectorIterator_arg,
                                                            typename std::vector<DataT>::const_iterator& dataVectorEndIterator_arg,
                                                            bool branchReset_arg, bool doXORDecoding_arg)
    {
      // child iterator
      unsigned char childIdx;

      // node bits
      char nodeBits;
      char recoveredNodeBits;

      // branch reset -> this branch has been taken from previous buffer
      if (branchReset_arg)
      {
        // we can safely remove children references
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          setBranchChild (*branch_arg, childIdx, 0);
        }  
      }
      
      // read branch occupancy bit pattern from vector
      nodeBits = (*binaryTreeIn_arg);
      binaryTreeIn_arg++;
      
      // recover branch occupancy bit pattern
      if (doXORDecoding_arg)
      {
        recoveredNodeBits = getBranchBitPattern (*branch_arg, !bufferSelector_) ^ nodeBits;
      }
      else
      {
        recoveredNodeBits = nodeBits;
      }
      
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
          
          bool doNodeReset;
          
          doNodeReset = false;
          
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
                childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
                setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childBranch));
                doNodeReset = true;
              }
              else
              {
                // if required branch does not exist -> create it
                createBranchChild (*branch_arg, childIdx, childBranch);
              }
              
              branchCount_++;
              
            }
            else
            {
              // required branch node already exists - use it
              childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, childIdx));
            }
            
            // recursively proceed with indexed child branch
            deserializeTreeRecursive (binaryTreeIn_arg, childBranch, depthMask_arg / 2, newKey,
                                      dataVectorIterator_arg, dataVectorEndIterator_arg, doNodeReset, doXORDecoding_arg);
          }
          else
          {
            // branch childs are leaf nodes
            OctreeLeaf* childLeaf;
            
            // check if we can take copy a reference pointer from previous buffer
            if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
            {
              // take child leaf node from previous buffer
              childLeaf = static_cast<OctreeLeaf*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
              setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childLeaf));
              
              // reset child leaf
              childLeaf->reset ();
            }
            else
            {
              // if required leaf does not exist -> create it
              createLeafChild (*branch_arg, childIdx, childLeaf);
            }
            
            leafCount_++;
            
            // execute deserialization callback
            deserializeLeafCallback (*childLeaf, newKey, dataVectorIterator_arg, dataVectorEndIterator_arg);
          }
        }
        else
        {      
          // remove old branch pointer information in current branch
          setBranchChild (*branch_arg, bufferSelector_, childIdx, 0);
          
          // remove unused branches in previous buffer
          deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
        }
      }
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeTreeAndOutputLeafDataRecursive (typename std::vector<char>::const_iterator& binaryTreeIn_arg,
                                                                             OctreeBranch* branch_arg,
                                                                             const unsigned int depthMask_arg,
                                                                             const OctreeKey& key_arg,
                                                                             typename std::vector<DataT>& dataVector_arg,
                                                                             bool branchReset_arg, bool doXORDecoding_arg)
    {
      // child iterator
      unsigned char childIdx;

      // node bits
      char nodeBits;
      char recoveredNodeBits;

      // branch reset -> this branch has been taken from previous buffer
      if (branchReset_arg)
      {
        // we can safely remove children references
        for (childIdx = 0; childIdx < 8; childIdx++)
        {
          setBranchChild (*branch_arg, childIdx, 0);
        }
      }

      // read branch occupancy bit pattern from vector
      nodeBits = (*binaryTreeIn_arg);
      binaryTreeIn_arg++;

      // recover branch occupancy bit pattern
      if (doXORDecoding_arg)
      {
        recoveredNodeBits = getBranchBitPattern (*branch_arg, !bufferSelector_) ^ nodeBits;
      }
      else
      {
        recoveredNodeBits = nodeBits;
      }
      
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
          
          bool doNodeReset;
          
          doNodeReset = false;
          
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
                childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
                setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childBranch));
                doNodeReset = true;       
              }
              else
              {
                // if required branch does not exist -> create it
                createBranchChild (*branch_arg, childIdx, childBranch);
              }
              
              branchCount_++;
            }
            else
            {
              // required branch node already exists - use it
              childBranch = static_cast<OctreeBranch*> (getBranchChild (*branch_arg, childIdx));
            }
            // recursively proceed with indexed child branch
            deserializeTreeRecursive (binaryTreeIn_arg, childBranch, depthMask_arg / 2, newKey, doNodeReset, doXORDecoding_arg);
          }
          else
          {
            // branch childs are leaf nodes
            OctreeLeaf* childLeaf;
            
            // check if we can take copy a reference from previous buffer
            if (branchHasChild (*branch_arg, !bufferSelector_, childIdx))
            {
              // take child leaf node from previous buffer
              childLeaf = static_cast<OctreeLeaf*> (getBranchChild (*branch_arg, !bufferSelector_, childIdx));
              setBranchChild (*branch_arg, bufferSelector_, childIdx, static_cast<OctreeNode*> (childLeaf));
              
              // reset child leaf
              childLeaf->reset ();
            }
            else
            {
              // if required leaf does not exist -> create it
              createLeafChild (*branch_arg, childIdx, childLeaf);
            }
            
            // execute deserialization callback
            deserializeTreeAndSerializeLeafCallback (*childLeaf, newKey, dataVector_arg);
            
            leafCount_++;
          }
        }
        else
        {
          // remove old branch pointer information in current branch
          setBranchChild (*branch_arg, bufferSelector_, childIdx, 0);
          
          // remove unused branches in previous buffer
          deleteBranchChild (*branch_arg, !bufferSelector_, childIdx);
        }
      } 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeLeafCallback (OctreeLeaf &, const OctreeKey &)
    {
      // nothing to do
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeLeafCallback (OctreeLeaf &leaf_arg, const OctreeKey &,
                                                         std::vector<DataT> &dataVector_arg)
    {
      leaf_arg.getData (dataVector_arg);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::serializeNewLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey&,
                                                            const int minPointsPerLeaf_arg,
                                                            std::vector<DataT>& dataVector_arg)
    {
      // we reached a leaf node
      std::vector<int> newPointIdx;

      if (minPointsPerLeaf_arg != 0)
      {
        // push to newPointIdx
        leaf_arg.getData (newPointIdx);
        
        // check for minimum amount of leaf point indices
        if (newPointIdx.size () >= static_cast<size_t> (minPointsPerLeaf_arg))
        {
          dataVector_arg.insert (dataVector_arg.end (), newPointIdx.begin (), newPointIdx.end ());
        }
      }
      else
      {
        // push leaf data to dataVector_arg
        leaf_arg.getData (dataVector_arg);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeLeafCallback (OctreeLeaf& leaf_arg,
                                                           const OctreeKey& key_arg,
                                                           typename std::vector<DataT>::const_iterator& dataVectorIterator_arg,
                                                           typename std::vector<DataT>::const_iterator& dataVectorEndIterator_arg)
    {
      OctreeKey dataKey;
      bool bKeyBasedEncoding = false;

      // add DataT objects to octree leaf as long as their key fit to voxel
      while ((dataVectorIterator_arg != dataVectorEndIterator_arg)
             && (this->genOctreeKeyForDataT (*dataVectorIterator_arg, dataKey) && (dataKey == key_arg)))
      {
        leaf_arg.setData (*dataVectorIterator_arg);
        dataVectorIterator_arg++;
        bKeyBasedEncoding = true;
      }
      
      // add single DataT object to octree if key-based encoding is disabled
      if (!bKeyBasedEncoding)
      {
        leaf_arg.setData (*dataVectorIterator_arg);
        dataVectorIterator_arg++;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg)
    {
      DataT newDataT;

      // initialize new leaf child
      if (genDataTByOctreeKey (key_arg, newDataT))
      {
        leaf_arg.setData (newDataT);
      } 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
    Octree2BufBase<DataT, LeafT>::deserializeTreeAndSerializeLeafCallback (OctreeLeaf& leaf_arg,
                                                                           const OctreeKey& key_arg,
                                                                           std::vector<DataT>& dataVector_arg)
    {
      DataT newDataT;

      // initialize new leaf child
      if (genDataTByOctreeKey (key_arg, newDataT))
      {
        leaf_arg.setData (newDataT);
        dataVector_arg.push_back (newDataT);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT> void
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
          OctreeNode *childNode = getBranchChild (*branch_arg, childIdx);
          
          switch (childNode->getNodeType ())
          {
            case BRANCH_NODE:
            {
              // recursively proceed with indexed child branch
              treeCleanUpRecursive (static_cast<OctreeBranch*> (childNode));
              break;
            }
            case LEAF_NODE:
              // leaf level - nothing to do..
              break;
            default:
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

#define PCL_INSTANTIATE_Octree2BufBase(T) template class PCL_EXPORTS pcl::octree::Octree2BufBase<T>;

#endif

