/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include <gtest/gtest.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h>

#include <pcl/octree/octree_impl.h>

using namespace pcl;
using namespace octree;
using namespace pcl::common;

struct MyVoxel
{
  unsigned int x;
  unsigned int y;
  unsigned int z;
};

TEST (OctreeBase, CreateFindRemoveLeaves)
{
  OctreeBase<int> octree;
  octree.setTreeDepth (8);

  int data[256];
  MyVoxel voxels[256];

  srand (static_cast<unsigned int> (time (NULL)));

  // Generate some voxel indices
  for (size_t i = 0; i < 256; i++)
  {
    data[i] = i;
    voxels[i].x = i;
    voxels[i].y = 255 - i;
    voxels[i].z = i;
    // Add data to leaf node voxel
    int* voxel_container = octree.createLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
    data[i] = *voxel_container;
  }

  ASSERT_EQ (256, octree.getLeafCount ());

  for (size_t i = 0; i < 128; i++)
  {
    // Retrieve and check data from leaf voxel
    int* voxel_container = octree.findLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
    ASSERT_EQ (*voxel_container, data[i]);
  }

  for (size_t i = 128; i < 256; i++)
  {
    // Check if leaf node exists in tree
    ASSERT_TRUE (octree.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
    // Remove leaf node
    octree.removeLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
    // Leaf node shouldn't exist in tree anymore
    ASSERT_FALSE (octree.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }
}

TEST (OctreeBase, DeleteTree)
{
  OctreeBase<int> octree;
  octree.setTreeDepth (8);

  int data[256];
  MyVoxel voxels[256];

  srand (static_cast<unsigned int> (time (NULL)));

  // Generate some voxel indices
  for (size_t i = 0; i < 256; i++)
  {
    data[i] = i;
    voxels[i].x = i;
    voxels[i].y = 255 - i;
    voxels[i].z = i;
    // Add data to leaf node voxel
    int* voxel_container = octree.createLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
    data[i] = *voxel_container;
  }

  ASSERT_EQ (256, octree.getLeafCount ());

  octree.deleteTree ();
  octree.setTreeDepth (8);

  // octree.getLeafCount() should be zero now;
  ASSERT_EQ (0, octree.getLeafCount ());

  for (size_t i = 0; i < 128; i++)
    ASSERT_FALSE (octree.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
}

TEST (OctreeBase, Serialization)
{
  OctreeBase<int> octree_a, octree_b;
  octree_a.setTreeDepth (8);
  octree_b.setTreeDepth (8);

  int data[256];
  MyVoxel voxels[256];

  srand (static_cast<unsigned int> (time (NULL)));

  // Generate some voxel indices
  for (size_t i = 0; i < 256; i++)
  {
    data[i] = i;
    voxels[i].x = i;
    voxels[i].y = 255 - i;
    voxels[i].z = i;
    // Only insert first 128 voxels
    if (i < 128)
    {
      int* voxel_container = octree_a.createLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
      data[i] = *voxel_container;
    }
  }

  std::vector<char> tree_binary_a;
  std::vector<char> tree_binary_b;

  std::vector<int*> leaf_vector_a;
  std::vector<int*> leaf_vector_b;

  // Serialize tree - generate binary octree description
  octree_a.serializeTree (tree_binary_a);

  // Deserialize tree - rebuild octree based on binary octree description
  octree_b.deserializeTree (tree_binary_a);

  // Check if leafs exist in deserialized octree
  for (size_t i = 0; i < 128; i++)
    ASSERT_TRUE (octree_b.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));

  // These leafs should not exist..
  for (size_t i = 128; i < 256; i++)
    ASSERT_FALSE (octree_b.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));

  // Test tree serialization
  octree_a.serializeTree (tree_binary_a, leaf_vector_a);

  // Make sure, we retrieved all data objects
  ASSERT_EQ (octree_a.getLeafCount (), leaf_vector_a.size ());

  // Check if leaf data is found in octree input data
  for (size_t i = 0; i < 128; i++)
  {
    int leaf_int = *leaf_vector_a.back ();
    leaf_vector_a.pop_back ();
    bool found = false;
    for (size_t j = 0; j < 256; j++)
      if (data[j] == leaf_int)
      {
        found = true;
        break;
      }
    ASSERT_TRUE (found);
  }

  // Test tree serialization
  octree_a.serializeLeafs (leaf_vector_a);

  for (size_t i = 0; i < 128; i++)
  {
    int leaf_int = *leaf_vector_a.back ();
    leaf_vector_a.pop_back ();
    bool found = false;
    for (size_t j = 0; j < 256; j++)
      if (data[j] == leaf_int)
      {
        found = true;
        break;
      }
    ASSERT_TRUE (found);
  }

  // Test tree serialization with leaf data vectors
  octree_a.serializeTree (tree_binary_a, leaf_vector_a);
  octree_b.deserializeTree (tree_binary_a, leaf_vector_a);

  ASSERT_EQ (octree_a.getLeafCount (), octree_b.getLeafCount ());
  ASSERT_EQ (128, octree_b.getLeafCount ());

  octree_b.serializeTree (tree_binary_b, leaf_vector_b);

  // Test size and leaf count of reconstructed octree
  ASSERT_EQ (octree_b.getLeafCount (), leaf_vector_b.size ());
  ASSERT_EQ (leaf_vector_a.size (), leaf_vector_b.size ());

  for (size_t i = 0; i < leaf_vector_b.size (); i++)
    ASSERT_TRUE (*leaf_vector_a[i] == *leaf_vector_b[i]);
}

TEST (OctreeBase, LeafAndBranchCount)
{
  OctreeBase<int> octree;
  octree.setTreeDepth (8);

  int data[256];
  MyVoxel voxels[256];

  srand (static_cast<unsigned int> (time (NULL)));

  // Generate some voxel indices
  for (size_t i = 0; i < 256; i++)
  {
    data[i] = i;
    voxels[i].x = i;
    voxels[i].y = 255 - i;
    voxels[i].z = i;
    int* voxel_container = octree.createLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
    data[i] = *voxel_container;
  }

  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  // Iterate over the tree checking depth, counting nodes, leaves, and branches
  OctreeBase<int>::Iterator it;
  OctreeBase<int>::Iterator it_end = octree.end ();
  for (it = octree.begin (); it != it_end; ++it)
  {
    // Depth should always be less than tree depth
    ASSERT_GE (octree.getTreeDepth (), it.getCurrentOctreeDepth ());
    // Update node, branch and leaf count
    const OctreeNode* node = it.getCurrentOctreeNode ();
    if (node->getNodeType () == BRANCH_NODE)
      branch_count++;
    else if (node->getNodeType () == LEAF_NODE)
      leaf_count++;
    node_count++;
  }

  // Compare node, branch and leaf count against actual tree values
  ASSERT_EQ (octree.getBranchCount () + octree.getLeafCount (), node_count);
  ASSERT_EQ (octree.getBranchCount (), branch_count);
  ASSERT_EQ (octree.getLeafCount (), leaf_count);
}

TEST (OctreeBase, BreadthFirstIterator)
{
  OctreeBase<int> octree;
  octree.setTreeDepth (8);

  int data[256];
  MyVoxel voxels[256];

  srand (static_cast<unsigned int> (time (NULL)));

  // Generate some voxel indices
  for (size_t i = 0; i < 256; i++)
  {
    data[i] = i;
    voxels[i].x = i;
    voxels[i].y = 255 - i;
    voxels[i].z = i;
    int* voxel_container = octree.createLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
    data[i] = *voxel_container;
  }

  // Breadth-first iterator test
  unsigned int last_depth = 0;
  unsigned int branch_node_count = 0;
  unsigned int leaf_node_count = 0;

  bool leaf_node_visited = false;

  OctreeBase<int>::BreadthFirstIterator bf_it;
  const OctreeBase<int>::BreadthFirstIterator bf_it_end = octree.breadth_end ();

  for (bf_it = octree.breadth_begin (); bf_it != bf_it_end; ++bf_it)
  {
    // Tree depth of visited nodes must grow
    ASSERT_EQ (bf_it.getCurrentOctreeDepth () >= last_depth, true);
    last_depth = bf_it.getCurrentOctreeDepth ();

    if (bf_it.isBranchNode ())
    {
      branch_node_count++;
      // Leaf nodes are traversed in the end
      ASSERT_FALSE (leaf_node_visited);
    }

    if (bf_it.isLeafNode ())
    {
      leaf_node_count++;
      leaf_node_visited = true;
    }
  }

  // Check if every branch node and every leaf node has been visited
  ASSERT_EQ (leaf_node_count, octree.getLeafCount ());
  ASSERT_EQ (branch_node_count, octree.getBranchCount ());
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

