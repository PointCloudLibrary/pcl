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
#include <pcl/octree/octree2buf_base.h>

using namespace pcl;
using namespace octree;
using namespace pcl::common;

struct MyVoxel
{
  unsigned int x;
  unsigned int y;
  unsigned int z;
};

TEST (Octree2Buf, DoubleBuffering)
{
  const size_t TEST_RUNS = 20;
  const size_t TEST_POINTS = 3000;

  // Create octree instances
  Octree2BufBase<int> octree_a;
  Octree2BufBase<int> octree_b;

  std::vector<char> tree_binary_a;
  std::vector<char> tree_binary_b;

  std::vector<int*> leaf_vector_a;
  std::vector<int*> leaf_vector_b;

  octree_a.setTreeDepth (5);
  octree_b.setTreeDepth (5);

  int data[TEST_POINTS];
  MyVoxel voxels[TEST_POINTS];

  srand (static_cast<unsigned int> (time (NULL)));

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    octree_a.deleteTree ();
    octree_b.deleteTree ();
    octree_a.setTreeDepth (5);
    octree_b.setTreeDepth (5);

    size_t runs = rand () % 20 + 1;
    for (size_t k = 0; k < runs; k++)
    {
      // Switch buffers
      octree_a.switchBuffers ();
      octree_b.switchBuffers ();

      for (size_t i = 0; i < TEST_POINTS; i++)
      {
        data[i] = rand ();
        voxels[i].x = rand () % 4096;
        voxels[i].y = rand () % 4096;
        voxels[i].z = rand () % 4096;
        // Add data to octree
        int* container = octree_a.createLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
        *container = data[i];
      }

      // Test serialization
      octree_a.serializeTree (tree_binary_a, leaf_vector_a, true);
      octree_b.deserializeTree (tree_binary_a, leaf_vector_a, true);
    }

    octree_b.serializeTree (tree_binary_b, leaf_vector_b, true);

    // Check leaf count of rebuilt octree
    ASSERT_EQ (octree_a.getLeafCount (), octree_b.getLeafCount ());
    ASSERT_EQ (leaf_vector_b.size (), octree_b.getLeafCount ());
    ASSERT_EQ (leaf_vector_a.size (), leaf_vector_b.size ());

    // Check if octree octree structure is consistent.
    for (size_t i = 0; i < leaf_vector_b.size (); i++)
      ASSERT_TRUE (*leaf_vector_a[i] == *leaf_vector_b[i]);
  }
}

TEST (Octree2Buf, DoubleBufferingXOR)
{
  const size_t TEST_RUNS = 15;
  const size_t TEST_POINTS = 3000;

  // Create octree instances
  Octree2BufBase<int> octree_a;
  Octree2BufBase<int> octree_b;

  std::vector<char> tree_binary_a;
  std::vector<char> tree_binary_b;

  std::vector<int*> leaf_vector_a;
  std::vector<int*> leaf_vector_b;

  octree_a.setTreeDepth (5);
  octree_b.setTreeDepth (5);

  int data[TEST_POINTS];
  MyVoxel voxels[TEST_POINTS];

  srand (static_cast<unsigned int> (time (NULL)));

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    for (size_t i = 0; i < TEST_POINTS; i++)
    {
      data[i] = rand ();
      voxels[i].x = rand () % 4096;
      voxels[i].y = rand () % 4096;
      voxels[i].z = rand () % 4096;
      // Add data to octree
      int* container = octree_a.createLeaf (voxels[i].x, voxels[i].y, voxels[i].z);
      *container = data[i];
    }

    // Test serialization - XOR tree binary data
    octree_a.serializeTree (tree_binary_a, leaf_vector_a, true);
    octree_b.deserializeTree (tree_binary_a, leaf_vector_a, true);
    octree_b.serializeTree (tree_binary_b, leaf_vector_b, true);

    // Check leaf count of rebuilt octree
    ASSERT_EQ (octree_a.getLeafCount (), octree_b.getLeafCount ());
    ASSERT_EQ (leaf_vector_b.size (), octree_b.getLeafCount ());
    ASSERT_EQ (leaf_vector_a.size (), leaf_vector_b.size ());
    ASSERT_EQ (tree_binary_a.size (), octree_b.getBranchCount ());
    ASSERT_EQ (tree_binary_a.size (), tree_binary_b.size ());

    // Check if octree octree structure is consistent.
    for (size_t i = 0; i < leaf_vector_b.size (); i++)
      ASSERT_TRUE (*leaf_vector_a[i] == *leaf_vector_b[i]);

    // Switch buffers
    octree_a.switchBuffers ();
    octree_b.switchBuffers ();
  }
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

