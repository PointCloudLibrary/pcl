/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2017-, Open Perception, Inc.
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
 *
 */
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_base.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <gtest/gtest.h>

using pcl::octree::OctreeBase;
using pcl::octree::OctreeIteratorBase;
using pcl::octree::OctreeKey;

////////////////////////////////////////////////////////
//                  OctreeIteratorBase
////////////////////////////////////////////////////////

struct OctreeIteratorBaseTest : public testing::Test
{
  // types
  typedef OctreeBase<int> OctreeBaseT;
  typedef OctreeIteratorBase<OctreeBaseT> OctreeIteratorBaseT;


  // methods
  virtual void SetUp ()
  {
    octree_.setTreeDepth (2); //can have at most 8^2 leaves
  }

  // members
  OctreeBaseT octree_;
};

TEST_F (OctreeIteratorBaseTest, CopyConstructor)
{
  OctreeIteratorBaseT it_a;
  OctreeIteratorBaseT it_b (&octree_, 0);
  OctreeIteratorBaseT it_c (it_b); //Our copy constructor

  EXPECT_NE (it_a, it_c);
  EXPECT_EQ (it_b, it_c);
}

TEST_F (OctreeIteratorBaseTest, CopyAssignment)
{
  OctreeIteratorBaseT it_a;
  OctreeIteratorBaseT it_b (&octree_, 0);
  OctreeIteratorBaseT it_c;

  EXPECT_EQ (it_a, it_c);
  EXPECT_NE (it_b, it_c);

  it_c = it_a; //Our copy assignment
  EXPECT_EQ (it_a, it_c);
  EXPECT_NE (it_b, it_c);

  it_c = it_b; //Our copy assignment
  EXPECT_NE (it_a, it_c);
  EXPECT_EQ (it_b, it_c);
}

////////////////////////////////////////////////////////
//        Iterator fixture setup
////////////////////////////////////////////////////////

template<typename T>
struct OctreeIteratorTest : public OctreeIteratorBaseTest
{
  // types
  typedef OctreeKey OctreeKeyT;

  // methods
  OctreeIteratorTest () : it_ (&octree_, tree_depth_) {}

  virtual void SetUp ()
  {
    // Set up my octree
    octree_.setTreeDepth (tree_depth_); //can have at most 8 leaves

    // Generate the unique key for our leaves
    keys_[0] = OctreeKeyT (0b0u, 0b0u, 0b0u);
    keys_[1] = OctreeKeyT (0b0u, 0b0u, 0b1u);
    keys_[2] = OctreeKeyT (0b0u, 0b1u, 0b0u);
    keys_[3] = OctreeKeyT (0b0u, 0b1u, 0b1u);
    keys_[4] = OctreeKeyT (0b1u, 0b0u, 0b0u);
    keys_[5] = OctreeKeyT (0b1u, 0b0u, 0b1u);
    keys_[6] = OctreeKeyT (0b1u, 0b1u, 0b0u);
    keys_[7] = OctreeKeyT (0b1u, 0b1u, 0b1u);

    // Create the leaves
    for (uint8_t i = 0; i < 8; ++i)
      octree_.createLeaf (keys_[i].x, keys_[i].y, keys_[i].z);

    // reset the iterator state
    it_.reset ();

    // increment the iterator 4 times
    for (uint8_t i = 0; i < 4; ++it_, ++i);
  }

  // members
  const static unsigned tree_depth_ = 1;

  OctreeKeyT keys_[8];

  T it_;
};

using pcl::octree::OctreeDepthFirstIterator;
using pcl::octree::OctreeBreadthFirstIterator;
using pcl::octree::OctreeLeafNodeDepthFirstIterator;
using pcl::octree::OctreeFixedDepthIterator;
using pcl::octree::OctreeLeafNodeBreadthFirstIterator;

typedef testing::Types<OctreeDepthFirstIterator<OctreeBase<int> >,
                       OctreeBreadthFirstIterator<OctreeBase<int> >,
                       OctreeLeafNodeDepthFirstIterator<OctreeBase<int> >,
                       OctreeFixedDepthIterator<OctreeBase<int> >,
                       OctreeLeafNodeBreadthFirstIterator<OctreeBase<int> > > OctreeIteratorTypes;
TYPED_TEST_CASE (OctreeIteratorTest, OctreeIteratorTypes);

TYPED_TEST (OctreeIteratorTest, CopyConstructor)
{
  TypeParam it_a;
  TypeParam it_b (this->it_); // copy ctor

  EXPECT_NE (it_a, it_b);
  EXPECT_EQ (this->it_, it_b);
  EXPECT_EQ (*this->it_, *it_b);
  EXPECT_EQ (this->it_.getNodeID (), it_b.getNodeID ());
  EXPECT_EQ (this->it_ == it_b, !(this->it_ != it_b));

  EXPECT_EQ (this->it_.getCurrentOctreeKey (), it_b.getCurrentOctreeKey ());
  EXPECT_EQ (this->it_.getCurrentOctreeDepth (), it_b.getCurrentOctreeDepth ());
  EXPECT_EQ (this->it_.getCurrentOctreeNode (), it_b.getCurrentOctreeNode ());

  EXPECT_EQ (this->it_.isBranchNode (), it_b.isBranchNode ());
  EXPECT_EQ (this->it_.isLeafNode (), it_b.isLeafNode ());

  EXPECT_EQ (this->it_.getNodeConfiguration (), it_b.getNodeConfiguration ());

  EXPECT_EQ (this->it_.getNodeID (), it_b.getNodeID ());
}

TYPED_TEST (OctreeIteratorTest, CopyAssignment)
{
  TypeParam it_a;
  TypeParam it_b (this->it_); // copy ctor
  TypeParam it_c;

  // Don't modify it
  EXPECT_EQ (it_a, it_c);
  EXPECT_NE (it_b, it_c);

  it_c = it_b; //Our copy assignment
  EXPECT_NE (it_a, it_c);
  EXPECT_EQ (it_b, it_c);
  EXPECT_EQ (*it_b, *it_c);
  EXPECT_EQ (it_b.getNodeID (), it_c.getNodeID ());
  EXPECT_EQ (it_b == it_c, !(it_b != it_c));

  it_c = it_a; //Our copy assignment
  EXPECT_EQ (it_a, it_c);
  EXPECT_NE (it_b, it_c);
}

////////////////////////////////////////////////////////
//        OctreeBase Begin/End Iterator Construction
////////////////////////////////////////////////////////

struct OctreeBaseBeginEndIteratorsTest : public testing::Test
{
  // Types
  typedef OctreeBase<int> OctreeT;

  // Methods
  void SetUp ()
  {
    // Set tree depth
    oct_a_.setTreeDepth (2);
    oct_b_.setTreeDepth (2);

    // Populate leaves 8^2 = 64 uids
    // 64 needs 6 bits in total spread among 3 keys (x, y and z).
    // 2 bits per key
    // The 3 LSBs of the id match the 1 LSB of the x, y and z keys
    // The 3 MSBs of the id match the 1 MSB of the x, y and z keys
    for (size_t i = 0; i < 64u; ++i)
    {
      const OctreeKey key (((i >> 4) & 0b10u) | ((i >> 2) & 1u), // x
                           ((i >> 3) & 0b10u) | ((i >> 1) & 1u), // y
                           ((i >> 2) & 0b10u) | (i & 1u));// z
      oct_a_.createLeaf (key.x, key.y, key.z);
      oct_b_.createLeaf (key.x, key.y, key.z);
    }
  }

  // Members
  OctreeT oct_a_, oct_b_;
};

TEST_F (OctreeBaseBeginEndIteratorsTest, Begin)
{
  // Useful types
  typedef typename OctreeT::Iterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.begin ();
  IteratorT it_a_2 = oct_a_.begin ();
  IteratorT it_b = oct_b_.begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.begin ();
  IteratorT it_m_1 = oct_a_.begin (1);
  IteratorT it_m_2 = oct_a_.begin (2);
  IteratorT it_m_b_1 = oct_b_.begin (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, End)
{
  // Useful types
  typedef typename OctreeT::Iterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.end ();
  IteratorT it_a_2 = oct_a_.end ();
  IteratorT it_b = oct_b_.end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, LeafBegin)
{
  // Useful types
  typedef typename OctreeT::LeafNodeDepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_depth_begin ();
  IteratorT it_a_2 = oct_a_.leaf_depth_begin ();
  IteratorT it_b = oct_b_.leaf_depth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_depth_begin ();
  IteratorT it_m_1 = oct_a_.leaf_depth_begin (1);
  IteratorT it_m_2 = oct_a_.leaf_depth_begin (2);
  IteratorT it_m_b_1 = oct_b_.leaf_depth_begin (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, LeafEnd)
{
  // Useful types
  typedef typename OctreeT::LeafNodeDepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_depth_end ();
  IteratorT it_a_2 = oct_a_.leaf_depth_end ();
  IteratorT it_b = oct_b_.leaf_depth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, DepthBegin)
{
  // Useful types
  typedef typename OctreeT::DepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.depth_begin ();
  IteratorT it_a_2 = oct_a_.depth_begin ();
  IteratorT it_b = oct_b_.depth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.depth_begin ();
  IteratorT it_m_1 = oct_a_.depth_begin (1);
  IteratorT it_m_2 = oct_a_.depth_begin (2);
  IteratorT it_m_b_1 = oct_b_.depth_begin (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, DepthEnd)
{
  // Useful types
  typedef typename OctreeT::DepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.depth_end ();
  IteratorT it_a_2 = oct_a_.depth_end ();
  IteratorT it_b = oct_b_.depth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, BreadthBegin)
{
  // Useful types
  typedef typename OctreeT::BreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.breadth_begin ();
  IteratorT it_a_2 = oct_a_.breadth_begin ();
  IteratorT it_b = oct_b_.breadth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.breadth_begin ();
  IteratorT it_m_1 = oct_a_.breadth_begin (1);
  IteratorT it_m_2 = oct_a_.breadth_begin (2);
  IteratorT it_m_b_1 = oct_b_.breadth_begin (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, BreadthEnd)
{
  // Useful types
  typedef typename OctreeT::BreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.breadth_end ();
  IteratorT it_a_2 = oct_a_.breadth_end ();
  IteratorT it_b = oct_b_.breadth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, LeafBreadthBegin)
{
  // Useful types
  typedef typename OctreeT::LeafNodeBreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_breadth_begin ();
  IteratorT it_a_2 = oct_a_.leaf_breadth_begin ();
  IteratorT it_b = oct_b_.leaf_breadth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_breadth_begin ();
  IteratorT it_m_1 = oct_a_.leaf_breadth_begin (1);
  IteratorT it_m_2 = oct_a_.leaf_breadth_begin (2);
  IteratorT it_m_b_1 = oct_b_.leaf_breadth_begin (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, LeafBreadthEnd)
{
  // Useful types
  typedef typename OctreeT::LeafNodeBreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_breadth_end ();
  IteratorT it_a_2 = oct_a_.leaf_breadth_end ();
  IteratorT it_b = oct_b_.leaf_breadth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

////////////////////////////////////////////////////////
//        OctreeBase Iterator For Loop Case
////////////////////////////////////////////////////////

struct OctreeBaseIteratorsForLoopTest : public OctreeBaseBeginEndIteratorsTest
{
};

TEST_F (OctreeBaseIteratorsForLoopTest, DefaultIterator)
{
  // Useful types
  typedef typename OctreeT::Iterator IteratorT;

  // Default initialization
  IteratorT it_a;
  IteratorT it_a_end = oct_a_.end ();

  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  // Iterate over every node of the octree oct_a_.
  for (it_a = oct_a_.begin (); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 64);
  ASSERT_EQ (branch_count, 9);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (oct_a_.getLeafCount (), leaf_count);
  ASSERT_EQ (oct_a_.getBranchCount (), branch_count);

  // Iterate over the octree oct_a_ with a depth max of 1.
  // As oct_a_ has a depth level of 2, we should only iterate
  // over 9 branch node: the root node + 8 node at depth 1
  node_count = 0;
  branch_count = 0;
  leaf_count = 0;
  unsigned int max_depth = 1;
  for (it_a = oct_a_.begin (max_depth); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 0);
  ASSERT_EQ (branch_count, 9);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (oct_a_.getBranchCount (), branch_count);
}

TEST_F (OctreeBaseIteratorsForLoopTest, LeafNodeDepthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::LeafNodeDepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a;
  IteratorT it_a_end = oct_a_.leaf_depth_end ();

  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  // Iterate over every node of the octree oct_a_.
  for (it_a = oct_a_.leaf_depth_begin (); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
  }

  // Check the branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 64);
  ASSERT_EQ (branch_count, 0);
  ASSERT_EQ (oct_a_.getLeafCount (), leaf_count);

  // Iterate over the octree oct_a_ with a depth max of 1.
  // As oct_a_ has a depth level of 2, we should only iterate
  // over 9 branch node: the root node + 8 node at depth 1
  branch_count = 0;
  leaf_count = 0;
  unsigned int max_depth = 1;
  for (it_a = oct_a_.leaf_depth_begin (max_depth); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
  }

  // Check the branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 0);
  ASSERT_EQ (branch_count, 0);
}

TEST_F (OctreeBaseIteratorsForLoopTest, DepthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::DepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a;
  IteratorT it_a_end = oct_a_.depth_end ();

  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  // Iterate over every node of the octree oct_a_.
  for (it_a = oct_a_.depth_begin (); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 64);
  ASSERT_EQ (branch_count, 9);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (oct_a_.getLeafCount (), leaf_count);
  ASSERT_EQ (oct_a_.getBranchCount (), branch_count);

  // Iterate over the octree oct_a_ with a depth max of 1.
  // As oct_a_ has a depth level of 2, we should only iterate
  // over 9 branch node: the root node + 8 node at depth 1
  node_count = 0;
  branch_count = 0;
  leaf_count = 0;
  unsigned int max_depth = 1;
  for (it_a = oct_a_.depth_begin (max_depth); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 0);
  ASSERT_EQ (branch_count, 9);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (oct_a_.getBranchCount (), branch_count);
}

TEST_F (OctreeBaseIteratorsForLoopTest, BreadthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::BreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a;
  IteratorT it_a_end = oct_a_.breadth_end ();

  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  // Iterate over every node of the octree oct_a_.
  for (it_a = oct_a_.breadth_begin (); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 64);
  ASSERT_EQ (branch_count, 9);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (oct_a_.getLeafCount (), leaf_count);
  ASSERT_EQ (oct_a_.getBranchCount (), branch_count);

  // Iterate over the octree oct_a_ with a depth max of 1.
  // As oct_a_ has a depth level of 2, we should only iterate
  // over 9 branch node: the root node + 8 node at depth 1
  node_count = 0;
  branch_count = 0;
  leaf_count = 0;
  unsigned int max_depth = 1;
  for (it_a = oct_a_.breadth_begin (max_depth); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 0);
  ASSERT_EQ (branch_count, 9);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (oct_a_.getBranchCount (), branch_count);
}

TEST_F (OctreeBaseIteratorsForLoopTest, FixedDepthIterator)
{
  // Useful types
  typedef typename OctreeT::FixedDepthIterator IteratorT;

  // Default initialization
  IteratorT it_a;
  IteratorT it_a_end = oct_a_.fixed_depth_end ();

  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  for (unsigned int depth = 0; depth <= oct_a_.getTreeDepth (); ++depth)
  {
    // Iterate over every node of the octree oct_a_.
    for (it_a = oct_a_.fixed_depth_begin (depth); it_a != it_a_end; ++it_a)
    {
      // store node, branch and leaf count
      const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
      if (node->getNodeType () == pcl::octree::BRANCH_NODE)
      {
        branch_count++;
      }
      else if (node->getNodeType () == pcl::octree::LEAF_NODE)
      {
        leaf_count++;
      }
      node_count++;
    }
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 64);
  ASSERT_EQ (branch_count, 9);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (oct_a_.getLeafCount (), leaf_count);
  ASSERT_EQ (oct_a_.getBranchCount (), branch_count);

  // Iterate over the octree oct_a_ with a depth max of 1.
  // As oct_a_ has a depth level of 2, we should only iterate
  // over 9 branch node: the root node + 8 node at depth 1
  node_count = 0;
  branch_count = 0;
  leaf_count = 0;
  unsigned int fixed_depth = 1;
  for (it_a = oct_a_.fixed_depth_begin (fixed_depth); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 0);
  ASSERT_EQ (branch_count, 8);
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ ((oct_a_.getBranchCount () - 1), branch_count);
}

TEST_F (OctreeBaseIteratorsForLoopTest, LeafNodeBreadthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::LeafNodeBreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a;
  IteratorT it_a_end = oct_a_.leaf_breadth_end ();

  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  // Iterate over every node of the octree oct_a_.
  for (it_a = oct_a_.leaf_breadth_begin (); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 64);
  ASSERT_EQ (branch_count, 0);
  ASSERT_EQ (node_count, 64);
  ASSERT_EQ (oct_a_.getLeafCount (), leaf_count);

  // Iterate over the octree oct_a_ with a depth max of 1.
  // As oct_a_ has a depth level of 2, we should only iterate
  // over 9 branch node: the root node + 8 node at depth 1
  node_count = 0;
  branch_count = 0;
  leaf_count = 0;
  unsigned int max_depth = 1;
  for (it_a = oct_a_.leaf_breadth_begin (max_depth); it_a != it_a_end; ++it_a)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it_a.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  // Check the node_count, branch_count and leaf_count values
  ASSERT_EQ (leaf_count, 0);
  ASSERT_EQ (branch_count, 0);
  ASSERT_EQ (node_count, 0);
}

////////////////////////////////////////////////////////
//        OctreeBase Walk Through Iterator Test
////////////////////////////////////////////////////////

struct OctreeBaseWalkThroughIteratorsTest : public testing::Test
{
  // Types
  typedef OctreeBase<int> OctreeT;

  // Methods
  void SetUp ()
  {
    // Create manually an irregular octree.
    // Graphically, this octree appears as follows:
    //          root
    //        ' /  \ `
    //     '   /    \   `
    //  '     /      \     `
    // 000  010      100  110
    //       |             |
    //       |             |
    //      020           220
    //
    // The octree key of the different node are represented on this graphic.
    // This octree is of max_depth 2.
    // At depth 1, you will find:
    // - 2 leaf nodes  , with the keys 000 and 100,
    // - 2 branch nodes, with the keys 010 and 110.
    // At depth 2, you will find:
    // - 2 leaf nodes  , with the keys 000 and 000.
    // This octree is build to be able to check the order in which the nodes
    // appear depending on the used iterator.

    // Set the leaf nodes at depth 1
    oct_a_.setTreeDepth (1);

    oct_a_.createLeaf (0u, 0u, 0u);
    oct_a_.createLeaf (1u, 0u, 0u);

    // Set the leaf nodes at depth 2. As createLeaf method create recursively
    // the octree nodes, if the parent node are not present, they will be create.
    // In this case, at depth 1, the nodes 010 and 110 are created.
    oct_a_.setTreeDepth (2);

    oct_a_.createLeaf (0u, 2u, 0u);
    oct_a_.createLeaf (2u, 2u, 0u);
  }

  // Members
  OctreeT oct_a_;
};

TEST_F (OctreeBaseWalkThroughIteratorsTest, LeafNodeDepthFirstIterator)
{
  OctreeT::LeafNodeDepthFirstIterator it = oct_a_.leaf_depth_begin ();

  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (2u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.leaf_depth_end ());
}

TEST_F (OctreeBaseWalkThroughIteratorsTest, LeafNodeBreadthFirstIterator)
{
  OctreeT::LeafNodeBreadthFirstIterator it = oct_a_.leaf_breadth_begin ();

  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (2u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.leaf_breadth_end ());
}

TEST_F (OctreeBaseWalkThroughIteratorsTest, DepthFirstIterator)
{
  OctreeT::DepthFirstIterator it = oct_a_.depth_begin ();

  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 0
  EXPECT_EQ (it.getCurrentOctreeDepth (), 0u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 1u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 1u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (2u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.depth_end ());
}

TEST_F (OctreeBaseWalkThroughIteratorsTest, BreadthFirstIterator)
{
  OctreeT::BreadthFirstIterator it = oct_a_.breadth_begin ();

  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 0
  EXPECT_EQ (it.getCurrentOctreeDepth (), 0u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 1u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 1u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (2u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.breadth_end ());
}

TEST_F (OctreeBaseWalkThroughIteratorsTest, FixedDepthIterator)
{
  OctreeT::FixedDepthIterator it;

  // Check the default behavior of the iterator
  it = oct_a_.fixed_depth_begin ();
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 0
  EXPECT_EQ (it.getCurrentOctreeDepth (), 0u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.fixed_depth_end ());

  // Check the iterator at depth 0
  it = oct_a_.fixed_depth_begin (0);
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 0
  EXPECT_EQ (it.getCurrentOctreeDepth (), 0u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.fixed_depth_end ());

  // Check the iterator at depth 1
  it = oct_a_.fixed_depth_begin (1);
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 1u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 0u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (1u, 1u, 0u)); // depth: 1
  EXPECT_EQ (it.getCurrentOctreeDepth (), 1u);
  EXPECT_TRUE (it.isBranchNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.fixed_depth_end ());

  // Check the iterator at depth 2
  it = oct_a_.fixed_depth_begin (2);
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (0u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it.getCurrentOctreeKey (), OctreeKey (2u, 2u, 0u)); // depth: 2
  EXPECT_EQ (it.getCurrentOctreeDepth (), 2u);
  EXPECT_TRUE (it.isLeafNode ());
  ++it;
  EXPECT_EQ (it, oct_a_.fixed_depth_end ());
}

////////////////////////////////////////////////////////
//        OctreeBase Iterator Pre/Post increment
////////////////////////////////////////////////////////

struct OctreeBaseIteratorsPrePostTest : public OctreeBaseBeginEndIteratorsTest
{
};

TEST_F (OctreeBaseIteratorsPrePostTest, DefaultIterator)
{
  // Useful types
  typedef typename OctreeT::Iterator IteratorT;

  // Default initialization
  IteratorT it_a_pre;
  IteratorT it_a_post;
  IteratorT it_a_end = oct_a_.end ();

  // Iterate over every node of the octree oct_a_.
  for (it_a_pre = oct_a_.begin (), it_a_post = oct_a_.begin ();
       ((it_a_pre != it_a_end) && (it_a_post != it_a_end)); )
  {
    EXPECT_EQ (it_a_pre, it_a_post++);
    EXPECT_EQ (++it_a_pre, it_a_post);
  }

  EXPECT_EQ (it_a_pre, it_a_end);
  EXPECT_EQ (it_a_post, it_a_end);
}

TEST_F (OctreeBaseIteratorsPrePostTest, LeafNodeDepthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::LeafNodeDepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_pre;
  IteratorT it_a_post;
  IteratorT it_a_end = oct_a_.leaf_depth_end ();

  // Iterate over every node of the octree oct_a_.
  for (it_a_pre = oct_a_.leaf_depth_begin (), it_a_post = oct_a_.leaf_depth_begin ();
       ((it_a_pre != it_a_end) && (it_a_post != it_a_end)); )
  {
    EXPECT_EQ (it_a_pre, it_a_post++);
    EXPECT_EQ (++it_a_pre, it_a_post);
  }

  EXPECT_EQ (it_a_pre, it_a_end);
  EXPECT_EQ (it_a_post, it_a_end);
}

TEST_F (OctreeBaseIteratorsPrePostTest, DepthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::DepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_pre;
  IteratorT it_a_post;
  IteratorT it_a_end = oct_a_.depth_end ();

  // Iterate over every node of the octree oct_a_.
  for (it_a_pre = oct_a_.depth_begin (), it_a_post = oct_a_.depth_begin ();
       ((it_a_pre != it_a_end) && (it_a_post != it_a_end)); )
  {
    EXPECT_EQ (it_a_pre, it_a_post++);
    EXPECT_EQ (++it_a_pre, it_a_post);
  }

  EXPECT_EQ (it_a_pre, it_a_end);
  EXPECT_EQ (it_a_post, it_a_end);
}

TEST_F (OctreeBaseIteratorsPrePostTest, BreadthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::BreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_pre;
  IteratorT it_a_post;
  IteratorT it_a_end = oct_a_.breadth_end ();

  // Iterate over every node of the octree oct_a_.
  for (it_a_pre = oct_a_.breadth_begin (), it_a_post = oct_a_.breadth_begin ();
       ((it_a_pre != it_a_end) && (it_a_post != it_a_end)); )
  {
    EXPECT_EQ (it_a_pre, it_a_post++);
    EXPECT_EQ (++it_a_pre, it_a_post);
  }

  EXPECT_EQ (it_a_pre, it_a_end);
  EXPECT_EQ (it_a_post, it_a_end);
}

TEST_F (OctreeBaseIteratorsPrePostTest, FixedDepthIterator)
{
  // Useful types
  typedef typename OctreeT::FixedDepthIterator IteratorT;

  // Default initialization
  IteratorT it_a_pre;
  IteratorT it_a_post;
  IteratorT it_a_end = oct_a_.fixed_depth_end ();

  for (unsigned int depth = 0; depth <= oct_a_.getTreeDepth (); ++depth)
  {
    it_a_pre = oct_a_.fixed_depth_begin (depth);
    it_a_post = oct_a_.fixed_depth_begin (depth);


    // Iterate over every node at a given depth of the octree oct_a_.
    for (it_a_pre = oct_a_.fixed_depth_begin (depth), it_a_post = oct_a_.fixed_depth_begin (depth);
         ((it_a_pre != it_a_end) && (it_a_post != it_a_end)); )
    {
      EXPECT_EQ (it_a_pre, it_a_post++);
      EXPECT_EQ (++it_a_pre, it_a_post);
    }

    EXPECT_EQ (it_a_pre, it_a_end);
    EXPECT_EQ (it_a_post, it_a_end);
  }
}

TEST_F (OctreeBaseIteratorsPrePostTest, LeafNodeBreadthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::LeafNodeBreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_pre;
  IteratorT it_a_post;
  IteratorT it_a_end = oct_a_.leaf_breadth_end ();

  // Iterate over every node of the octree oct_a_.
  for (it_a_pre = oct_a_.leaf_breadth_begin (), it_a_post = oct_a_.leaf_breadth_begin ();
       ((it_a_pre != it_a_end) && (it_a_post != it_a_end)); )
  {
    EXPECT_EQ (it_a_pre, it_a_post++);
    EXPECT_EQ (++it_a_pre, it_a_post);
  }

  EXPECT_EQ (it_a_pre, it_a_end);
  EXPECT_EQ (it_a_post, it_a_end);
}

////////////////////////////////////////////////////////
//     OctreePointCloudAdjacency Begin/End Iterator Construction
////////////////////////////////////////////////////////

struct OctreePointCloudAdjacencyBeginEndIteratorsTest
  : public testing::Test
{
  // Types
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef pcl::octree::OctreePointCloudAdjacency<PointT> OctreeT;

  // Methods
  OctreePointCloudAdjacencyBeginEndIteratorsTest ()
    : oct_a_ (1)
    , oct_b_ (1)
  {}

  void SetUp ()
  {
    // Replicable results
    std::srand (42);

    // Generate Point Cloud
    typename PointCloudT::Ptr cloud (new PointCloudT (100, 1));
    const float max_inv = 1.f / float (RAND_MAX);
    for (size_t i = 0; i < 100; ++i)
    {
      const PointT pt (10.f * (float (std::rand ()) * max_inv - .5f),
                       10.f * (float (std::rand ()) * max_inv - .5f),
                       10.f * (float (std::rand ()) * max_inv - .5f));
      (*cloud)[i] = pt;
    }

    // Add points to octree
    oct_a_.setInputCloud (cloud);
    oct_a_.addPointsFromInputCloud ();

    oct_b_.setInputCloud (cloud);
    oct_b_.addPointsFromInputCloud ();
  }

  // Members
  OctreeT oct_a_, oct_b_;
};

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, LeafDepthBegin)
{
  // Useful types
  typedef typename OctreeT::LeafNodeDepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_depth_begin ();
  IteratorT it_a_2 = oct_a_.leaf_depth_begin ();
  IteratorT it_b = oct_b_.leaf_depth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_depth_begin ();
  IteratorT it_m_1 = oct_a_.leaf_depth_begin (1);
  IteratorT it_m_md = oct_a_.leaf_depth_begin (oct_a_.getTreeDepth ());
  IteratorT it_m_b_1 = oct_b_.leaf_depth_begin (1);

  EXPECT_NE (it_m_1, it_m_md);
  EXPECT_EQ (it_m_md, it_m); // should default to tree depth
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, LeafDepthEnd)
{
  // Useful types
  typedef typename OctreeT::LeafNodeDepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_depth_end ();
  IteratorT it_a_2 = oct_a_.leaf_depth_end ();
  IteratorT it_b = oct_b_.leaf_depth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, DepthBegin)
{
  // Useful types
  typedef typename OctreeT::DepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.depth_begin ();
  IteratorT it_a_2 = oct_a_.depth_begin ();
  IteratorT it_b = oct_b_.depth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.depth_begin ();
  IteratorT it_m_1 = oct_a_.depth_begin (1);
  IteratorT it_m_md = oct_a_.depth_begin (oct_a_.getTreeDepth ());
  IteratorT it_m_b_1 = oct_b_.depth_begin (1);

  EXPECT_NE (it_m_1, it_m_md);
  EXPECT_EQ (it_m_md, it_m); // should default to tree depth
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, DepthEnd)
{
  // Useful types
  typedef typename OctreeT::DepthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.depth_end ();
  IteratorT it_a_2 = oct_a_.depth_end ();
  IteratorT it_b = oct_b_.depth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, BreadthBegin)
{
  // Useful types
  typedef typename OctreeT::BreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.breadth_begin ();
  IteratorT it_a_2 = oct_a_.breadth_begin ();
  IteratorT it_b = oct_b_.breadth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.breadth_begin ();
  IteratorT it_m_1 = oct_a_.breadth_begin (1);
  IteratorT it_m_md = oct_a_.breadth_begin (oct_a_.getTreeDepth ());
  IteratorT it_m_b_1 = oct_b_.breadth_begin (1);

  EXPECT_NE (it_m_1, it_m_md);
  EXPECT_EQ (it_m_md, it_m); // should default to tree depth
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, BreadthEnd)
{
  // Useful types
  typedef typename OctreeT::BreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.breadth_end ();
  IteratorT it_a_2 = oct_a_.breadth_end ();
  IteratorT it_b = oct_b_.breadth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, FixedDepthBegin)
{
  // Useful types
  typedef typename OctreeT::FixedDepthIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.fixed_depth_begin ();
  IteratorT it_a_2 = oct_a_.fixed_depth_begin ();
  IteratorT it_b = oct_b_.fixed_depth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_f = oct_a_.fixed_depth_begin ();
  IteratorT it_f_1 = oct_a_.fixed_depth_begin (1);
  IteratorT it_f_fd = oct_a_.fixed_depth_begin (oct_a_.getTreeDepth ());
  IteratorT it_f_0 = oct_a_.fixed_depth_begin (0);
  IteratorT it_f_b_1 = oct_b_.fixed_depth_begin (1);

  EXPECT_NE (it_f_1, it_f_fd);
  EXPECT_NE (it_f_fd, it_f);
  EXPECT_EQ (it_f_0, it_f); // should default to root tree
  EXPECT_NE (it_f_1, it_f_b_1);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, FixedDepthEnd)
{
  // Useful types
  typedef typename OctreeT::FixedDepthIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.fixed_depth_end ();
  IteratorT it_a_2 = oct_a_.fixed_depth_end ();
  IteratorT it_b = oct_b_.fixed_depth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, LeafBreadthBegin)
{
  // Useful types
  typedef typename OctreeT::LeafNodeBreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_breadth_begin ();
  IteratorT it_a_2 = oct_a_.leaf_breadth_begin ();
  IteratorT it_b = oct_b_.leaf_breadth_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_breadth_begin ();
  IteratorT it_m_1 = oct_a_.leaf_breadth_begin (1);
  IteratorT it_m_md = oct_a_.leaf_breadth_begin (oct_a_.getTreeDepth ());
  IteratorT it_m_b_1 = oct_b_.leaf_breadth_begin (1);

  EXPECT_NE (it_m_1, it_m_md);
  EXPECT_EQ (it_m_md, it_m); // should default to tree depth
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, LeafBreadthEnd)
{
  // Useful types
  typedef typename OctreeT::LeafNodeBreadthFirstIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_breadth_end ();
  IteratorT it_a_2 = oct_a_.leaf_breadth_end ();
  IteratorT it_b = oct_b_.leaf_breadth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);
}

////////////////////////////////////////////////////////
//     OctreePointCloudSierpinski Iterator Traversal Test
////////////////////////////////////////////////////////

struct OctreePointCloudSierpinskiTest
  : public testing::Test
{
  // Types
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef pcl::octree::OctreePointCloud<PointT> OctreeT;

  // Methods
  OctreePointCloudSierpinskiTest ()
    : oct_ (1)
    , depth_ (7)
  {}

  void SetUp ()
  {
    // Create a point cloud which points are inside Sierpinski fractal voxel at the deepest level
    // https://en.wikipedia.org/wiki/Sierpinski_triangle
    typename PointCloudT::Ptr cloud (new PointCloudT);

    // The voxels will be generate between the points (0, 0, 0) and (1, 1, 1)
    Eigen::Vector3f v_min (0, 0, 0);
    Eigen::Vector3f v_max (1, 1, 1);

    // Generate Sierpinski fractal voxel at the deepest level
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > voxels (generateSierpinskiVoxelExtremities (v_min, v_max, depth_));

    // The number of points in each voxel
    unsigned int total_nb_pt = 100000;
    unsigned int nb_pt_in_voxel = total_nb_pt / pow (4, depth_);

    // Replicable results
    std::srand (42);

    // Fill the point cloud
    for (std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >::const_iterator it = voxels.begin ();
        it != voxels.end ();
        ++it)
    {
      const static float eps = std::numeric_limits<float>::epsilon ();
      double x_min = it->first.x () + eps;
      double y_min = it->first.y () + eps;
      double z_min = it->first.z () + eps;
      double x_max = it->second.x () - eps;
      double y_max = it->second.y () - eps;
      double z_max = it->second.z () - eps;

      for (unsigned int i = 0; i < nb_pt_in_voxel; ++i)
      {
        float x = x_min + (rand () / ((float)(RAND_MAX) + 1)) * (x_max - x_min);
        float y = y_min + (rand () / ((float)(RAND_MAX) + 1)) * (y_max - y_min);
        float z = z_min + (rand () / ((float)(RAND_MAX) + 1)) * (z_max - z_min);

        cloud->points.push_back (PointT (x, y, z));
      }
    }

    // Setup the octree
    double resolution = 1.0 / (pow (2, depth_));
    oct_.setResolution (resolution);

    // Add points to octree
    oct_.setInputCloud (cloud);

    // Bounding box
    oct_.defineBoundingBox (0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    // Add points from cloud to octree
    oct_.addPointsFromInputCloud ();
  }

  // Generate a vector of Sierpinski voxels at a given 'depth_arg'
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >
  generateSierpinskiVoxelExtremities (const Eigen::Vector3f & v_min, const Eigen::Vector3f & v_max,
                                      const unsigned int & depth_arg)
  {
    std::pair<Eigen::Vector3f, Eigen::Vector3f> voxel (v_min, v_max);
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > voxels;
    voxels.push_back (voxel);

    for (unsigned int i = 0; i < depth_arg; ++i)
    {
      voxels = generateSierpinskiVoxelExtremitiesAux (voxels);
    }

    return voxels;
  }

  // Apply a recursion step to a vector of macro Sierpinski voxel
  // For each voxel in the input vector 'voxels_in', 4 sub-voxels are generated
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >
  generateSierpinskiVoxelExtremitiesAux (const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > & voxels_in)
  {
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > voxels_out;

    for (std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >::const_iterator it = voxels_in.begin ();
        it != voxels_in.end ();
        ++it)
      {
        Eigen::Vector3f v_min = it->first;
        Eigen::Vector3f v_max = it->second;
        Eigen::Vector3f v_mid = 0.5 * (v_min + v_max);

        std::pair<Eigen::Vector3f, Eigen::Vector3f> voxel_0;
        std::pair<Eigen::Vector3f, Eigen::Vector3f> voxel_1;
        std::pair<Eigen::Vector3f, Eigen::Vector3f> voxel_2;
        std::pair<Eigen::Vector3f, Eigen::Vector3f> voxel_3;

        voxel_0.first  = v_min;
        voxel_0.second = v_mid;
        voxel_1.first  = Eigen::Vector3f (v_min.x (), v_mid.y (), v_mid.z ());
        voxel_1.second = Eigen::Vector3f (v_mid.x (), v_max.y (), v_max.z ());
        voxel_2.first  = Eigen::Vector3f (v_mid.x (), v_min.y (), v_mid.z ());
        voxel_2.second = Eigen::Vector3f (v_max.x (), v_mid.y (), v_max.z ());
        voxel_3.first  = Eigen::Vector3f (v_mid.x (), v_mid.y (), v_min.z ());
        voxel_3.second = Eigen::Vector3f (v_max.x (), v_max.y (), v_mid.z ());

        voxels_out.push_back (voxel_0);
        voxels_out.push_back (voxel_1);
        voxels_out.push_back (voxel_2);
        voxels_out.push_back (voxel_3);
      }

    return voxels_out;
  }

  /** \brief Computes the total number of parent nodes at the specified depth
    *
    * The octree is built such that the number of the leaf nodes is equal to
    * 4^depth and the number of branch nodes is egal to (4^depth -1)/(4 - 1),
    * where depth is the detph of the octree. The details of the expression
    * provided for the number of branch nodes could be found at:
    * https://en.wikipedia.org/wiki/Geometric_progression#Geometric_series
    * \param[in] depth - The depth of the octree
    * \return The total number of parent nodes at the specified depth level.
    */
  static unsigned
  computeTotalParentNodeCount (const unsigned depth)
  {
    return (pow (4, depth) - 1) / (4 - 1);
  }

  // Members
  OctreeT oct_;

  const unsigned depth_;
};

/** \brief Octree test based on a Sierpinski fractal
 */
TEST_F (OctreePointCloudSierpinskiTest, DefaultIterator)
{
  // Useful types
  typedef typename OctreeT::Iterator IteratorT;

  // Check the number of branch and leaf nodes
  ASSERT_EQ (oct_.getLeafCount (), pow (4, depth_));
  ASSERT_EQ (oct_.getBranchCount (), computeTotalParentNodeCount (depth_));

  IteratorT it;
  IteratorT it_end = oct_.end ();

  // Check the number of leaf and branch nodes
  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  for (it = oct_.begin (); it != it_end; ++it)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  EXPECT_EQ (leaf_count, pow (4, depth_));
  EXPECT_EQ (branch_count, computeTotalParentNodeCount (depth_));
  EXPECT_EQ (node_count, computeTotalParentNodeCount (depth_ + 1));

  // Check the specific key/child_idx value for this octree
  for (it = oct_.begin (); it != it_end; ++it)
  {
    for (unsigned int i = 0; i < depth_; ++i)
    {
      // The binary representation child_idx can only contain an even number of 1
      int child_idx = it.getCurrentOctreeKey ().getChildIdxWithDepthMask (1 << i);
      EXPECT_TRUE ((child_idx == 0) || (child_idx == 3) || (child_idx == 5) || (child_idx == 6));
    }
  }
}

TEST_F (OctreePointCloudSierpinskiTest, LeafNodeDepthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::LeafNodeDepthFirstIterator IteratorT;

  // Check the number of branch and leaf nodes
  ASSERT_EQ (oct_.getLeafCount (), pow (4, depth_));
  ASSERT_EQ (oct_.getBranchCount (), computeTotalParentNodeCount (depth_));

  IteratorT it;
  IteratorT it_end = oct_.leaf_depth_end ();

  // Check the number of leaf and branch nodes
  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  for (it = oct_.leaf_depth_begin (); it != it_end; ++it)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  EXPECT_EQ (leaf_count, pow (4, depth_));
  EXPECT_EQ (branch_count, 0);
  EXPECT_EQ (node_count, pow (4, depth_));

  // Check the specific key/child_idx value for this octree
  for (it = oct_.leaf_depth_begin (); it != it_end; ++it)
  {
    for (unsigned int i = 0; i < depth_; ++i)
    {
      // The binary representation child_idx can only contain an even number of 1
      int child_idx = it.getCurrentOctreeKey ().getChildIdxWithDepthMask (1 << i);
      EXPECT_TRUE ((child_idx == 0) || (child_idx == 3) || (child_idx == 5) || (child_idx == 6));
    }
  }
}

TEST_F (OctreePointCloudSierpinskiTest, DepthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::DepthFirstIterator IteratorT;

  // Check the number of branch and leaf nodes
  ASSERT_EQ (oct_.getLeafCount (), pow (4, depth_));
  ASSERT_EQ (oct_.getBranchCount (), computeTotalParentNodeCount (depth_));

  IteratorT it;
  IteratorT it_end = oct_.depth_end ();

  // Check the number of leaf and branch nodes
  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  for (it = oct_.depth_begin (); it != it_end; ++it)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  EXPECT_EQ (leaf_count, pow (4, depth_));
  EXPECT_EQ (branch_count, computeTotalParentNodeCount (depth_));
  EXPECT_EQ (node_count, computeTotalParentNodeCount (depth_ + 1));

  // Check the specific key/child_idx value for this octree
  for (it = oct_.depth_begin (); it != it_end; ++it)
  {
    for (unsigned int i = 0; i < depth_; ++i)
    {
      // The binary representation child_idx can only contain an even number of 1
      int child_idx = it.getCurrentOctreeKey ().getChildIdxWithDepthMask (1 << i);
      EXPECT_TRUE ((child_idx == 0) || (child_idx == 3) || (child_idx == 5) || (child_idx == 6));
    }
  }
}

TEST_F (OctreePointCloudSierpinskiTest, BreadthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::BreadthFirstIterator IteratorT;

  // Check the number of branch and leaf nodes
  ASSERT_EQ (oct_.getLeafCount (), pow (4, depth_));
  ASSERT_EQ (oct_.getBranchCount (), computeTotalParentNodeCount (depth_));

  IteratorT it;
  IteratorT it_end = oct_.breadth_end ();

  // Check the number of leaf and branch nodes
  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  for (it = oct_.breadth_begin (); it != it_end; ++it)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  EXPECT_EQ (leaf_count, pow (4, depth_));
  EXPECT_EQ (branch_count, computeTotalParentNodeCount (depth_));
  EXPECT_EQ (node_count, computeTotalParentNodeCount (depth_ + 1));

  // Check the specific key/child_idx value for this octree
  for (it = oct_.breadth_begin (); it != it_end; ++it)
  {
    for (unsigned int i = 0; i < depth_; ++i)
    {
      // The binary representation child_idx can only contain an even number of 1
      int child_idx = it.getCurrentOctreeKey ().getChildIdxWithDepthMask (1 << i);
      EXPECT_TRUE ((child_idx == 0) || (child_idx == 3) || (child_idx == 5) || (child_idx == 6));
    }
  }
}

TEST_F (OctreePointCloudSierpinskiTest, FixedDepthIterator)
{
  // Useful types
  typedef typename OctreeT::FixedDepthIterator IteratorT;

  // Check the number of branch and leaf nodes
  ASSERT_EQ (oct_.getLeafCount (), pow (4, depth_));
  ASSERT_EQ (oct_.getBranchCount (), computeTotalParentNodeCount (depth_));

  IteratorT it;
  IteratorT it_end = oct_.fixed_depth_end ();

  // Check the number of nodes at each level of the octree
  unsigned int nb_nodes;
  for (unsigned int idx_depth = 1; idx_depth <= depth_; ++idx_depth)
  {
    nb_nodes = 0;
    for (it = oct_.fixed_depth_begin (idx_depth); it != it_end; ++it)
    {
      ASSERT_EQ (it.getCurrentOctreeDepth (), idx_depth);
      ++nb_nodes;
    }

    ASSERT_EQ (nb_nodes, pow (4, idx_depth));
  }

  // Check the specific key/child_idx value for this octree
  for (it = oct_.fixed_depth_begin (depth_); it != it_end; ++it)
  {
    for (unsigned int i = 0; i < depth_; ++i)
    {
      // The binary representation child_idx can only contain an even number of 1
      int child_idx = it.getCurrentOctreeKey ().getChildIdxWithDepthMask (1 << i);
      ASSERT_TRUE ((child_idx == 0) || (child_idx == 3) || (child_idx == 5) || (child_idx == 6));
    }
  }
}

TEST_F (OctreePointCloudSierpinskiTest, LeafNodeBreadthFirstIterator)
{
  // Useful types
  typedef typename OctreeT::LeafNodeBreadthFirstIterator IteratorT;

  // Check the number of branch and leaf nodes
  ASSERT_EQ (oct_.getLeafCount (), pow (4, depth_));
  ASSERT_EQ (oct_.getBranchCount (), (pow (4, depth_) - 1) / (4 - 1));

  IteratorT it;
  IteratorT it_end = oct_.leaf_breadth_end ();

  // Check the number of leaf and branch nodes
  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  for (it = oct_.leaf_breadth_begin (); it != it_end; ++it)
  {
    // store node, branch and leaf count
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode ();
    if (node->getNodeType () == pcl::octree::BRANCH_NODE)
    {
      branch_count++;
    }
    else if (node->getNodeType () == pcl::octree::LEAF_NODE)
    {
      leaf_count++;
    }
    node_count++;
  }

  EXPECT_EQ (leaf_count, pow (4, depth_));
  EXPECT_EQ (branch_count, 0);
  EXPECT_EQ (node_count, pow (4, depth_));

  // Check the specific key/child_idx value for this octree
  for (it = oct_.leaf_breadth_begin (); it != it_end; ++it)
  {
    for (unsigned int i = 0; i < depth_; ++i)
    {
      // The binary representation child_idx can only contain an even number of 1
      int child_idx = it.getCurrentOctreeKey ().getChildIdxWithDepthMask (1 << i);
      EXPECT_TRUE ((child_idx == 0) || (child_idx == 3) || (child_idx == 5) || (child_idx == 6));
    }
  }
}

int
main (int argc, char** const argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
