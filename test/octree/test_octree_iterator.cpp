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
using pcl::octree::OctreeLeafNodeIterator;

typedef testing::Types<OctreeDepthFirstIterator<OctreeBase<int> >,
                       OctreeBreadthFirstIterator<OctreeBase<int> >,
                       OctreeLeafNodeIterator<OctreeBase<int> > > OctreeIteratorTypes;
TYPED_TEST_CASE(OctreeIteratorTest, OctreeIteratorTypes);

TYPED_TEST (OctreeIteratorTest, CopyConstructor)
{
  TypeParam it_a;
  TypeParam it_b (this->it_); // copy ctor

  EXPECT_NE (it_a, it_b);
  EXPECT_EQ (this->it_, it_b);
  EXPECT_EQ (*this->it_, *it_b);
  EXPECT_EQ (this->it_.getNodeID (), it_b.getNodeID ());
  EXPECT_EQ (this->it_ == it_b, !(this->it_ != it_b));
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
//        Octree End Begin/End Iterator Construction
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

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.end ();
  IteratorT it_m_1 = oct_a_.end (1);
  IteratorT it_m_2 = oct_a_.end (2);
  IteratorT it_m_b_1 = oct_b_.end (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, LeafBegin)
{
  // Useful types
  typedef typename OctreeT::LeafNodeIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_begin ();
  IteratorT it_a_2 = oct_a_.leaf_begin ();
  IteratorT it_b = oct_b_.leaf_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_begin ();
  IteratorT it_m_1 = oct_a_.leaf_begin (1);
  IteratorT it_m_2 = oct_a_.leaf_begin (2);
  IteratorT it_m_b_1 = oct_b_.leaf_begin (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreeBaseBeginEndIteratorsTest, LeafEnd)
{
  // Useful types
  typedef typename OctreeT::LeafNodeIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_end ();
  IteratorT it_a_2 = oct_a_.leaf_end ();
  IteratorT it_b = oct_b_.leaf_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_end ();
  IteratorT it_m_1 = oct_a_.leaf_end (1);
  IteratorT it_m_2 = oct_a_.leaf_end (2);
  IteratorT it_m_b_1 = oct_b_.leaf_end (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
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

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.depth_end ();
  IteratorT it_m_1 = oct_a_.depth_end (1);
  IteratorT it_m_2 = oct_a_.depth_end (2);
  IteratorT it_m_b_1 = oct_b_.depth_end (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
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

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.breadth_end ();
  IteratorT it_m_1 = oct_a_.breadth_end (1);
  IteratorT it_m_2 = oct_a_.breadth_end (2);
  IteratorT it_m_b_1 = oct_b_.breadth_end (1);

  EXPECT_NE (it_m_1, it_m_2);
  EXPECT_EQ (it_m_2, it_m); // tree depth is 2
  EXPECT_NE (it_m_1, it_m_b_1);
}

////////////////////////////////////////////////////////
//     OctreeBase End Begin/End Iterator Construction
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

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, DepthBegin)
{
  // Useful types
  typedef typename OctreeT::Iterator IteratorT;

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
  typedef typename OctreeT::Iterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.depth_end ();
  IteratorT it_a_2 = oct_a_.depth_end ();
  IteratorT it_b = oct_b_.depth_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.depth_end ();
  IteratorT it_m_1 = oct_a_.depth_end (1);
  IteratorT it_m_md = oct_a_.depth_end (oct_a_.getTreeDepth ());
  IteratorT it_m_b_1 = oct_b_.depth_end (1);

  EXPECT_NE (it_m_1, it_m_md);
  EXPECT_EQ (it_m_md, it_m); // should default to tree depth
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, LeafBegin)
{
  // Useful types
  typedef typename OctreeT::LeafNodeIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_begin ();
  IteratorT it_a_2 = oct_a_.leaf_begin ();
  IteratorT it_b = oct_b_.leaf_begin ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_begin ();
  IteratorT it_m_1 = oct_a_.leaf_begin (1);
  IteratorT it_m_md = oct_a_.leaf_begin (oct_a_.getTreeDepth ());
  IteratorT it_m_b_1 = oct_b_.leaf_begin (1);

  EXPECT_NE (it_m_1, it_m_md);
  EXPECT_EQ (it_m_md, it_m); // should default to tree depth
  EXPECT_NE (it_m_1, it_m_b_1);
}

TEST_F (OctreePointCloudAdjacencyBeginEndIteratorsTest, LeafEnd)
{
  // Useful types
  typedef typename OctreeT::LeafNodeIterator IteratorT;

  // Default initialization
  IteratorT it_a_1 = oct_a_.leaf_end ();
  IteratorT it_a_2 = oct_a_.leaf_end ();
  IteratorT it_b = oct_b_.leaf_end ();

  EXPECT_EQ (it_a_1, it_a_2);
  EXPECT_NE (it_a_1, it_b);
  EXPECT_NE (it_a_2, it_b);

  // Different max depths are not the same iterators
  IteratorT it_m = oct_a_.leaf_end ();
  IteratorT it_m_1 = oct_a_.leaf_end (1);
  IteratorT it_m_md = oct_a_.leaf_end (oct_a_.getTreeDepth ());
  IteratorT it_m_b_1 = oct_b_.leaf_end (1);

  EXPECT_NE (it_m_1, it_m_md);
  EXPECT_EQ (it_m_md, it_m); // should default to tree depth
  EXPECT_NE (it_m_1, it_m_b_1);
}

int
main (int argc, char** const argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
