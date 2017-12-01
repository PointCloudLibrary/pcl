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
#include <pcl/octree/octree_base.h>
#include <pcl/octree/octree_iterator.h>
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

typedef testing::Types<OctreeDepthFirstIterator<OctreeBase<int> >,
                       OctreeBreadthFirstIterator<OctreeBase<int> > > OctreeIteratorTypes;
TYPED_TEST_CASE(OctreeIteratorTest, OctreeIteratorTypes);

TYPED_TEST (OctreeIteratorTest, CopyConstructor)
{
  TypeParam it_a;
  TypeParam it_b (this->it_); // copy ctor

  EXPECT_NE (it_a, it_b);
  EXPECT_EQ (this->it_, it_b);
  EXPECT_EQ (*this->it_, *it_b);
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

  it_c = it_a; //Our copy assignment
  EXPECT_EQ (it_a, it_c);
  EXPECT_NE (it_b, it_c);
}

int
main (int argc, char** const argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
