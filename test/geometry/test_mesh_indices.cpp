/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/geometry/mesh_indices.h>

////////////////////////////////////////////////////////////////////////////////

TEST (TestMesh, MeshIndices)
{
  typedef pcl::geometry::VertexIndex   VertexIndex;
  typedef pcl::geometry::HalfEdgeIndex HalfEdgeIndex;
  typedef pcl::geometry::EdgeIndex     EdgeIndex;
  typedef pcl::geometry::FaceIndex     FaceIndex;

  // The other indices should behave the same because they have a common base.
  VertexIndex vi0, vi1 (-5), vi2 (0), vi3 (5), vi4 (5), vi5 (6);

  EXPECT_FALSE (vi0.isValid ());
  EXPECT_FALSE (vi1.isValid ());
  EXPECT_TRUE  (vi2.isValid ());
  EXPECT_TRUE  (vi3.isValid ());
  EXPECT_TRUE  (vi4.isValid ());
  vi2.invalidate ();
  EXPECT_FALSE (vi2.isValid ());

  EXPECT_LT (vi4, vi5);
  EXPECT_EQ (vi3, vi4);

  EXPECT_EQ (5, vi3.get ());
  vi3.set (2);
  EXPECT_EQ (2, vi3.get ());

  EXPECT_EQ (3, (++vi3).get ());
  EXPECT_EQ (2, (--vi3).get ());
  EXPECT_EQ (9, (vi3 += VertexIndex (7)).get ());
  EXPECT_EQ (2, (vi3 -= VertexIndex (7)).get ());

  // Conversions
  HalfEdgeIndex he0, he1 (-9), he2 (4), he3 (5);
  EdgeIndex     e0 , e1  (-9), e2  (4), e3  (5);

  EXPECT_FALSE (pcl::geometry::toEdgeIndex (he0).isValid ());
  EXPECT_FALSE (pcl::geometry::toEdgeIndex (he1).isValid ());
  EXPECT_EQ (2, pcl::geometry::toEdgeIndex (he2).get ());
  EXPECT_EQ (2, pcl::geometry::toEdgeIndex (he3).get ());

  EXPECT_FALSE  (pcl::geometry::toHalfEdgeIndex (e0).isValid ());
  EXPECT_FALSE  (pcl::geometry::toHalfEdgeIndex (e1).isValid ());
  EXPECT_EQ (8 , pcl::geometry::toHalfEdgeIndex (e2).get ());
  EXPECT_EQ (10, pcl::geometry::toHalfEdgeIndex (e3).get ());
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
