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

#include <vector>

#include <pcl/test/gtest.h>

#include <pcl/geometry/triangle_mesh.h>

#include "test_mesh_common_functions.h"

////////////////////////////////////////////////////////////////////////////////

using VertexIndex = pcl::geometry::VertexIndex;
using HalfEdgeIndex = pcl::geometry::HalfEdgeIndex;
using EdgeIndex = pcl::geometry::EdgeIndex;
using FaceIndex = pcl::geometry::FaceIndex;

using VertexIndices = std::vector<VertexIndex>;
using HalfEdgeIndices = std::vector<HalfEdgeIndex>;
using FaceIndices = std::vector<FaceIndex>;

template <bool IsManifoldT>
struct MeshTraits
{
    using VertexData = int;
    using HalfEdgeData = pcl::geometry::NoData;
    using EdgeData = pcl::geometry::NoData;
    using FaceData = pcl::geometry::NoData;
    using IsManifold = std::integral_constant <bool, IsManifoldT>;
};

using ManifoldTriangleMesh = pcl::geometry::TriangleMesh<MeshTraits<true> >;
using NonManifoldTriangleMesh = pcl::geometry::TriangleMesh<MeshTraits<false> >;

using TriangleMeshTypes = testing::Types <ManifoldTriangleMesh, NonManifoldTriangleMesh>;

template <class MeshT>
class TestTriangleMesh : public testing::Test
{
  protected:
    using Mesh = MeshT;
};

TYPED_TEST_SUITE (TestTriangleMesh, TriangleMeshTypes);

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, CorrectMeshTag)
{
  using Mesh = typename TestFixture::Mesh;
  using MeshTag = typename Mesh::MeshTag;

  ASSERT_EQ (typeid (pcl::geometry::TriangleMeshTag), typeid (MeshTag));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, CorrectNumberOfVertices)
{
  // Make sure that only quads can be added
  using Mesh = typename TestFixture::Mesh;

  for (unsigned int n=1; n<=5; ++n)
  {
    Mesh mesh;
    VertexIndices vi;
    for (unsigned int i=0; i<n; ++i)
    {
      vi.push_back (VertexIndex (i));
      mesh.addVertex (i);
    }

    const FaceIndex index = mesh.addFace (vi);
    if (n==3) EXPECT_TRUE  (index.isValid ()) << "Number of vertices in the face: " << n;
    else      EXPECT_FALSE (index.isValid ()) << "Number of vertices in the face: " << n;
  }
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, OneTriangle)
{
  using Mesh = typename TestFixture::Mesh;

  //   2   //
  //  / \  //
  // 0 - 1 //
  Mesh mesh;
  VertexIndices vi;
  for (unsigned int i=0; i<3; ++i) vi.push_back (mesh.addVertex (i));

  const FaceIndex fi = mesh.addFace (vi);
  ASSERT_TRUE (fi.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi [1]);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi [2]);
  const HalfEdgeIndex he_02 = mesh.getOutgoingHalfEdgeIndex (vi [0]);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_20 = mesh.getOppositeHalfEdgeIndex (he_02);

  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi [1], vi[0]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi [2], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_02, vi [0], vi[2]));

  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi [0], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi [1], vi[2]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_20, vi [2], vi[0]));

  EXPECT_EQ (he_01, mesh.getIncomingHalfEdgeIndex (vi [1]));
  EXPECT_EQ (he_12, mesh.getIncomingHalfEdgeIndex (vi [2]));
  EXPECT_EQ (he_20, mesh.getIncomingHalfEdgeIndex (vi [0]));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_20, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_20));

  EXPECT_EQ (he_20, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_20));

  EXPECT_EQ (he_02, mesh.getNextHalfEdgeIndex (he_10));
  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_02));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_21));

  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_10));
  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_02));
  EXPECT_EQ (he_02, mesh.getPrevHalfEdgeIndex (he_21));

  EXPECT_EQ (fi, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_20));

  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_01).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_20).isValid ());

  EXPECT_FALSE (mesh.getFaceIndex (he_10).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_02).isValid ());

  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_10));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_02));

  EXPECT_EQ (he_20, mesh.getInnerHalfEdgeIndex (fi));
  EXPECT_EQ (he_02, mesh.getOuterHalfEdgeIndex (fi));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_20));

  EXPECT_TRUE (mesh.isBoundary (he_10));
  EXPECT_TRUE (mesh.isBoundary (he_21));
  EXPECT_TRUE (mesh.isBoundary (he_02));

  EXPECT_TRUE (mesh.isBoundary (vi [0]));
  EXPECT_TRUE (mesh.isBoundary (vi [1]));
  EXPECT_TRUE (mesh.isBoundary (vi [2]));

  EXPECT_TRUE (mesh.isBoundary (fi));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, TwoTriangles)
{
  using Mesh = typename TestFixture::Mesh;

  // 3 - 2   //
  //  \ / \  //
  //   0 - 1 //
  Mesh mesh;
  VertexIndex vi_0 = mesh.addVertex (0);
  VertexIndex vi_1 = mesh.addVertex (1);
  VertexIndex vi_2 = mesh.addVertex (2);
  VertexIndex vi_3 = mesh.addVertex (3);

  // First face
  VertexIndices vi;
  vi.push_back (vi_0);
  vi.push_back (vi_1);
  vi.push_back (vi_2);
  const FaceIndex fi_0 = mesh.addFace (vi);
  ASSERT_TRUE (fi_0.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi_1);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi_2);
  const HalfEdgeIndex he_02 = mesh.getOutgoingHalfEdgeIndex (vi_0);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_20 = mesh.getOppositeHalfEdgeIndex (he_02);

  // Second face
  vi.clear ();
  vi.push_back (vi_0);
  vi.push_back (vi_2);
  vi.push_back (vi_3);
  const FaceIndex fi_1 = mesh.addFace (vi);
  ASSERT_TRUE (fi_1.isValid ());

  const HalfEdgeIndex he_03 = mesh.getOutgoingHalfEdgeIndex (vi_0);
  const HalfEdgeIndex he_32 = mesh.getOutgoingHalfEdgeIndex (vi_3);

  const HalfEdgeIndex he_30 = mesh.getOppositeHalfEdgeIndex (he_03);
  const HalfEdgeIndex he_23 = mesh.getOppositeHalfEdgeIndex (he_32);

  // Tests
  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi_0, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi_1, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_20, vi_2, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_02, vi_0, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_23, vi_2, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_30, vi_3, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_03, vi_0, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_32, vi_3, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi_2, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi_1, vi_0));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_20, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_20));

  EXPECT_EQ (he_20, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_20));

  EXPECT_EQ (he_23, mesh.getNextHalfEdgeIndex (he_02));
  EXPECT_EQ (he_30, mesh.getNextHalfEdgeIndex (he_23));
  EXPECT_EQ (he_02, mesh.getNextHalfEdgeIndex (he_30));

  EXPECT_EQ (he_30, mesh.getPrevHalfEdgeIndex (he_02));
  EXPECT_EQ (he_02, mesh.getPrevHalfEdgeIndex (he_23));
  EXPECT_EQ (he_23, mesh.getPrevHalfEdgeIndex (he_30));

  EXPECT_EQ (he_03, mesh.getNextHalfEdgeIndex (he_10));
  EXPECT_EQ (he_32, mesh.getNextHalfEdgeIndex (he_03));
  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_32));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_21));

  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_10));
  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_03));
  EXPECT_EQ (he_03, mesh.getPrevHalfEdgeIndex (he_32));
  EXPECT_EQ (he_32, mesh.getPrevHalfEdgeIndex (he_21));

  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_20));

  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_02));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_23));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_30));

  EXPECT_FALSE (mesh.getFaceIndex (he_10).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_03).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_32).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());

  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_01).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_23).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_30).isValid ());

  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_10));
  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_03));
  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_32));

  EXPECT_EQ (he_20, mesh.getInnerHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_30, mesh.getInnerHalfEdgeIndex (fi_1));

  EXPECT_EQ (he_02, mesh.getOuterHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_03, mesh.getOuterHalfEdgeIndex (fi_1));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_20));

  EXPECT_FALSE (mesh.isBoundary (he_02));
  EXPECT_FALSE (mesh.isBoundary (he_23));
  EXPECT_FALSE (mesh.isBoundary (he_30));

  EXPECT_TRUE (mesh.isBoundary (he_10));
  EXPECT_TRUE (mesh.isBoundary (he_03));
  EXPECT_TRUE (mesh.isBoundary (he_32));
  EXPECT_TRUE (mesh.isBoundary (he_21));

  EXPECT_TRUE (mesh.isBoundary (vi_0));
  EXPECT_TRUE (mesh.isBoundary (vi_1));
  EXPECT_TRUE (mesh.isBoundary (vi_2));
  EXPECT_TRUE (mesh.isBoundary (vi_3));

  EXPECT_TRUE (mesh.isBoundary (fi_0));
  EXPECT_TRUE (mesh.isBoundary (fi_1));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, ThreeTriangles)
{
  using Mesh = typename TestFixture::Mesh;

  //  1 ----- 3  //
  //  \ \   / /  //
  //   \  0  /   //
  //    \ | /    //
  //      2      //
  Mesh mesh;
  VertexIndex vi_0 = mesh.addVertex (0);
  VertexIndex vi_1 = mesh.addVertex (1);
  VertexIndex vi_2 = mesh.addVertex (2);
  VertexIndex vi_3 = mesh.addVertex (3);

  // First face
  VertexIndices vi;
  vi.push_back (vi_0);
  vi.push_back (vi_1);
  vi.push_back (vi_2);
  const FaceIndex fi_0 = mesh.addFace (vi);
  ASSERT_TRUE (fi_0.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi_1);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi_2);
  const HalfEdgeIndex he_02 = mesh.getOutgoingHalfEdgeIndex (vi_0);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_20 = mesh.getOppositeHalfEdgeIndex (he_02);

  // Second face
  vi.clear ();
  vi.push_back (vi_0);
  vi.push_back (vi_2);
  vi.push_back (vi_3);
  const FaceIndex fi_1 = mesh.addFace (vi);
  ASSERT_TRUE (fi_1.isValid ());

  const HalfEdgeIndex he_03 = mesh.getOutgoingHalfEdgeIndex (vi_0);
  const HalfEdgeIndex he_32 = mesh.getOutgoingHalfEdgeIndex (vi_3);

  const HalfEdgeIndex he_30 = mesh.getOppositeHalfEdgeIndex (he_03);
  const HalfEdgeIndex he_23 = mesh.getOppositeHalfEdgeIndex (he_32);

  // Third face
  vi.clear ();
  vi.push_back (vi_0);
  vi.push_back (vi_3);
  vi.push_back (vi_1);
  const FaceIndex fi_2 = mesh.addFace (vi);
  ASSERT_TRUE (fi_2.isValid ());

  const HalfEdgeIndex he_13 = mesh.getOutgoingHalfEdgeIndex (vi_1);

  const HalfEdgeIndex he_31 = mesh.getOppositeHalfEdgeIndex (he_13);

  // Tests
  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi_0, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi_1, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_20, vi_2, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_02, vi_0, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_23, vi_2, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_30, vi_3, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_03, vi_0, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_31, vi_3, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi_1, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_32, vi_3, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi_2, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_13, vi_1, vi_3));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_20, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_20));

  EXPECT_EQ (he_23, mesh.getNextHalfEdgeIndex (he_02));
  EXPECT_EQ (he_30, mesh.getNextHalfEdgeIndex (he_23));
  EXPECT_EQ (he_02, mesh.getNextHalfEdgeIndex (he_30));

  EXPECT_EQ (he_31, mesh.getNextHalfEdgeIndex (he_03));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_31));
  EXPECT_EQ (he_03, mesh.getNextHalfEdgeIndex (he_10));

  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_32));
  EXPECT_EQ (he_13, mesh.getNextHalfEdgeIndex (he_21));
  EXPECT_EQ (he_32, mesh.getNextHalfEdgeIndex (he_13));

  EXPECT_EQ (he_20, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_20));

  EXPECT_EQ (he_30, mesh.getPrevHalfEdgeIndex (he_02));
  EXPECT_EQ (he_02, mesh.getPrevHalfEdgeIndex (he_23));
  EXPECT_EQ (he_23, mesh.getPrevHalfEdgeIndex (he_30));

  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_03));
  EXPECT_EQ (he_03, mesh.getPrevHalfEdgeIndex (he_31));
  EXPECT_EQ (he_31, mesh.getPrevHalfEdgeIndex (he_10));

  EXPECT_EQ (he_13, mesh.getPrevHalfEdgeIndex (he_32));
  EXPECT_EQ (he_32, mesh.getPrevHalfEdgeIndex (he_21));
  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_13));

  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_20));

  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_02));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_23));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_30));

  EXPECT_EQ (fi_2, mesh.getFaceIndex (he_03));
  EXPECT_EQ (fi_2, mesh.getFaceIndex (he_31));
  EXPECT_EQ (fi_2, mesh.getFaceIndex (he_10));

  EXPECT_FALSE (mesh.getFaceIndex (he_32).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_13).isValid ());

  EXPECT_EQ (fi_2, mesh.getOppositeFaceIndex (he_01));
  EXPECT_FALSE    (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_20));

  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_02));
  EXPECT_FALSE    (mesh.getOppositeFaceIndex (he_23).isValid ());
  EXPECT_EQ (fi_2, mesh.getOppositeFaceIndex (he_30));

  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_03));
  EXPECT_FALSE    (mesh.getOppositeFaceIndex (he_31).isValid ());
  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_10));

  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_32));
  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi_2, mesh.getOppositeFaceIndex (he_13));

  EXPECT_EQ (he_20, mesh.getInnerHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_30, mesh.getInnerHalfEdgeIndex (fi_1));
  EXPECT_EQ (he_10, mesh.getInnerHalfEdgeIndex (fi_2));

  EXPECT_EQ (he_02, mesh.getOuterHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_03, mesh.getOuterHalfEdgeIndex (fi_1));
  EXPECT_EQ (he_01, mesh.getOuterHalfEdgeIndex (fi_2));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_20));

  EXPECT_FALSE (mesh.isBoundary (he_02));
  EXPECT_FALSE (mesh.isBoundary (he_23));
  EXPECT_FALSE (mesh.isBoundary (he_30));

  EXPECT_FALSE (mesh.isBoundary (he_03));
  EXPECT_FALSE (mesh.isBoundary (he_31));
  EXPECT_FALSE (mesh.isBoundary (he_10));

  EXPECT_TRUE  (mesh.isBoundary (he_32));
  EXPECT_TRUE  (mesh.isBoundary (he_21));
  EXPECT_TRUE  (mesh.isBoundary (he_13));

  EXPECT_FALSE (mesh.isBoundary (vi_0));
  EXPECT_TRUE  (mesh.isBoundary (vi_1));
  EXPECT_TRUE  (mesh.isBoundary (vi_2));
  EXPECT_TRUE  (mesh.isBoundary (vi_3));

  EXPECT_TRUE (mesh.isBoundary (fi_0));
  EXPECT_TRUE (mesh.isBoundary (fi_1));
  EXPECT_TRUE (mesh.isBoundary (fi_2));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestManifoldTriangleMesh, addTrianglePair)
{
  ManifoldTriangleMesh mesh;
  VertexIndices vi;
  for (unsigned int i=0; i<16; ++i)
  {
    vi.push_back (mesh.addVertex ());
  }

  // 00 - 01 - 02 - 03 // X means that both connections / and \ are possible.
  //  | X  | X  | X  | //
  // 04 - 05 - 06 - 07 //
  //  | X  | X  | X  | //
  // 08 - 09 - 10 - 11 //
  //  | X  | X  | X  | //
  // 12 - 13 - 14 - 15 //
  VertexIndices tmp;
  std::vector <VertexIndices> faces;
  tmp.push_back (vi [ 0]); tmp.push_back (vi [ 4]); tmp.push_back (vi [ 5]); tmp.push_back (vi [ 1]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 2]); tmp.push_back (vi [ 6]); tmp.push_back (vi [ 7]); tmp.push_back (vi [ 3]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 1]); tmp.push_back (vi [ 5]); tmp.push_back (vi [ 6]); tmp.push_back (vi [ 2]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 8]); tmp.push_back (vi [12]); tmp.push_back (vi [13]); tmp.push_back (vi [ 9]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [10]); tmp.push_back (vi [14]); tmp.push_back (vi [15]); tmp.push_back (vi [11]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 9]); tmp.push_back (vi [13]); tmp.push_back (vi [14]); tmp.push_back (vi [10]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 4]); tmp.push_back (vi [ 8]); tmp.push_back (vi [ 9]); tmp.push_back (vi [ 5]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 6]); tmp.push_back (vi [10]); tmp.push_back (vi [11]); tmp.push_back (vi [ 7]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 5]); tmp.push_back (vi [ 9]); tmp.push_back (vi [10]); tmp.push_back (vi [ 6]); faces.push_back (tmp); tmp.clear ();

  for (const auto &face : faces)
  {
    std::pair <FaceIndex, FaceIndex> fip;
    fip = mesh.addTrianglePair (face);
    ASSERT_TRUE (fip.first.isValid ());
    ASSERT_TRUE (fip.second.isValid ());
  }

  for (std::size_t i=0; i < faces.size (); ++i)
  {
    VertexIndices actual_1, actual_2;

    ManifoldTriangleMesh::VertexAroundFaceCirculator circ     = mesh.getVertexAroundFaceCirculator (FaceIndex (2*i));
    ManifoldTriangleMesh::VertexAroundFaceCirculator circ_end = circ;
    do
    {
      actual_1.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);

    circ     = mesh.getVertexAroundFaceCirculator (FaceIndex (2*i + 1));
    circ_end = circ;
    do
    {
      actual_2.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);

    VertexIndices expected_1, expected_2, expected_3, expected_4;
    tmp = faces [i];
    expected_1.push_back (tmp [0]); expected_1.push_back (tmp [1]); expected_1.push_back (tmp [2]);
    expected_2.push_back (tmp [0]); expected_2.push_back (tmp [2]); expected_2.push_back (tmp [3]);
    expected_3.push_back (tmp [0]); expected_3.push_back (tmp [1]); expected_3.push_back (tmp [3]);
    expected_4.push_back (tmp [1]); expected_4.push_back (tmp [2]); expected_4.push_back (tmp [3]);

    EXPECT_TRUE ((isCircularPermutation (expected_1, actual_1) && isCircularPermutation (expected_2, actual_2)) ||
                 (isCircularPermutation (expected_2, actual_1) && isCircularPermutation (expected_1, actual_2)) ||
                 (isCircularPermutation (expected_3, actual_1) && isCircularPermutation (expected_4, actual_2)) ||
                 (isCircularPermutation (expected_4, actual_1) && isCircularPermutation (expected_3, actual_2)));
  }
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
