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
#include <typeinfo>

#include <gtest/gtest.h>

#include <pcl/geometry/quad_mesh.h>

#include "test_mesh_common_functions.h"

////////////////////////////////////////////////////////////////////////////////

typedef pcl::geometry::VertexIndex   VertexIndex;
typedef pcl::geometry::HalfEdgeIndex HalfEdgeIndex;
typedef pcl::geometry::EdgeIndex     EdgeIndex;
typedef pcl::geometry::FaceIndex     FaceIndex;

typedef std::vector <VertexIndex>   VertexIndices;
typedef std::vector <HalfEdgeIndex> HalfEdgeIndices;
typedef std::vector <FaceIndex>     FaceIndices;

template <bool IsManifoldT>
struct MeshTraits
{
    typedef int                                          VertexData;
    typedef pcl::geometry::NoData                        HalfEdgeData;
    typedef pcl::geometry::NoData                        EdgeData;
    typedef pcl::geometry::NoData                        FaceData;
    typedef boost::integral_constant <bool, IsManifoldT> IsManifold;
};

typedef pcl::geometry::QuadMesh <MeshTraits <true > > ManifoldQuadMesh;
typedef pcl::geometry::QuadMesh <MeshTraits <false> > NonManifoldQuadMesh;

typedef testing::Types <ManifoldQuadMesh, NonManifoldQuadMesh> QuadMeshTypes;

template <class MeshT>
class TestQuadMesh : public testing::Test
{
  protected:
    typedef MeshT Mesh;
};

TYPED_TEST_CASE (TestQuadMesh, QuadMeshTypes);

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestQuadMesh, CorrectMeshTag)
{
  typedef typename TestFixture::Mesh Mesh;
  typedef typename Mesh::MeshTag     MeshTag;

  ASSERT_EQ (typeid (pcl::geometry::QuadMeshTag), typeid (MeshTag));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestQuadMesh, CorrectNumberOfVertices)
{
  // Make sure that only quads can be added
  typedef typename TestFixture::Mesh Mesh;

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
    if (n==4) EXPECT_TRUE  (index.isValid ()) << "Number of vertices in the face: " << n;
    else      EXPECT_FALSE (index.isValid ()) << "Number of vertices in the face: " << n;
  }
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestQuadMesh, OneQuad)
{
  typedef typename TestFixture::Mesh Mesh;

  // 3 - 2 //
  // |   | //
  // 0 - 1 //
  Mesh mesh;
  VertexIndices vi;
  for (unsigned int i=0; i<4; ++i) vi.push_back (mesh.addVertex (i));

  const FaceIndex fi = mesh.addFace (vi);
  ASSERT_TRUE (fi.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi [1]);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi [2]);
  const HalfEdgeIndex he_32 = mesh.getOutgoingHalfEdgeIndex (vi [3]);
  const HalfEdgeIndex he_03 = mesh.getOutgoingHalfEdgeIndex (vi [0]);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_23 = mesh.getOppositeHalfEdgeIndex (he_32);
  const HalfEdgeIndex he_30 = mesh.getOppositeHalfEdgeIndex (he_03);

  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi [1], vi[0]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi [2], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_32, vi [3], vi[2]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_03, vi [0], vi[3]));

  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi [0], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi [1], vi[2]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_23, vi [2], vi[3]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_30, vi [3], vi[0]));

  EXPECT_EQ (he_01, mesh.getIncomingHalfEdgeIndex (vi [1]));
  EXPECT_EQ (he_12, mesh.getIncomingHalfEdgeIndex (vi [2]));
  EXPECT_EQ (he_23, mesh.getIncomingHalfEdgeIndex (vi [3]));
  EXPECT_EQ (he_30, mesh.getIncomingHalfEdgeIndex (vi [0]));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_23, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_30, mesh.getNextHalfEdgeIndex (he_23));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_30));

  EXPECT_EQ (he_30, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_23));
  EXPECT_EQ (he_23, mesh.getPrevHalfEdgeIndex (he_30));

  EXPECT_EQ (he_03, mesh.getNextHalfEdgeIndex (he_10));
  EXPECT_EQ (he_32, mesh.getNextHalfEdgeIndex (he_03));
  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_32));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_21));

  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_10));
  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_03));
  EXPECT_EQ (he_03, mesh.getPrevHalfEdgeIndex (he_32));
  EXPECT_EQ (he_32, mesh.getPrevHalfEdgeIndex (he_21));

  EXPECT_EQ (fi, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_23));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_30));

  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_01).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_23).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_30).isValid ());

  EXPECT_FALSE (mesh.getFaceIndex (he_10).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_32).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_03).isValid ());

  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_10));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_32));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_03));

  EXPECT_EQ (he_30, mesh.getInnerHalfEdgeIndex (fi));
  EXPECT_EQ (he_03, mesh.getOuterHalfEdgeIndex (fi));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_23));
  EXPECT_FALSE (mesh.isBoundary (he_30));

  EXPECT_TRUE (mesh.isBoundary (he_10));
  EXPECT_TRUE (mesh.isBoundary (he_21));
  EXPECT_TRUE (mesh.isBoundary (he_32));
  EXPECT_TRUE (mesh.isBoundary (he_03));

  EXPECT_TRUE (mesh.isBoundary (vi [0]));
  EXPECT_TRUE (mesh.isBoundary (vi [1]));
  EXPECT_TRUE (mesh.isBoundary (vi [2]));
  EXPECT_TRUE (mesh.isBoundary (vi [3]));

  EXPECT_TRUE (mesh.isBoundary (fi));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestQuadMesh, NineQuads)
{
  typedef typename TestFixture::Mesh Mesh;
  const int int_max = std::numeric_limits <int>::max ();

  // Order
  //    -   -   -   //
  //  | 0 | 1 | 2 | //
  //    -   -   -   //
  //  | 3 | 4 | 5 | //
  //    -   -   -   //
  //  | 6 | 7 | 8 | //
  //    -   -   -   //

  // Add the configuration in different orders. Some of them create non-manifold states.
  std::vector <std::vector <int> > order_vec;
  std::vector <int> non_manifold; // When is the first non-manifold quad added?
  std::vector <int> order_tmp;

  // Configuration 0
  order_tmp.push_back (0);
  order_tmp.push_back (1);
  order_tmp.push_back (2);
  order_tmp.push_back (3);
  order_tmp.push_back (5);
  order_tmp.push_back (4);
  order_tmp.push_back (7);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (int_max);
  order_tmp.clear ();

  // Configuration 1
  order_tmp.push_back (0);
  order_tmp.push_back (1);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_tmp.push_back (2);
  order_tmp.push_back (3);
  order_tmp.push_back (7);
  order_tmp.push_back (5);
  order_tmp.push_back (4);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (int_max);
  order_tmp.clear ();

  // Configuration 2
  order_tmp.push_back (0);
  order_tmp.push_back (1);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_tmp.push_back (5);
  order_tmp.push_back (2);
  order_tmp.push_back (3);
  order_tmp.push_back (7);
  order_tmp.push_back (4);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (4);
  order_tmp.clear ();

  // Configuration 3
  order_tmp.push_back (1);
  order_tmp.push_back (3);
  order_tmp.push_back (5);
  order_tmp.push_back (7);
  order_tmp.push_back (4);
  order_tmp.push_back (0);
  order_tmp.push_back (2);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (1);
  order_tmp.clear ();

  // Configuration 4
  order_tmp.push_back (1);
  order_tmp.push_back (3);
  order_tmp.push_back (5);
  order_tmp.push_back (7);
  order_tmp.push_back (0);
  order_tmp.push_back (2);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_tmp.push_back (4);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (1);
  order_tmp.clear ();

  // Configuration 5
  order_tmp.push_back (0);
  order_tmp.push_back (4);
  order_tmp.push_back (8);
  order_tmp.push_back (2);
  order_tmp.push_back (6);
  order_tmp.push_back (1);
  order_tmp.push_back (7);
  order_tmp.push_back (5);
  order_tmp.push_back (3);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (1);
  order_tmp.clear ();

  // 00 - 01 - 02 - 03 //
  //  |    |    |    | //
  // 04 - 05 - 06 - 07 //
  //  |    |    |    | //
  // 08 - 09 - 10 - 11 //
  //  |    |    |    | //
  // 12 - 13 - 14 - 15 //
  typedef VertexIndex VI;
  std::vector <VertexIndices> faces;
  VertexIndices vi;
  vi.push_back (VI ( 0)); vi.push_back (VI ( 4)); vi.push_back (VI ( 5)); vi.push_back (VI ( 1)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 1)); vi.push_back (VI ( 5)); vi.push_back (VI ( 6)); vi.push_back (VI ( 2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 2)); vi.push_back (VI ( 6)); vi.push_back (VI ( 7)); vi.push_back (VI ( 3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 4)); vi.push_back (VI ( 8)); vi.push_back (VI ( 9)); vi.push_back (VI ( 5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 5)); vi.push_back (VI ( 9)); vi.push_back (VI (10)); vi.push_back (VI ( 6)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 6)); vi.push_back (VI (10)); vi.push_back (VI (11)); vi.push_back (VI ( 7)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 8)); vi.push_back (VI (12)); vi.push_back (VI (13)); vi.push_back (VI ( 9)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 9)); vi.push_back (VI (13)); vi.push_back (VI (14)); vi.push_back (VI (10)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (10)); vi.push_back (VI (14)); vi.push_back (VI (15)); vi.push_back (VI (11)); faces.push_back (vi); vi.clear ();

  ASSERT_EQ (order_vec.size (), non_manifold.size ());
  ASSERT_EQ (9, faces.size ());
  for (unsigned int i=0; i<order_vec.size (); ++i)
  {
    std::stringstream ss;
    ss << "Configuration " << i;
    SCOPED_TRACE (ss.str ());

    const std::vector <int> order = order_vec [i];

    EXPECT_EQ (9, order.size ()); // No assert so the other cases can run as well
    if (9 != order.size ()) continue;

    Mesh mesh;
    for (unsigned int j=0; j<16; ++j) mesh.addVertex (j);

    std::vector <VertexIndices> ordered_faces;

    for (unsigned int j=0; j<faces.size (); ++j)
    {
      ordered_faces.push_back (faces [order [j]]);
    }
    bool check_has_faces = true;
    for (unsigned int j=0; j<faces.size (); ++j)
    {
      const FaceIndex index = mesh.addFace (ordered_faces [j]);

      if (j < non_manifold [i] || !Mesh::IsManifold::value)
      {
        EXPECT_TRUE (index.isValid ());
      }
      else
      {
        EXPECT_FALSE (index.isValid ());
        check_has_faces = false;
        break;
      }
    }
    if (check_has_faces)
    {
      EXPECT_TRUE (hasFaces (mesh, ordered_faces));
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
