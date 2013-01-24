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

#include <pcl/geometry/polygon_mesh.h>

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

typedef pcl::geometry::PolygonMesh <MeshTraits <true > > ManifoldPolygonMesh;
typedef pcl::geometry::PolygonMesh <MeshTraits <false> > NonManifoldPolygonMesh;

typedef testing::Types <ManifoldPolygonMesh, NonManifoldPolygonMesh> PolygonMeshTypes;

template <class MeshT>
class TestPolygonMesh : public testing::Test
{
  protected:
    typedef MeshT Mesh;
};

TYPED_TEST_CASE (TestPolygonMesh, PolygonMeshTypes);

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestPolygonMesh, CorrectMeshTag)
{
  typedef typename TestFixture::Mesh Mesh;
  typedef typename Mesh::MeshTag     MeshTag;

  ASSERT_EQ (typeid (pcl::geometry::PolygonMeshTag), typeid (MeshTag));
}

////////////////////////////////////////////////////////////////////////////////

// NOTE: It is the responsibility of the user to ensure that all vertex indices are valid.

//TYPED_TEST (TestPolygonMesh, OutOfRange)
//{
//  typedef typename TestFixture::Mesh Mesh;

//  Mesh mesh;
//  VertexIndices vi;
//  for (unsigned int i=0; i<3; ++i)
//  {
//    vi.push_back (VertexIndex (i));
//  }
//  EXPECT_FALSE (mesh.addFace (vi).isValid ());

//  mesh.addVertex (0);
//  EXPECT_FALSE (mesh.addFace (vi).isValid ());

//  mesh.addVertex (1);
//  EXPECT_FALSE (mesh.addFace (vi).isValid ());

//  mesh.addVertex (2);
//  EXPECT_TRUE (mesh.addFace (vi).isValid ());
//}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestPolygonMesh, CorrectNumberOfVertices)
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
    if (n>=3) EXPECT_TRUE  (index.isValid ()) << "Number of vertices in the face: " << n;
    else      EXPECT_FALSE (index.isValid ()) << "Number of vertices in the face: " << n;
  }
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestPolygonMesh, ThreePolygons)
{
  typedef typename TestFixture::Mesh Mesh;

  //   1 - 6   //
  //  / \   \  //
  // 2 - 0   5 //
  //  \   \ /  //
  //   3 - 4   //
  Mesh mesh;
  for (unsigned int i=0; i<7; ++i) mesh.addVertex (i);

  std::vector <VertexIndices> faces;
  VertexIndices vi;
  vi.push_back (VertexIndex (0));
  vi.push_back (VertexIndex (1));
  vi.push_back (VertexIndex (2));
  faces.push_back (vi);
  vi.clear ();

  vi.push_back (VertexIndex (0));
  vi.push_back (VertexIndex (2));
  vi.push_back (VertexIndex (3));
  vi.push_back (VertexIndex (4));
  faces.push_back (vi);
  vi.clear ();

  vi.push_back (VertexIndex (0));
  vi.push_back (VertexIndex (4));
  vi.push_back (VertexIndex (5));
  vi.push_back (VertexIndex (6));
  vi.push_back (VertexIndex (1));
  faces.push_back (vi);
  vi.clear ();

  for (unsigned int i=0; i<faces.size (); ++i)
  {
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ());
  }

  ASSERT_TRUE (hasFaces (mesh, faces));

  mesh.deleteFace (FaceIndex (1));
  mesh.cleanUp ();

  std::vector <std::vector <int> > expected;
  std::vector <int> tmp;
  tmp.push_back (0);
  tmp.push_back (1);
  tmp.push_back (2);
  expected.push_back (tmp);
  tmp.clear ();

  tmp.push_back (0);
  tmp.push_back (4);
  tmp.push_back (5);
  tmp.push_back (6);
  tmp.push_back (1);
  expected.push_back (tmp);
  tmp.clear ();

  ASSERT_TRUE (hasFaces (mesh, expected));
  std::vector <int> expected_boundary;
  expected_boundary.push_back (2);
  expected_boundary.push_back (1);
  expected_boundary.push_back (6);
  expected_boundary.push_back (5);
  expected_boundary.push_back (4);
  expected_boundary.push_back (0);
  ASSERT_EQ (expected_boundary, getBoundaryVertices (mesh, 0));
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
