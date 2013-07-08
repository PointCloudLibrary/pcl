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

#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/get_boundary.h>
#include "test_mesh_common_functions.h"

////////////////////////////////////////////////////////////////////////////////

template <bool IsManifoldT>
struct MeshTraits
{
    typedef pcl::geometry::NoData                        VertexData;
    typedef pcl::geometry::NoData                        HalfEdgeData;
    typedef pcl::geometry::NoData                        EdgeData;
    typedef pcl::geometry::NoData                        FaceData;
    typedef boost::integral_constant <bool, IsManifoldT> IsManifold;
};

typedef pcl::geometry::PolygonMesh <MeshTraits <true > > ManifoldMesh;
typedef pcl::geometry::PolygonMesh <MeshTraits <false> > NonManifoldMesh;

typedef testing::Types <ManifoldMesh, NonManifoldMesh> MeshTypes;

template <class MeshT>
class TestGetBoundary : public testing::Test
{
  protected:
    typedef MeshT Mesh;
};

TYPED_TEST_CASE (TestGetBoundary, MeshTypes);

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestGetBoundary, GetBoundaryHalfEdges)
{
  typedef typename TestFixture::Mesh Mesh;


  typedef typename Mesh::VertexIndices   VertexIndices;
  typedef typename Mesh::HalfEdgeIndices HalfEdgeIndices;

  //  0 -  1 -  2 -  3
  //  |    |    |    |
  //  4 -  5 -  6 -  7
  //  |    |    |    |
  //  8 -  9   10 - 11
  //  |    |    |    |
  // 12 - 13 - 14 - 15
  //  |    |    |    |
  // 16 - 17 - 18 - 19

  Mesh mesh;
  VertexIndices vi;

  for (int i=0; i<23; ++i) vi.push_back (mesh.addVertex ());

  ASSERT_TRUE (mesh.addFace (vi [ 0], vi [ 4], vi [ 5], vi [ 1]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [ 1], vi [ 5], vi [ 6], vi [ 2]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [ 2], vi [ 6], vi [ 7], vi [ 3]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [ 4], vi [ 8], vi [ 9], vi [ 5]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [ 6], vi [10], vi [11], vi [ 7]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [ 8], vi [12], vi [13], vi [ 9]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [10], vi [14], vi [15], vi [11]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [12], vi [16], vi [17], vi [13]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [14], vi [18], vi [19], vi [15]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [13], vi [17], vi [18], vi [14]).isValid ());

  std::vector <HalfEdgeIndices> expected_boundary_collection;
  HalfEdgeIndices               boundary;

  boundary.push_back (findHalfEdge (mesh, vi [ 0], vi [ 1]));
  boundary.push_back (findHalfEdge (mesh, vi [ 1], vi [ 2]));
  boundary.push_back (findHalfEdge (mesh, vi [ 2], vi [ 3]));
  boundary.push_back (findHalfEdge (mesh, vi [ 3], vi [ 7]));
  boundary.push_back (findHalfEdge (mesh, vi [ 7], vi [11]));
  boundary.push_back (findHalfEdge (mesh, vi [11], vi [15]));
  boundary.push_back (findHalfEdge (mesh, vi [15], vi [19]));
  boundary.push_back (findHalfEdge (mesh, vi [19], vi [18]));
  boundary.push_back (findHalfEdge (mesh, vi [18], vi [17]));
  boundary.push_back (findHalfEdge (mesh, vi [17], vi [16]));
  boundary.push_back (findHalfEdge (mesh, vi [16], vi [12]));
  boundary.push_back (findHalfEdge (mesh, vi [12], vi [ 8]));
  boundary.push_back (findHalfEdge (mesh, vi [ 8], vi [ 4]));
  boundary.push_back (findHalfEdge (mesh, vi [ 4], vi [ 0]));
  expected_boundary_collection.push_back (boundary);
  boundary.clear ();

  boundary.push_back (findHalfEdge (mesh, vi [ 5], vi [ 9]));
  boundary.push_back (findHalfEdge (mesh, vi [ 9], vi [13]));
  boundary.push_back (findHalfEdge (mesh, vi [13], vi [14]));
  boundary.push_back (findHalfEdge (mesh, vi [14], vi [10]));
  boundary.push_back (findHalfEdge (mesh, vi [10], vi [ 6]));
  boundary.push_back (findHalfEdge (mesh, vi [ 6], vi [ 5]));
  expected_boundary_collection.push_back (boundary);
  boundary.clear ();

  std::vector <HalfEdgeIndices> actual_boundary_collection;

  pcl::geometry::getBoundBoundaryHalfEdges (mesh, actual_boundary_collection);
  ASSERT_TRUE (isCircularPermutationVec (expected_boundary_collection, actual_boundary_collection));

  if (Mesh::IsManifold::value) return;

  //  0 -  1 -  2 -  3
  //  |    |    |    |
  //  4 -  5 -  6 -  7
  //  |    |    |    |
  //  8 -  9   10 - 11
  //  |    |    |    |
  // 12 - 13 - 14 - 15
  //  |    |    |    |
  // 16 - 17 - 18 - 19 - 20
  //                 |    |
  //                21 - 22
  ASSERT_TRUE (mesh.addFace (vi [ 5], vi [ 9], vi [10], vi [ 6]).isValid ());
  ASSERT_TRUE (mesh.addFace (vi [19], vi [21], vi [22], vi [20]).isValid ());

  expected_boundary_collection.clear ();

  boundary.push_back (findHalfEdge (mesh, vi [ 0], vi [ 1]));
  boundary.push_back (findHalfEdge (mesh, vi [ 1], vi [ 2]));
  boundary.push_back (findHalfEdge (mesh, vi [ 2], vi [ 3]));
  boundary.push_back (findHalfEdge (mesh, vi [ 3], vi [ 7]));
  boundary.push_back (findHalfEdge (mesh, vi [ 7], vi [11]));
  boundary.push_back (findHalfEdge (mesh, vi [11], vi [15]));
  boundary.push_back (findHalfEdge (mesh, vi [15], vi [19]));
  boundary.push_back (findHalfEdge (mesh, vi [19], vi [20]));
  boundary.push_back (findHalfEdge (mesh, vi [20], vi [22]));
  boundary.push_back (findHalfEdge (mesh, vi [22], vi [21]));
  boundary.push_back (findHalfEdge (mesh, vi [21], vi [19]));
  boundary.push_back (findHalfEdge (mesh, vi [19], vi [18]));
  boundary.push_back (findHalfEdge (mesh, vi [18], vi [17]));
  boundary.push_back (findHalfEdge (mesh, vi [17], vi [16]));
  boundary.push_back (findHalfEdge (mesh, vi [16], vi [12]));
  boundary.push_back (findHalfEdge (mesh, vi [12], vi [ 8]));
  boundary.push_back (findHalfEdge (mesh, vi [ 8], vi [ 4]));
  boundary.push_back (findHalfEdge (mesh, vi [ 4], vi [ 0]));
  expected_boundary_collection.push_back (boundary);
  boundary.clear ();

  boundary.push_back (findHalfEdge (mesh, vi [ 9], vi [13]));
  boundary.push_back (findHalfEdge (mesh, vi [13], vi [14]));
  boundary.push_back (findHalfEdge (mesh, vi [14], vi [10]));
  boundary.push_back (findHalfEdge (mesh, vi [10], vi [ 9]));
  expected_boundary_collection.push_back (boundary);
  boundary.clear ();

  pcl::geometry::getBoundBoundaryHalfEdges (mesh, actual_boundary_collection);
  ASSERT_TRUE (isCircularPermutationVec (expected_boundary_collection, actual_boundary_collection));
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
