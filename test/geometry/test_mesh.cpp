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

// Needed for one test. Must be defined before including the mesh.
#define PCL_GEOMETRY_MESH_BASE_TEST_DELETE_FACE_MANIFOLD_2

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

////////////////////////////////////////////////////////////////////////////////

TEST (TestAddDeleteFace, NonManifold1)
{
  // The triangle 0-1-2 is added last //
  //     5                            //
  //    / \                           //
  //   0 - 2                          //
  //  / \ / \                         //
  // 3 - 1 - 4                        //
  NonManifoldTriangleMesh mesh;
  for (unsigned int i=0; i<6; ++i) mesh.addVertex (i);

  using VI = VertexIndex;
  VertexIndices vi;
  std::vector <VertexIndices> faces;
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear (); // 0
  vi.push_back (VI (2)); vi.push_back (VI (1)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear (); // 1
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear (); // 2
  for (const auto &face : faces)
  {
    ASSERT_TRUE (mesh.addFace (face).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Check if the whole boundary is reached.
  VertexIndices boundary_expected;
  boundary_expected.push_back (VI (0));
  boundary_expected.push_back (VI (5));
  boundary_expected.push_back (VI (2));
  boundary_expected.push_back (VI (4));
  boundary_expected.push_back (VI (1));
  boundary_expected.push_back (VI (3));

  VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (3));
  EXPECT_EQ (boundary_expected, boundary_vertices);

  // Close the gaps.
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear (); // 3
  ASSERT_TRUE (mesh.addFace (faces [3]).isValid ());
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Delete faces
  mesh.deleteFace (FaceIndex (3));
  mesh.deleteFace (FaceIndex (1));
  mesh.cleanUp ();

  std::vector <std::vector <int> > expected_data;
  std::vector <int> data;
  data.push_back (0); data.push_back (3); data.push_back (1); expected_data.push_back (data); data.clear ();
  data.push_back (0); data.push_back (2); data.push_back (5); expected_data.push_back (data); data.clear ();

  std::vector <int> boundary_data_expected;
  boundary_data_expected.push_back (3);
  boundary_data_expected.push_back (0);
  boundary_data_expected.push_back (5);
  boundary_data_expected.push_back (2);
  boundary_data_expected.push_back (0);
  boundary_data_expected.push_back (1);

  ASSERT_TRUE (hasFaces (mesh, expected_data));
  ASSERT_EQ (boundary_data_expected, getBoundaryVertices (mesh, 1));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestAddDeleteFace, NonManifold2)
{
  NonManifoldTriangleMesh mesh;
  for (unsigned int i=0; i<9; ++i) mesh.addVertex (i);
  using VI = VertexIndex;
  VertexIndices vi;
  std::vector <VertexIndices> faces;

  // 2 - 1 //
  //  \ /  //
  //   0   //
  //  / \  //
  // 3 - 4 //
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  for (const auto &face : faces)
  {
    ASSERT_TRUE (mesh.addFace (face).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  // (*) Adding the next two faces would destroy the connectivity around vertex 0. E.g. a VertexAroundVertexCirculator would not be able to access all the vertices (1, 2, 3, 4) anymore.
  vi.push_back (VI (3)); vi.push_back (VI (0)); vi.push_back (VI (4));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (1)); vi.push_back (VI (0)); vi.push_back (VI (2));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  {
    // Check if the whole boundary is reached.
    VertexIndices boundary_expected;
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (1));
    boundary_expected.push_back (VI (2));
    boundary_expected.push_back (VI (3));
    boundary_expected.push_back (VI (4));

    VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (2));
    std::sort (boundary_vertices.begin (), boundary_vertices.end ());
    EXPECT_EQ (boundary_expected, boundary_vertices);
  }

  // New config //
  //   2 - 1    //
  //    \ /     //
  // 3 - 0 - 6  //
  //  \ / \ /   //
  //   4   5    //
  vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces.push_back (vi); vi.clear ();
  EXPECT_TRUE (mesh.addFace (faces [2]).isValid ());
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Same as (*)
  vi.push_back (VI (1)); vi.push_back (VI (0)); vi.push_back (VI (2));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (3)); vi.push_back (VI (0)); vi.push_back (VI (4));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (5)); vi.push_back (VI (0)); vi.push_back (VI (6));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  {
    // Check if the whole boundary is reached.
    VertexIndices boundary_expected;
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (1));
    boundary_expected.push_back (VI (2));
    boundary_expected.push_back (VI (3));
    boundary_expected.push_back (VI (4));
    boundary_expected.push_back (VI (5));
    boundary_expected.push_back (VI (6));

    VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (2));
    std::sort (boundary_vertices.begin (), boundary_vertices.end ());
    EXPECT_EQ (boundary_expected, boundary_vertices);
  }

  // New config //
  // 2---1   8  //
  //  \  |  /|  //
  //   \ | / |  //
  //    \|/  |  //
  // 3---0---7  //
  // |  /|\     //
  // | / | \    //
  // |/  |  \   //
  // 4   5---6  //
  vi.push_back (VI (0)); vi.push_back (VI (7)); vi.push_back (VI (8)); faces.push_back (vi); vi.clear ();
  EXPECT_TRUE (mesh.addFace (faces [3]).isValid ());
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Same as (*)
  vi.push_back (VI (1)); vi.push_back (VI (0)); vi.push_back (VI (2));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (3)); vi.push_back (VI (0)); vi.push_back (VI (4));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (5)); vi.push_back (VI (0)); vi.push_back (VI (6));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (7)); vi.push_back (VI (0)); vi.push_back (VI (8));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  // Check if the whole boundary is reached.
  {
    VertexIndices boundary_expected;
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (1));
    boundary_expected.push_back (VI (2));
    boundary_expected.push_back (VI (3));
    boundary_expected.push_back (VI (4));
    boundary_expected.push_back (VI (5));
    boundary_expected.push_back (VI (6));
    boundary_expected.push_back (VI (7));
    boundary_expected.push_back (VI (8));

    VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (2));
    std::sort (boundary_vertices.begin (), boundary_vertices.end ());
    EXPECT_EQ (boundary_expected, boundary_vertices);
  }

  // Close the gaps //
  // 2---1---8      //
  // |\  |  /|      //
  // | \0|5/ |      //
  // | 6\|/3 |      //
  // 3---0---7      //
  // | 1/|\7 |      //
  // | /4|2\ |      //
  // |/  |  \|      //
  // 4---5---6      //
  vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (8)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (7)); faces.push_back (vi); vi.clear ();
  for (std::size_t i = 4; i < faces.size (); ++i)
  {
    EXPECT_TRUE (mesh.addFace (faces [i]).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  VertexIndices boundary_expected;
  for (unsigned int i=8; i>0; --i)
  {
    boundary_expected.push_back (VI (i));
  }
  VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (1));
  EXPECT_EQ (boundary_expected, boundary_vertices);

  // Copy vertex indices to data
  std::vector <std::vector <int> > expected (faces.size ());
  for (std::size_t i = 0; i < faces.size (); ++i)
  {
    std::vector <int> tmp (faces [i].size ());
    for (std::size_t j = 0; j < faces [i].size (); ++j)
    {
      tmp [j] = faces [i][j].get ();
    }
    expected [i] = tmp;
  }

  // Delete all faces
  while (!expected.empty ())
  {
    mesh.deleteFace (FaceIndex (0));
    mesh.cleanUp ();
    expected.erase (expected.begin ());
    ASSERT_TRUE (hasFaces (mesh, expected));
  }

  ASSERT_TRUE (mesh.empty ());
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestAddDeleteFace, Manifold1)
{
  using Mesh = ManifoldTriangleMesh;
  using VI = VertexIndex;
  Mesh mesh;
  for (unsigned int i=0; i<7; ++i) mesh.addVertex (i);

  //   2 - 1   //
  //  / \ / \  //
  // 3 - 0 - 6 //
  //  \ / \ /  //
  //   4 - 5   //
  std::vector <VertexIndices> faces;
  std::vector <std::vector <int> > expected;
  VertexIndices vi;
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();

  for (std::size_t i = 0; i < faces.size (); ++i)
  {
    std::vector <int> tmp (faces [i].size ());
    for (std::size_t j = 0; j < faces [i].size (); ++j)
    {
      tmp [j] = faces [i][j].get ();
    }
    expected.push_back (tmp);
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ()) << "Face " << i;
  }
  ASSERT_TRUE (hasFaces (mesh, faces));
  ASSERT_TRUE (hasFaces (mesh, expected));

  mesh.deleteFace (FaceIndex (0)); // 012
  mesh.deleteFace (FaceIndex (3)); // 045 -> makes mesh non-manifold -> delete additional faces.
  mesh.cleanUp ();

  EXPECT_TRUE (mesh.isManifold ());

  // Only the faces 0-5-6 and 0-6-1 remain after deletion.
  // NOTE: It would be also valid if the faces 0-2-3 and 0-3-4 remain.
  ASSERT_TRUE (hasFaces (mesh, std::vector <std::vector <int> > (expected.begin ()+4, expected.end ())));

  //   9 - 10    //
  //  / \ / \    //
  // 8 - 2 - 1   //
  //  \ / \ / \  //
  //   3 - 0 - 6 //
  //  / \ / \ /  //
  // 7 - 4 - 5   //
  mesh.clear ();
  expected.clear ();
  for (unsigned int i=0; i<11; ++i) mesh.addVertex (i);
  vi.push_back (VI ( 3)); vi.push_back (VI (7)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 3)); vi.push_back (VI (2)); vi.push_back (VI (8)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 8)); vi.push_back (VI (2)); vi.push_back (VI (9)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (10)); vi.push_back (VI (9)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (10)); vi.push_back (VI (2)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();
  for (std::size_t i = 0; i < faces.size (); ++i)
  {
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ()) << "Face " << i;
  }
  ASSERT_TRUE (hasFaces (mesh, faces));

  mesh.deleteFace (FaceIndex (0)); // 0-1-2 -> deletes 10-2-1
  mesh.deleteFace (FaceIndex (3)); // 0-4-5 -> deletes  0-3-4, 3-7-4 and 0-2-3
  mesh.cleanUp ();

  std::vector <int> tmp;
  tmp.push_back ( 0); tmp.push_back (5); tmp.push_back (6); expected.push_back (tmp); tmp.clear ();
  tmp.push_back ( 0); tmp.push_back (6); tmp.push_back (1); expected.push_back (tmp); tmp.clear ();
  tmp.push_back ( 3); tmp.push_back (2); tmp.push_back (8); expected.push_back (tmp); tmp.clear ();
  tmp.push_back ( 8); tmp.push_back (2); tmp.push_back (9); expected.push_back (tmp); tmp.clear ();
  tmp.push_back (10); tmp.push_back (9); tmp.push_back (2); expected.push_back (tmp); tmp.clear ();
  EXPECT_TRUE (mesh.isManifold ());
  ASSERT_TRUE (hasFaces (mesh, expected));

  std::vector <int> expected_boundary;
  expected_boundary.push_back (6);
  expected_boundary.push_back (5);
  expected_boundary.push_back (0);
  expected_boundary.push_back (1);
  EXPECT_EQ (expected_boundary, getBoundaryVertices (mesh, 1));

  expected_boundary.clear ();
  expected_boundary.push_back ( 2);
  expected_boundary.push_back ( 3);
  expected_boundary.push_back ( 8);
  expected_boundary.push_back ( 9);
  expected_boundary.push_back (10);
  EXPECT_EQ (expected_boundary, getBoundaryVertices (mesh, 8));
}

////////////////////////////////////////////////////////////////////////////////

#ifdef PCL_GEOMETRY_MESH_BASE_TEST_DELETE_FACE_MANIFOLD_2

TEST (TestAddDeleteFace, Manifold2)
{
  using Mesh = ManifoldTriangleMesh;
  using VI = VertexIndex;

  //  1 ----- 3  //
  //  \ \   / /  //
  //   \  0  /   //
  //    \ | /    //
  //      2      //
  std::vector <VertexIndices> faces;
  VertexIndices vi;
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();

  // Try all possible combinations of adding the faces and deleting a vertex.
  // NOTE: Some cases are redundant.
  std::vector <int>                p;
  std::vector <std::vector <int> > permutations;

  p.push_back (0); p.push_back (1); p.push_back (2); permutations.push_back (p); p.clear ();
  p.push_back (1); p.push_back (2); p.push_back (0); permutations.push_back (p); p.clear ();
  p.push_back (2); p.push_back (0); p.push_back (1); permutations.push_back (p); p.clear ();

  Mesh mesh_tmp;
  vi.clear ();
  for (int i=0; i<4; ++i)
  {
    vi.push_back (mesh_tmp.addVertex ());
  }

  for (std::size_t i=0; i<permutations.size (); ++i) // first face
  {
    for (std::size_t j=0; j<permutations.size (); ++j) // second face
    {
      for (std::size_t k=0; k<permutations.size (); ++k) // third face
      {
        for (VI l (0); l < VI (3); ++l) // deleted vertex
        {
          pcl::geometry::g_pcl_geometry_mesh_base_test_delete_face_manifold_2_success = true;

          std::stringstream errormsg;
          errormsg << "\n";
          errormsg << faces [0] [permutations [i][0]] << faces [0] [permutations [i][1]] << faces [0] [permutations [i][2]] << " ";
          errormsg << faces [1] [permutations [j][0]] << faces [1] [permutations [j][1]] << faces [1] [permutations [j][2]] << " ";
          errormsg << faces [2] [permutations [k][0]] << faces [2] [permutations [k][1]] << faces [2] [permutations [k][2]] << " ";
          errormsg << "v: " << l << "\n";

          Mesh mesh = mesh_tmp;

          mesh.addFace (faces [0] [permutations [i][0]], faces [0] [permutations [i][1]], faces [0] [permutations [i][2]]);
          mesh.addFace (faces [1] [permutations [j][0]], faces [1] [permutations [j][1]], faces [1] [permutations [j][2]]);
          mesh.addFace (faces [2] [permutations [k][0]], faces [2] [permutations [k][1]], faces [2] [permutations [k][2]]);

          mesh.deleteVertex (VI (l));
          ASSERT_TRUE (pcl::geometry::g_pcl_geometry_mesh_base_test_delete_face_manifold_2_success) << errormsg.str ();
        }
      }
    }
  }
}
#endif // PCL_GEOMETRY_MESH_BASE_TEST_DELETE_FACE_MANIFOLD_2

////////////////////////////////////////////////////////////////////////////////

TEST (TestDelete, VertexAndEdge)
{
  using Mesh = NonManifoldTriangleMesh;
  using VI = VertexIndex;
  Mesh mesh;
  for (unsigned int i=0; i<7; ++i) mesh.addVertex (i);

  //   2 - 1          2 - 1  //
  //  / \ / \        / \ /   //
  // 3 - 0 - 6  ->  3 - 0    //
  //  \ / \ /        \ / \   //
  //   4 - 5          4 - 5  //
  std::vector <VertexIndices> faces;
  std::vector <std::vector <int> > expected;
  VertexIndices vi;
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();

  for (std::size_t i = 0; i < faces.size (); ++i)
  {
    std::vector <int> tmp (faces [i].size ());
    for (std::size_t j = 0; j < faces [i].size (); ++j)
    {
      tmp [j] = faces [i][j].get ();
    }
    expected.push_back (tmp);
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ()) << "Face " << i;
  }
  ASSERT_TRUE (hasFaces (mesh, faces));
  ASSERT_TRUE (hasFaces (mesh, expected));

  // Search edge 0-6
  Mesh::VertexAroundVertexCirculator       circ     = mesh.getVertexAroundVertexCirculator (VI (0));
  const Mesh::VertexAroundVertexCirculator circ_end = circ;
  HalfEdgeIndex idx_he_06;
  do
  {
    if (circ.getTargetIndex () == VI (6))
    {
      idx_he_06 = circ.getCurrentHalfEdgeIndex ();
      break;
    }
  } while (++circ != circ_end);

  // Delete
  Mesh mesh_deleted_vertex    = mesh;
  Mesh mesh_deleted_half_edge = mesh;
  Mesh mesh_deleted_edge      = mesh;
  expected.pop_back ();
  expected.pop_back ();

  mesh_deleted_vertex.deleteVertex (VI (6));
  mesh_deleted_vertex.cleanUp ();

  mesh_deleted_half_edge.deleteEdge (idx_he_06);
  mesh_deleted_half_edge.cleanUp ();

  mesh_deleted_edge.deleteEdge (pcl::geometry::toEdgeIndex (idx_he_06));
  mesh_deleted_edge.cleanUp ();

  EXPECT_TRUE (hasFaces (mesh_deleted_vertex,    expected));
  EXPECT_TRUE (hasFaces (mesh_deleted_half_edge, expected));
  EXPECT_TRUE (hasFaces (mesh_deleted_edge,      expected));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestMesh, IsBoundaryIsManifold)
{
  // The triangle 0-1-2 is added last //
  //     5                            //
  //    / \                           //
  //   0 - 2                          //
  //  / \ / \                         //
  // 3 - 1 - 4                        //
  NonManifoldTriangleMesh mesh;
  for (unsigned int i=0; i<6; ++i) mesh.addVertex (i);

  using VI = VertexIndex;
  VertexIndices vi;
  std::vector <VertexIndices> faces;
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear (); // 0
  vi.push_back (VI (2)); vi.push_back (VI (1)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear (); // 1
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear (); // 2
  for (const auto &face : faces)
  {
    ASSERT_TRUE (mesh.addFace (face).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Check if the whole boundary is reached.
  VertexIndices boundary_expected;
  boundary_expected.push_back (VI (0));
  boundary_expected.push_back (VI (5));
  boundary_expected.push_back (VI (2));
  boundary_expected.push_back (VI (4));
  boundary_expected.push_back (VI (1));
  boundary_expected.push_back (VI (3));

  VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (3));
  EXPECT_EQ (boundary_expected, boundary_vertices);
  EXPECT_FALSE (mesh.isManifold (VI (0)));
  EXPECT_FALSE (mesh.isManifold (VI (1)));
  EXPECT_FALSE (mesh.isManifold (VI (2)));
  EXPECT_TRUE  (mesh.isManifold (VI (3)));
  EXPECT_TRUE  (mesh.isManifold (VI (4)));
  EXPECT_TRUE  (mesh.isManifold (VI (5)));
  ASSERT_FALSE (mesh.isManifold ());

  for (std::size_t i = 0; i < mesh.sizeEdges (); ++i)
  {
    ASSERT_TRUE (mesh.isBoundary (EdgeIndex (i)));
  }

  // Make manifold
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear (); // 3
  ASSERT_TRUE  (mesh.addFace (faces [3]).isValid ());
  EXPECT_TRUE  (hasFaces (mesh, faces));

  EXPECT_TRUE  (mesh.isManifold ());
  EXPECT_TRUE  (mesh.isBoundary (FaceIndex (3)));         // This version checks the vertices
  EXPECT_FALSE (mesh.isBoundary <false> (FaceIndex (3))); // This version checks the edges

  const HalfEdgeIndex idx_he_01 = findHalfEdge (mesh, VI (0), VI (1));
  const HalfEdgeIndex idx_he_12 = findHalfEdge (mesh, VI (1), VI (2));
  const HalfEdgeIndex idx_he_20 = findHalfEdge (mesh, VI (2), VI (0));

  ASSERT_TRUE (idx_he_01.isValid ());
  ASSERT_TRUE (idx_he_12.isValid ());
  ASSERT_TRUE (idx_he_20.isValid ());

  EXPECT_FALSE (mesh.isBoundary (pcl::geometry::toEdgeIndex (idx_he_01)));
  EXPECT_FALSE (mesh.isBoundary (pcl::geometry::toEdgeIndex (idx_he_12)));
  EXPECT_FALSE (mesh.isBoundary (pcl::geometry::toEdgeIndex (idx_he_20)));

  // Isolated vertex
  mesh.addVertex (6);
  ASSERT_TRUE (mesh.isManifold (VI (6)));
  ASSERT_TRUE (mesh.isBoundary (VI (6)));
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
