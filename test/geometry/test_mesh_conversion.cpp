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

#include <pcl/test/gtest.h>

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "test_mesh_common_functions.h"

#include <type_traits>

using pcl::index_t;
using pcl::Indices;

////////////////////////////////////////////////////////////////////////////////

template <class MeshTraitsT>
class TestMeshConversion : public ::testing::Test
{
  protected:

    using MeshTraits = MeshTraitsT;

    // 2 - 1  7 - 6         17 - 16   //
    //  \ /   |   |        /       \  //
    //   0    8 - 5 - 11  12       15 //
    //  / \       |    |   \       /  //
    // 3 - 4      9 - 10    13 - 14   //
    void
    SetUp () override
    {
      // Vertices
      pcl::PointXYZRGBNormal pt;
      for (unsigned int i=0; i<18; ++i)
      {
        pt.x = static_cast <float> (10 * i);
        pt.y = static_cast <float> (20 * i);
        pt.z = static_cast <float> (30 * i);

        pt.normal_x = static_cast <float> (100 * i);
        pt.normal_y = static_cast <float> (200 * i);
        pt.normal_z = static_cast <float> (300 * i);

        pt.r = static_cast <std::uint8_t> (1 * i);
        pt.g = static_cast <std::uint8_t> (2 * i);
        pt.b = static_cast <std::uint8_t> (3 * i);

        vertices_.push_back (pt);
      }

      // Faces
      Indices face;

      face.push_back (0);
      face.push_back (1);
      face.push_back (2);
      manifold_faces_.push_back (face);
      non_manifold_faces_.push_back (face);

      face.clear ();
      face.push_back (0);
      face.push_back (3);
      face.push_back (4);
      non_manifold_faces_.push_back (face);

      face.clear ();
      face.push_back (5);
      face.push_back (6);
      face.push_back (7);
      face.push_back (8);
      manifold_faces_.push_back (face);
      non_manifold_faces_.push_back (face);

      face.clear ();
      face.push_back ( 5);
      face.push_back ( 9);
      face.push_back (10);
      face.push_back (11);
      non_manifold_faces_.push_back (face);

      face.clear ();
      face.push_back (12);
      face.push_back (13);
      face.push_back (14);
      face.push_back (15);
      face.push_back (16);
      face.push_back (17);
      manifold_faces_.push_back (face);
      non_manifold_faces_.push_back (face);
    }

    pcl::PointCloud <pcl::PointXYZRGBNormal> vertices_;
    std::vector <Indices>    non_manifold_faces_;
    std::vector <Indices>    manifold_faces_;

  public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

template <bool IsManifoldT>
struct MeshTraits
{
    using VertexData = pcl::PointXYZRGBNormal;
    using HalfEdgeData = pcl::geometry::NoData;
    using EdgeData = pcl::geometry::NoData;
    using FaceData = pcl::geometry::NoData;
    using IsManifold = std::integral_constant <bool, IsManifoldT>;
};

using ManifoldMeshTraits = MeshTraits<true>;
using NonManifoldMeshTraits = MeshTraits<false>;

using MeshTraitsTypes = testing::Types <ManifoldMeshTraits, NonManifoldMeshTraits>;

TYPED_TEST_SUITE (TestMeshConversion, MeshTraitsTypes);

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestMeshConversion, HalfEdgeMeshToFaceVertexMesh)
{
  using Traits = typename TestFixture::MeshTraits;
  using Mesh = pcl::geometry::PolygonMesh<Traits>;
  using VertexIndex = typename Mesh::VertexIndex;
  using VertexIndices = typename Mesh::VertexIndices;

  const std::vector <Indices> faces =
      Mesh::IsManifold::value ? this->manifold_faces_ :
                                this->non_manifold_faces_;

  // Generate the mesh
  Mesh half_edge_mesh;
  VertexIndices vi;

  for (std::size_t i=0; i<this->vertices_.size (); ++i)
  {
    half_edge_mesh.addVertex (this->vertices_ [i]);
  }

  for (std::size_t i=0; i<faces.size (); ++i)
  {
    vi.clear ();
    for (const auto &j : faces [i])
    {
      vi.push_back (VertexIndex (static_cast <int> (j)));
    }

    ASSERT_TRUE (half_edge_mesh.addFace (vi).isValid ()) << "Face number " << i;
  }

  // Convert
  pcl::PolygonMesh face_vertex_mesh;
  pcl::geometry::toFaceVertexMesh (half_edge_mesh, face_vertex_mesh);

  // Check if the cloud got copied correctly.
  pcl::PointCloud <pcl::PointXYZRGBNormal> converted_cloud;
  pcl::fromPCLPointCloud2 (face_vertex_mesh.cloud, converted_cloud);
  ASSERT_EQ (this->vertices_.size (), converted_cloud.size ());
  for (std::size_t i=0; i<this->vertices_.size (); ++i)
  {
    const pcl::PointXYZRGBNormal& expected_pt = this->vertices_ [i];
    const pcl::PointXYZRGBNormal& actual_pt   = converted_cloud [i];

    EXPECT_FLOAT_EQ (expected_pt.x, actual_pt.x);
    EXPECT_FLOAT_EQ (expected_pt.y, actual_pt.y);
    EXPECT_FLOAT_EQ (expected_pt.z, actual_pt.z);

    EXPECT_FLOAT_EQ (expected_pt.normal_x, actual_pt.normal_x);
    EXPECT_FLOAT_EQ (expected_pt.normal_y, actual_pt.normal_y);
    EXPECT_FLOAT_EQ (expected_pt.normal_z, actual_pt.normal_z);

    EXPECT_EQ (expected_pt.r, actual_pt.r);
    EXPECT_EQ (expected_pt.g, actual_pt.g);
    EXPECT_EQ (expected_pt.b, actual_pt.b);
  }

  // Check the polygons
  ASSERT_EQ (faces.size (), face_vertex_mesh.polygons.size ());
  for (std::size_t i=0; i<faces.size (); ++i)
  {
    EXPECT_TRUE (isCircularPermutation (faces [i], face_vertex_mesh.polygons [i].vertices)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestMeshConversion, FaceVertexMeshToHalfEdgeMesh)
{
  using Traits = typename TestFixture::MeshTraits;
  using Mesh = pcl::geometry::PolygonMesh<Traits>;
  using FaceIndex = typename Mesh::FaceIndex;
  using VAFC = typename Mesh::VertexAroundFaceCirculator;

  // Generate the mesh
  pcl::PolygonMesh face_vertex_mesh;
  pcl::toPCLPointCloud2 (this->vertices_, face_vertex_mesh.cloud);
  pcl::Vertices face;
  for (std::size_t i=0; i<this->non_manifold_faces_.size (); ++i)
  {
    face.vertices = this->non_manifold_faces_ [i];
    face_vertex_mesh.polygons.push_back (face);
  }

  // Convert
  Mesh half_edge_mesh;

  int n_not_added = pcl::geometry::toHalfEdgeMesh (face_vertex_mesh, half_edge_mesh);
  if (Mesh::IsManifold::value) ASSERT_EQ (2, n_not_added);
  else                         ASSERT_EQ (0, n_not_added);

  // Check if the cloud got copied correctly.
  ASSERT_EQ (this->vertices_.size (), half_edge_mesh.getVertexDataCloud ().size ());
  for (std::size_t i=0; i<this->vertices_.size (); ++i)
  {
    const pcl::PointXYZRGBNormal& expected_pt = this->vertices_ [i];
    const pcl::PointXYZRGBNormal& actual_pt   = half_edge_mesh.getVertexDataCloud () [i];

    EXPECT_FLOAT_EQ (expected_pt.x, actual_pt.x);
    EXPECT_FLOAT_EQ (expected_pt.y, actual_pt.y);
    EXPECT_FLOAT_EQ (expected_pt.z, actual_pt.z);

    EXPECT_FLOAT_EQ (expected_pt.normal_x, actual_pt.normal_x);
    EXPECT_FLOAT_EQ (expected_pt.normal_y, actual_pt.normal_y);
    EXPECT_FLOAT_EQ (expected_pt.normal_z, actual_pt.normal_z);

    EXPECT_EQ (expected_pt.r, actual_pt.r);
    EXPECT_EQ (expected_pt.g, actual_pt.g);
    EXPECT_EQ (expected_pt.b, actual_pt.b);
  }

  // Check the faces
  const std::vector <Indices> expected_faces =
      Mesh::IsManifold::value ? this->manifold_faces_ :
                                this->non_manifold_faces_;

  ASSERT_EQ (expected_faces.size (), half_edge_mesh.sizeFaces ());

  Indices converted_face;
  for (std::size_t i=0; i<half_edge_mesh.sizeFaces (); ++i)
  {
    VAFC       circ     = half_edge_mesh.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    converted_face.clear ();
    do
    {
      converted_face.push_back (static_cast <index_t> (circ.getTargetIndex ().get ()));
    } while (++circ != circ_end);

    EXPECT_TRUE (isCircularPermutation (expected_faces [i], converted_face)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

// This test should not compile (mesh has no vertex data).

//TEST (TestFaceVertexMeshToHalfEdgeMesh, NoVertexData)
//{
//  using MeshTraits = pcl::geometry::DefaultMeshTraits <>;
//  using Mesh = pcl::geometry::PolygonMesh <MeshTraits>;

//  Mesh half_edge_mesh;
//  pcl::PolygonMesh face_vertex_mesh;

//  pcl::geometry::toHalfEdgeMesh (face_vertex_mesh, half_edge_mesh);
//}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestMeshConversion, NonConvertibleCases)
{
  using Traits = typename TestFixture::MeshTraits;
  using TriangleMesh = pcl::geometry::TriangleMesh<Traits>;
  using QuadMesh = pcl::geometry::QuadMesh<Traits>;
  using PolygonMesh = pcl::geometry::PolygonMesh<Traits>;

  // Generate the mesh
  pcl::PolygonMesh face_vertex_mesh;
  pcl::toPCLPointCloud2 (this->vertices_, face_vertex_mesh.cloud);
  pcl::Vertices face;
  for (std::size_t i=0; i<this->non_manifold_faces_.size (); ++i)
  {
    face.vertices = this->non_manifold_faces_ [i];
    face_vertex_mesh.polygons.push_back (face);
  }

  // Convert
  TriangleMesh tm;
  QuadMesh     qm;
  PolygonMesh  pm;

  const int n_not_added_t = pcl::geometry::toHalfEdgeMesh (face_vertex_mesh, tm);
  const int n_not_added_q = pcl::geometry::toHalfEdgeMesh (face_vertex_mesh, qm);
  const int n_not_added_p = pcl::geometry::toHalfEdgeMesh (face_vertex_mesh, pm);

  if (Traits::IsManifold::value)
  {
    ASSERT_EQ (4, n_not_added_t);
    ASSERT_EQ (4, n_not_added_q);
    ASSERT_EQ (2, n_not_added_p);
  }
  else
  {
    ASSERT_EQ (3, n_not_added_t);
    ASSERT_EQ (3, n_not_added_q);
    ASSERT_EQ (0, n_not_added_p);
  }
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
