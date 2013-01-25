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
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "test_mesh_common_functions.h"

////////////////////////////////////////////////////////////////////////////////

TEST (TestMeshConversion, HalfEdgeMeshToFaceVertexMesh)
{
  typedef pcl::geometry::DefaultMeshTraits <pcl::PointXYZRGBNormal> MeshTraits;
  typedef pcl::geometry::PolygonMesh <MeshTraits>                   Mesh;

  // 2 - 1  7 - 6         17 - 16    //
  //  \ /   |   |        /       \   //
  //   0    8 - 5 - 11  12       15  //
  //  / \       |    |   \       /   //
  // 3 - 4      9 - 10    13 - 14    //

  Mesh half_edge_mesh;
  Mesh::VertexIndices vi, hexagon;
  pcl::PointXYZRGBNormal pt;
  pcl::PointCloud <pcl::PointXYZRGBNormal> expected_cloud;
  for (unsigned int i=0; i<18; ++i)
  {
    pt.x = static_cast <float> (10 * i);
    pt.y = static_cast <float> (20 * i);
    pt.z = static_cast <float> (30 * i);

    pt.normal_x = static_cast <float> (100 * i);
    pt.normal_y = static_cast <float> (200 * i);
    pt.normal_z = static_cast <float> (300 * i);

    pt.r = static_cast <uint8_t> (1 * i);
    pt.g = static_cast <uint8_t> (2 * i);
    pt.b = static_cast <uint8_t> (3 * i);

    expected_cloud.push_back (pt);
    vi.push_back (half_edge_mesh.addVertex (pt));
  }

  ASSERT_TRUE (half_edge_mesh.addFace (vi [0], vi [1], vi [ 2]).isValid ());
  ASSERT_TRUE (half_edge_mesh.addFace (vi [0], vi [3], vi [ 4]).isValid ());

  ASSERT_TRUE (half_edge_mesh.addFace (vi [5], vi [6], vi [ 7], vi [ 8]).isValid ());
  ASSERT_TRUE (half_edge_mesh.addFace (vi [5], vi [9], vi [10], vi [11]).isValid ());

  hexagon.push_back (vi [12]);
  hexagon.push_back (vi [13]);
  hexagon.push_back (vi [14]);
  hexagon.push_back (vi [15]);
  hexagon.push_back (vi [16]);
  hexagon.push_back (vi [17]);
  ASSERT_TRUE (half_edge_mesh.addFace (hexagon).isValid ());

  // The conversion
  pcl::geometry::MeshConversion conv;
  pcl::PolygonMesh face_vertex_mesh;
  conv.toFaceVertexMesh (half_edge_mesh, face_vertex_mesh);

  // Check if the cloud got copied correctly.
  pcl::PointCloud <pcl::PointXYZRGBNormal> converted_cloud;
  pcl::fromROSMsg (face_vertex_mesh.cloud, converted_cloud);
  ASSERT_EQ (expected_cloud.size (), converted_cloud.size ());
  for (unsigned int i=0; i<expected_cloud.size (); ++i)
  {
    const pcl::PointXYZRGBNormal& expected_pt  = expected_cloud  [i];
    const pcl::PointXYZRGBNormal& converted_pt = converted_cloud [i];

    EXPECT_FLOAT_EQ (expected_pt.x, converted_pt.x);
    EXPECT_FLOAT_EQ (expected_pt.y, converted_pt.y);
    EXPECT_FLOAT_EQ (expected_pt.z, converted_pt.z);

    EXPECT_FLOAT_EQ (expected_pt.normal_x, converted_pt.normal_x);
    EXPECT_FLOAT_EQ (expected_pt.normal_y, converted_pt.normal_y);
    EXPECT_FLOAT_EQ (expected_pt.normal_z, converted_pt.normal_z);

    EXPECT_EQ (expected_pt.r, converted_pt.r);
    EXPECT_EQ (expected_pt.g, converted_pt.g);
    EXPECT_EQ (expected_pt.b, converted_pt.b);
  }

  // Check the polygons
  ASSERT_EQ (5, face_vertex_mesh.polygons.size ());
  ASSERT_EQ (3, face_vertex_mesh.polygons [0].vertices.size ());
  ASSERT_EQ (3, face_vertex_mesh.polygons [1].vertices.size ());
  ASSERT_EQ (4, face_vertex_mesh.polygons [2].vertices.size ());
  ASSERT_EQ (4, face_vertex_mesh.polygons [3].vertices.size ());
  ASSERT_EQ (6, face_vertex_mesh.polygons [4].vertices.size ());

  std::vector <uint32_t> actual, expected;

  actual = face_vertex_mesh.polygons [0].vertices;
  expected.push_back (0);
  expected.push_back (1);
  expected.push_back (2);
  EXPECT_TRUE (isCircularPermutation (expected, actual));

  actual = face_vertex_mesh.polygons [1].vertices;
  expected.clear ();
  expected.push_back (0);
  expected.push_back (3);
  expected.push_back (4);
  EXPECT_TRUE (isCircularPermutation (expected, actual));

  actual = face_vertex_mesh.polygons [2].vertices;
  expected.clear ();
  expected.push_back (5);
  expected.push_back (6);
  expected.push_back (7);
  expected.push_back (8);
  EXPECT_TRUE (isCircularPermutation (expected, actual));

  actual = face_vertex_mesh.polygons [3].vertices;
  expected.clear ();
  expected.push_back ( 5);
  expected.push_back ( 9);
  expected.push_back (10);
  expected.push_back (11);
  EXPECT_TRUE (isCircularPermutation (expected, actual));

  actual = face_vertex_mesh.polygons [4].vertices;
  expected.clear ();
  expected.push_back (12);
  expected.push_back (13);
  expected.push_back (14);
  expected.push_back (15);
  expected.push_back (16);
  expected.push_back (17);
  EXPECT_TRUE (isCircularPermutation (expected, actual));
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
