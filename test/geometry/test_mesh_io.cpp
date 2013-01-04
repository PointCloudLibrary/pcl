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

#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/mesh_io.h>

////////////////////////////////////////////////////////////////////////////////

typedef pcl::geometry::DefaultMeshTraits <>      MeshTraits;
typedef pcl::geometry::TriangleMesh <MeshTraits> Mesh;
typedef pcl::geometry::MeshIO <Mesh>             MeshIO;

typedef Mesh::VertexIndex   VertexIndex;
typedef Mesh::HalfEdgeIndex HalfEdgeIndex;
typedef Mesh::FaceIndex     FaceIndex;

////////////////////////////////////////////////////////////////////////////////

TEST (TestMeshIO, WriteAndRead)
{
  Mesh expected_mesh, loaded_mesh;

  Mesh::VertexIndices vi;
  for (unsigned int i=0; i<11; ++i)
  {
    vi.push_back (expected_mesh.addVertex ());
  }

  // No triangles 1-3-4 and 6-9-7!
  // 0 - 1 - 2  //
  //  \ / \ /   //
  //   3 - 4    //
  //    \ /     //
  //     5      //
  //    / \     //
  //   6 - 7    //
  //  / \ / \   //
  // 8 - 9 - 10 //
  ASSERT_TRUE (expected_mesh.addFace (vi [0], vi [3], vi [ 1]).isValid ());
  ASSERT_TRUE (expected_mesh.addFace (vi [1], vi [4], vi [ 2]).isValid ());
  ASSERT_TRUE (expected_mesh.addFace (vi [3], vi [5], vi [ 4]).isValid ());
  ASSERT_TRUE (expected_mesh.addFace (vi [6], vi [8], vi [ 0]).isValid ());
  ASSERT_TRUE (expected_mesh.addFace (vi [7], vi [9], vi [10]).isValid ());
  ASSERT_TRUE (expected_mesh.addFace (vi [5], vi [6], vi [ 7]).isValid ());

  // 'PCL_TEST_GEOMETRY_BINARY_DIR' defined in CMakeLists.txt
  std::string filename = std::string (PCL_TEST_GEOMETRY_BINARY_DIR).append ("/test_mesh_io_mesh_tmp.txt");
  MeshIO io;

  ASSERT_TRUE (io.write (filename, expected_mesh));
  ASSERT_TRUE (io.read  (filename, loaded_mesh));

  ASSERT_TRUE (expected_mesh.isEqualTopology (loaded_mesh));

  // TODO: no test for the mesh data yet.

  std::remove (filename.c_str ());
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
