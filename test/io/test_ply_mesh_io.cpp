/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: test_mesh_io.cpp 8543 2013-01-17 23:31:01Z sdmiller $
 *
 */

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <fstream>
#include <locale>
#include <stdexcept>

std::string mesh_file_vtk_;


TEST (PCL, PLYPolygonMeshIO)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileVTK (mesh_file_vtk_, mesh);
  // Save ascii
  pcl::io::savePLYFile ("test_mesh_ascii.ply", mesh);
  // Save binary
  pcl::io::savePLYFileBinary ("test_mesh_binary.ply", mesh);
  // Load both
  pcl::PolygonMesh mesh_ascii;
  pcl::io::loadPolygonFilePLY ("test_mesh_ascii.ply", mesh_ascii);
  pcl::PolygonMesh mesh_binary;
  pcl::io::loadPolygonFilePLY ("test_mesh_binary.ply", mesh_binary);
  // Compare the 3
  pcl::PointCloud<pcl::PointXYZ> verts, verts_ascii, verts_binary;
  pcl::fromROSMsg (mesh.cloud, verts);
  pcl::fromROSMsg (mesh_ascii.cloud, verts_ascii);
  pcl::fromROSMsg (mesh_binary.cloud, verts_binary);
  EXPECT_EQ (verts_ascii.size (), verts.size ());
  EXPECT_EQ (verts_binary.size (), verts.size ());
  for (size_t i = 0; i < verts.size (); i++)
  {
    EXPECT_NEAR (verts_ascii.at (i).x, verts.at (i).x, 1E-2);
    EXPECT_NEAR (verts_ascii.at (i).y, verts.at (i).y, 1E-2);
    EXPECT_NEAR (verts_ascii.at (i).z, verts.at (i).z, 1E-2);
    EXPECT_NEAR (verts_binary.at (i).x, verts.at (i).x, 1E-4);
    EXPECT_NEAR (verts_binary.at (i).y, verts.at (i).y, 1E-4);
    EXPECT_NEAR (verts_binary.at (i).z, verts.at (i).z, 1E-4);
  }
  ASSERT_EQ (mesh_ascii.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_binary.polygons.size (), mesh.polygons.size ());
  for (size_t i = 0; i < mesh.polygons.size (); i++)
  {
    ASSERT_EQ (mesh_ascii.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_binary.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    for (size_t j = 0; j < mesh.polygons[i].vertices.size (); j++)
    {
      EXPECT_EQ (mesh_ascii.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_binary.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
    }
  }
}

TEST (PCL, PLYPolygonMeshColoredIO)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileVTK (mesh_file_vtk_, mesh);
  // Artificially color
  pcl::PolygonMesh mesh_rgb;
  pcl::PolygonMesh mesh_rgba;
  pcl::PointCloud<pcl::PointXYZRGB> vertices_rgb;
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_rgba;
  pcl::fromROSMsg (mesh.cloud, vertices_rgb);
  pcl::fromROSMsg (mesh.cloud, vertices_rgba);
  mesh_rgb.polygons = mesh.polygons;
  mesh_rgba.polygons = mesh.polygons;
  for (size_t i = 0; i < vertices_rgb.size (); ++i)
  {
    pcl::PointXYZRGB &pt_rgb = vertices_rgb.at (i);
    pcl::PointXYZRGBA &pt_rgba = vertices_rgba.at (i);
    pt_rgb.r = pt_rgba.r = static_cast<uint8_t> (rand () % 256);
    pt_rgb.g = pt_rgba.g = static_cast<uint8_t> (rand () % 256);
    pt_rgb.b = pt_rgba.b = static_cast<uint8_t> (rand () % 256);
    pt_rgba.a = static_cast<uint8_t> (rand () % 256);
  }
  pcl::toROSMsg (vertices_rgb, mesh_rgb.cloud);
  pcl::toROSMsg (vertices_rgba, mesh_rgba.cloud);
  // Save ascii
  pcl::io::savePLYFile ("test_mesh_rgb_ascii.ply", mesh_rgb);
  pcl::io::savePLYFile ("test_mesh_rgba_ascii.ply", mesh_rgba);
  // Save binary
  pcl::io::savePLYFileBinary ("test_mesh_rgb_binary.ply", mesh_rgb);
  pcl::io::savePLYFileBinary ("test_mesh_rgba_binary.ply", mesh_rgba);
  // Load both
  pcl::PolygonMesh mesh_rgb_ascii;
  pcl::PolygonMesh mesh_rgba_ascii;
  pcl::io::loadPolygonFilePLY ("test_mesh_rgb_ascii.ply", mesh_rgb_ascii);
  pcl::io::loadPolygonFilePLY ("test_mesh_rgba_ascii.ply", mesh_rgba_ascii);
  pcl::PolygonMesh mesh_rgb_binary;
  pcl::PolygonMesh mesh_rgba_binary;
  pcl::io::loadPolygonFilePLY ("test_mesh_rgb_binary.ply", mesh_rgb_binary);
  pcl::io::loadPolygonFilePLY ("test_mesh_rgba_binary.ply", mesh_rgba_binary);
  // Compare the 5
  pcl::PointCloud<pcl::PointXYZRGBA> verts_rgba_ascii, verts_rgba_binary;
  pcl::PointCloud<pcl::PointXYZRGB> verts_rgb_ascii, verts_rgb_binary;
  pcl::fromROSMsg (mesh_rgb_ascii.cloud,  verts_rgb_ascii);
  pcl::fromROSMsg (mesh_rgba_ascii.cloud,  verts_rgba_ascii);
  pcl::fromROSMsg (mesh_rgb_binary.cloud, verts_rgb_binary);
  pcl::fromROSMsg (mesh_rgba_binary.cloud, verts_rgba_binary);
  ASSERT_EQ (verts_rgb_ascii.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgba_ascii.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgb_binary.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgba_binary.size (), vertices_rgba.size ());
  for (size_t i = 0; i < vertices_rgba.size (); i++)
  {
    EXPECT_NEAR (verts_rgba_ascii.at (i).x, vertices_rgba.at (i).x, 1E-2);
    EXPECT_NEAR (verts_rgba_ascii.at (i).y, vertices_rgba.at (i).y, 1E-2);
    EXPECT_NEAR (verts_rgba_ascii.at (i).z, vertices_rgba.at (i).z, 1E-2);
    EXPECT_EQ   (verts_rgba_ascii.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgba_ascii.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgba_ascii.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgba_binary.at (i).x, vertices_rgba.at (i).x, 1E-4);
    EXPECT_NEAR (verts_rgba_binary.at (i).y, vertices_rgba.at (i).y, 1E-4);
    EXPECT_NEAR (verts_rgba_binary.at (i).z, vertices_rgba.at (i).z, 1E-4);
    EXPECT_EQ   (verts_rgba_binary.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgba_binary.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgba_binary.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgb_ascii.at (i).x, vertices_rgba.at (i).x, 1E-2);
    EXPECT_NEAR (verts_rgb_ascii.at (i).y, vertices_rgba.at (i).y, 1E-2);
    EXPECT_NEAR (verts_rgb_ascii.at (i).z, vertices_rgba.at (i).z, 1E-2);
    EXPECT_EQ   (verts_rgb_ascii.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgb_ascii.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgb_ascii.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgb_binary.at (i).x, vertices_rgba.at (i).x, 1E-4);
    EXPECT_NEAR (verts_rgb_binary.at (i).y, vertices_rgba.at (i).y, 1E-4);
    EXPECT_NEAR (verts_rgb_binary.at (i).z, vertices_rgba.at (i).z, 1E-4);
    EXPECT_EQ   (verts_rgb_binary.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgb_binary.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgb_binary.at (i).b, vertices_rgba.at (i).b);
  }
  ASSERT_EQ (mesh_rgb_ascii.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgba_ascii.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgb_binary.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgba_binary.polygons.size (), mesh.polygons.size ());
  for (size_t i = 0; i < mesh.polygons.size (); i++)
  {
    ASSERT_EQ (mesh_rgb_ascii.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgba_ascii.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgb_binary.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgba_binary.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    for (size_t j = 0; j < mesh.polygons[i].vertices.size (); j++)
    {
      EXPECT_EQ (mesh_rgb_ascii.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgba_ascii.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgb_binary.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgba_binary.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
    }
  }
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  if (argc < 2)
  {
    std::cerr << "No test files were given. Please add the path to TUM_Rabbit.vtk to this test." << std::endl;
    return (-1);
  }
  mesh_file_vtk_ = argv[1]; //TODO
  return (RUN_ALL_TESTS ());
}
