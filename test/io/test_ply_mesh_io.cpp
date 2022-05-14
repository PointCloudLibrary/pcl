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

#include <pcl/test/gtest.h>
#include <pcl/type_traits.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

std::string mesh_file_vtk_;


TEST (PCL, PLYPolygonMeshIO)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileVTK (mesh_file_vtk_, mesh);
  // Save ascii
  pcl::io::savePLYFile ("test_mesh_ascii.ply", mesh);
  // Save binary
  pcl::io::savePLYFileBinary ("test_mesh_binary.ply", mesh);
  // Load both with vtk ply parser
  pcl::PolygonMesh mesh_ascii_vtk;
  pcl::io::loadPolygonFilePLY ("test_mesh_ascii.ply", mesh_ascii_vtk);
  pcl::PolygonMesh mesh_binary_vtk;
  pcl::io::loadPolygonFilePLY ("test_mesh_binary.ply", mesh_binary_vtk);
  // Load both with pcl ply parser
  pcl::PolygonMesh mesh_ascii_pcl;
  pcl::io::loadPLYFile ("test_mesh_ascii.ply", mesh_ascii_pcl);
  pcl::PolygonMesh mesh_binary_pcl;
  pcl::io::loadPLYFile ("test_mesh_binary.ply", mesh_binary_pcl);
  // Compare the 3
  pcl::PointCloud<pcl::PointXYZ> verts, verts_ascii_vtk, verts_binary_vtk, verts_ascii_pcl, verts_binary_pcl;
  pcl::fromPCLPointCloud2 (mesh.cloud, verts);
  pcl::fromPCLPointCloud2 (mesh_ascii_vtk.cloud, verts_ascii_vtk);
  pcl::fromPCLPointCloud2 (mesh_binary_vtk.cloud, verts_binary_vtk);
  pcl::fromPCLPointCloud2 (mesh_ascii_pcl.cloud, verts_ascii_pcl);
  pcl::fromPCLPointCloud2 (mesh_binary_pcl.cloud, verts_binary_pcl);
  EXPECT_EQ (verts_ascii_vtk.size (), verts.size ());
  EXPECT_EQ (verts_binary_vtk.size (), verts.size ());
  EXPECT_EQ (verts_ascii_pcl.size (), verts.size ());
  EXPECT_EQ (verts_binary_pcl.size (), verts.size ());
  for (std::size_t i = 0; i < verts.size (); i++)
  {
    EXPECT_NEAR (verts_ascii_vtk.at (i).x, verts.at (i).x, 1E-2);
    EXPECT_NEAR (verts_ascii_vtk.at (i).y, verts.at (i).y, 1E-2);
    EXPECT_NEAR (verts_ascii_vtk.at (i).z, verts.at (i).z, 1E-2);
    EXPECT_NEAR (verts_binary_vtk.at (i).x, verts.at (i).x, 1E-4);
    EXPECT_NEAR (verts_binary_vtk.at (i).y, verts.at (i).y, 1E-4);
    EXPECT_NEAR (verts_binary_vtk.at (i).z, verts.at (i).z, 1E-4);
    EXPECT_NEAR (verts_ascii_pcl.at (i).x, verts.at (i).x, 1E-2);
    EXPECT_NEAR (verts_ascii_pcl.at (i).y, verts.at (i).y, 1E-2);
    EXPECT_NEAR (verts_ascii_pcl.at (i).z, verts.at (i).z, 1E-2);
    EXPECT_NEAR (verts_binary_pcl.at (i).x, verts.at (i).x, 1E-4);
    EXPECT_NEAR (verts_binary_pcl.at (i).y, verts.at (i).y, 1E-4);
    EXPECT_NEAR (verts_binary_pcl.at (i).z, verts.at (i).z, 1E-4);
  }
  ASSERT_EQ (mesh_ascii_vtk.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_binary_vtk.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_ascii_pcl.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_binary_pcl.polygons.size (), mesh.polygons.size ());
  for (std::size_t i = 0; i < mesh.polygons.size (); i++)
  {
    ASSERT_EQ (mesh_ascii_vtk.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_binary_vtk.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_ascii_pcl.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_binary_pcl.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    for (std::size_t j = 0; j < mesh.polygons[i].vertices.size (); j++)
    {
      EXPECT_EQ (mesh_ascii_vtk.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_binary_vtk.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_ascii_pcl.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_binary_pcl.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
    }
  }

  remove ("test_mesh_ascii.ply");
  remove ("test_mesh_binary.ply");
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
  pcl::fromPCLPointCloud2 (mesh.cloud, vertices_rgb);
  pcl::fromPCLPointCloud2 (mesh.cloud, vertices_rgba);
  mesh_rgb.polygons = mesh.polygons;
  mesh_rgba.polygons = mesh.polygons;
  for (std::size_t i = 0; i < vertices_rgb.size (); ++i)
  {
    pcl::PointXYZRGB &pt_rgb = vertices_rgb.at (i);
    pcl::PointXYZRGBA &pt_rgba = vertices_rgba.at (i);
    pt_rgb.r = pt_rgba.r = static_cast<std::uint8_t> (rand () % 256);
    pt_rgb.g = pt_rgba.g = static_cast<std::uint8_t> (rand () % 256);
    pt_rgb.b = pt_rgba.b = static_cast<std::uint8_t> (rand () % 256);
    pt_rgba.a = static_cast<std::uint8_t> (rand () % 256);
  }
  pcl::toPCLPointCloud2 (vertices_rgb, mesh_rgb.cloud);
  pcl::toPCLPointCloud2 (vertices_rgba, mesh_rgba.cloud);
  // Save ascii
  pcl::io::savePLYFile ("test_mesh_rgb_ascii.ply", mesh_rgb);
  pcl::io::savePLYFile ("test_mesh_rgba_ascii.ply", mesh_rgba);
  // Save binary
  pcl::io::savePLYFileBinary ("test_mesh_rgb_binary.ply", mesh_rgb);
  pcl::io::savePLYFileBinary ("test_mesh_rgba_binary.ply", mesh_rgba);
  // Load both with vtk ply parser
  pcl::PolygonMesh mesh_rgb_ascii_vtk;
  pcl::PolygonMesh mesh_rgba_ascii_vtk;
  pcl::io::loadPolygonFilePLY ("test_mesh_rgb_ascii.ply", mesh_rgb_ascii_vtk);
  pcl::io::loadPolygonFilePLY ("test_mesh_rgba_ascii.ply", mesh_rgba_ascii_vtk);
  pcl::PolygonMesh mesh_rgb_binary_vtk;
  pcl::PolygonMesh mesh_rgba_binary_vtk;
  pcl::io::loadPolygonFilePLY ("test_mesh_rgb_binary.ply", mesh_rgb_binary_vtk);
  pcl::io::loadPolygonFilePLY ("test_mesh_rgba_binary.ply", mesh_rgba_binary_vtk);
  // Load both with pcl ply parser
  pcl::PolygonMesh mesh_rgb_ascii_pcl;
  pcl::PolygonMesh mesh_rgba_ascii_pcl;
  pcl::io::loadPolygonFilePLY ("test_mesh_rgb_ascii.ply", mesh_rgb_ascii_pcl);
  pcl::io::loadPolygonFilePLY ("test_mesh_rgba_ascii.ply", mesh_rgba_ascii_pcl);
  pcl::PolygonMesh mesh_rgb_binary_pcl;
  pcl::PolygonMesh mesh_rgba_binary_pcl;
  pcl::io::loadPolygonFilePLY ("test_mesh_rgb_binary.ply", mesh_rgb_binary_pcl);
  pcl::io::loadPolygonFilePLY ("test_mesh_rgba_binary.ply", mesh_rgba_binary_pcl);
  // Compare the 5
  pcl::PointCloud<pcl::PointXYZRGBA> verts_rgba_ascii_vtk, verts_rgba_binary_vtk;
  pcl::PointCloud<pcl::PointXYZRGB> verts_rgb_ascii_vtk, verts_rgb_binary_vtk;
  pcl::fromPCLPointCloud2 (mesh_rgb_ascii_vtk.cloud,  verts_rgb_ascii_vtk);
  pcl::fromPCLPointCloud2 (mesh_rgba_ascii_vtk.cloud,  verts_rgba_ascii_vtk);
  pcl::fromPCLPointCloud2 (mesh_rgb_binary_vtk.cloud, verts_rgb_binary_vtk);
  pcl::fromPCLPointCloud2 (mesh_rgba_binary_vtk.cloud, verts_rgba_binary_vtk);
  pcl::PointCloud<pcl::PointXYZRGBA> verts_rgba_ascii_pcl, verts_rgba_binary_pcl;
  pcl::PointCloud<pcl::PointXYZRGB> verts_rgb_ascii_pcl, verts_rgb_binary_pcl;
  pcl::fromPCLPointCloud2 (mesh_rgb_ascii_pcl.cloud,  verts_rgb_ascii_pcl);
  pcl::fromPCLPointCloud2 (mesh_rgba_ascii_pcl.cloud,  verts_rgba_ascii_pcl);
  pcl::fromPCLPointCloud2 (mesh_rgb_binary_pcl.cloud, verts_rgb_binary_pcl);
  pcl::fromPCLPointCloud2 (mesh_rgba_binary_pcl.cloud, verts_rgba_binary_pcl);
  ASSERT_EQ (verts_rgb_ascii_vtk.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgba_ascii_vtk.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgb_binary_vtk.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgba_binary_vtk.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgb_ascii_pcl.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgba_ascii_pcl.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgb_binary_pcl.size (), vertices_rgba.size ());
  ASSERT_EQ (verts_rgba_binary_pcl.size (), vertices_rgba.size ());
  for (std::size_t i = 0; i < vertices_rgba.size (); i++)
  {
    EXPECT_NEAR (verts_rgba_ascii_vtk.at (i).x, vertices_rgba.at (i).x, 1E-2);
    EXPECT_NEAR (verts_rgba_ascii_vtk.at (i).y, vertices_rgba.at (i).y, 1E-2);
    EXPECT_NEAR (verts_rgba_ascii_vtk.at (i).z, vertices_rgba.at (i).z, 1E-2);
    EXPECT_EQ   (verts_rgba_ascii_vtk.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgba_ascii_vtk.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgba_ascii_vtk.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgba_binary_vtk.at (i).x, vertices_rgba.at (i).x, 1E-4);
    EXPECT_NEAR (verts_rgba_binary_vtk.at (i).y, vertices_rgba.at (i).y, 1E-4);
    EXPECT_NEAR (verts_rgba_binary_vtk.at (i).z, vertices_rgba.at (i).z, 1E-4);
    EXPECT_EQ   (verts_rgba_binary_vtk.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgba_binary_vtk.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgba_binary_vtk.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgb_ascii_vtk.at (i).x, vertices_rgba.at (i).x, 1E-2);
    EXPECT_NEAR (verts_rgb_ascii_vtk.at (i).y, vertices_rgba.at (i).y, 1E-2);
    EXPECT_NEAR (verts_rgb_ascii_vtk.at (i).z, vertices_rgba.at (i).z, 1E-2);
    EXPECT_EQ   (verts_rgb_ascii_vtk.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgb_ascii_vtk.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgb_ascii_vtk.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgb_binary_vtk.at (i).x, vertices_rgba.at (i).x, 1E-4);
    EXPECT_NEAR (verts_rgb_binary_vtk.at (i).y, vertices_rgba.at (i).y, 1E-4);
    EXPECT_NEAR (verts_rgb_binary_vtk.at (i).z, vertices_rgba.at (i).z, 1E-4);
    EXPECT_EQ   (verts_rgb_binary_vtk.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgb_binary_vtk.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgb_binary_vtk.at (i).b, vertices_rgba.at (i).b);

    EXPECT_NEAR (verts_rgba_ascii_pcl.at (i).x, vertices_rgba.at (i).x, 1E-2);
    EXPECT_NEAR (verts_rgba_ascii_pcl.at (i).y, vertices_rgba.at (i).y, 1E-2);
    EXPECT_NEAR (verts_rgba_ascii_pcl.at (i).z, vertices_rgba.at (i).z, 1E-2);
    EXPECT_EQ   (verts_rgba_ascii_pcl.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgba_ascii_pcl.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgba_ascii_pcl.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgba_binary_pcl.at (i).x, vertices_rgba.at (i).x, 1E-4);
    EXPECT_NEAR (verts_rgba_binary_pcl.at (i).y, vertices_rgba.at (i).y, 1E-4);
    EXPECT_NEAR (verts_rgba_binary_pcl.at (i).z, vertices_rgba.at (i).z, 1E-4);
    EXPECT_EQ   (verts_rgba_binary_pcl.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgba_binary_pcl.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgba_binary_pcl.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgb_ascii_pcl.at (i).x, vertices_rgba.at (i).x, 1E-2);
    EXPECT_NEAR (verts_rgb_ascii_pcl.at (i).y, vertices_rgba.at (i).y, 1E-2);
    EXPECT_NEAR (verts_rgb_ascii_pcl.at (i).z, vertices_rgba.at (i).z, 1E-2);
    EXPECT_EQ   (verts_rgb_ascii_pcl.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgb_ascii_pcl.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgb_ascii_pcl.at (i).b, vertices_rgba.at (i).b);
    EXPECT_NEAR (verts_rgb_binary_pcl.at (i).x, vertices_rgba.at (i).x, 1E-4);
    EXPECT_NEAR (verts_rgb_binary_pcl.at (i).y, vertices_rgba.at (i).y, 1E-4);
    EXPECT_NEAR (verts_rgb_binary_pcl.at (i).z, vertices_rgba.at (i).z, 1E-4);
    EXPECT_EQ   (verts_rgb_binary_pcl.at (i).r, vertices_rgba.at (i).r);
    EXPECT_EQ   (verts_rgb_binary_pcl.at (i).g, vertices_rgba.at (i).g);
    EXPECT_EQ   (verts_rgb_binary_pcl.at (i).b, vertices_rgba.at (i).b);
  }
  ASSERT_EQ (mesh_rgb_ascii_vtk.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgba_ascii_vtk.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgb_binary_vtk.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgba_binary_vtk.polygons.size (), mesh.polygons.size ());
  for (std::size_t i = 0; i < mesh.polygons.size (); i++)
  {
    ASSERT_EQ (mesh_rgb_ascii_vtk.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgba_ascii_vtk.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgb_binary_vtk.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgba_binary_vtk.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    for (std::size_t j = 0; j < mesh.polygons[i].vertices.size (); j++)
    {
      EXPECT_EQ (mesh_rgb_ascii_vtk.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgba_ascii_vtk.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgb_binary_vtk.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgba_binary_vtk.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
    }
  }

  ASSERT_EQ (mesh_rgb_ascii_pcl.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgba_ascii_pcl.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgb_binary_pcl.polygons.size (), mesh.polygons.size ());
  ASSERT_EQ (mesh_rgba_binary_pcl.polygons.size (), mesh.polygons.size ());
  for (std::size_t i = 0; i < mesh.polygons.size (); i++)
  {
    ASSERT_EQ (mesh_rgb_ascii_pcl.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgba_ascii_pcl.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgb_binary_pcl.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    ASSERT_EQ (mesh_rgba_binary_pcl.polygons[i].vertices.size (), mesh.polygons[i].vertices.size ());
    for (std::size_t j = 0; j < mesh.polygons[i].vertices.size (); j++)
    {
      EXPECT_EQ (mesh_rgb_ascii_pcl.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgba_ascii_pcl.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgb_binary_pcl.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
      EXPECT_EQ (mesh_rgba_binary_pcl.polygons[i].vertices[j], mesh.polygons[i].vertices[j]);
    }
  }

  remove ("test_mesh_rgb_ascii.ply");
  remove ("test_mesh_rgba_ascii.ply");
  remove ("test_mesh_rgb_binary.ply");
  remove ("test_mesh_rgba_binary.ply");
}

TEST (PCL, PLYPolygonMeshSpecificFieldOrder)
{ // test a specific order of xyz, rgba, and normal fields
  pcl::PolygonMesh mesh;
  auto add_field = [](std::vector<pcl::PCLPointField>& fields, const std::string& name, const pcl::uindex_t offset, const std::uint8_t datatype)
                   { fields.emplace_back(); fields.back().name = name; fields.back().offset = offset; fields.back().datatype = datatype; fields.back().count = 1; };
  add_field(mesh.cloud.fields, "x", 0, pcl::PCLPointField::PointFieldTypes::FLOAT32);
  add_field(mesh.cloud.fields, "y", 4, pcl::PCLPointField::PointFieldTypes::FLOAT32);
  add_field(mesh.cloud.fields, "z", 8, pcl::PCLPointField::PointFieldTypes::FLOAT32);
  add_field(mesh.cloud.fields, "normal_x", 12, pcl::PCLPointField::PointFieldTypes::FLOAT32);
  add_field(mesh.cloud.fields, "normal_y", 16, pcl::PCLPointField::PointFieldTypes::FLOAT32);
  add_field(mesh.cloud.fields, "normal_z", 20, pcl::PCLPointField::PointFieldTypes::FLOAT32);
  add_field(mesh.cloud.fields, "rgba", 24, pcl::PCLPointField::PointFieldTypes::UINT32);
  mesh.cloud.height = mesh.cloud.width = 1;
  mesh.cloud.data.resize(28);
  const float x = 0.0, y = 1.0, z = 2.0, normal_x = 1.0, normal_y = 0.0, normal_z = 0.0;
  const std::uint32_t rgba = 0x326496;
  memcpy(&mesh.cloud.data[0], &x, sizeof(float));
  memcpy(&mesh.cloud.data[4], &y, sizeof(float));
  memcpy(&mesh.cloud.data[8], &z, sizeof(float));
  memcpy(&mesh.cloud.data[12], &normal_x, sizeof(float));
  memcpy(&mesh.cloud.data[16], &normal_y, sizeof(float));
  memcpy(&mesh.cloud.data[20], &normal_z, sizeof(float));
  memcpy(&mesh.cloud.data[24], &rgba, sizeof(std::uint32_t));

  pcl::io::savePLYFileBinary("test_mesh_xyzrgbnormal_binary.ply", mesh);
  pcl::PolygonMesh mesh_in_binary;
  pcl::io::loadPLYFile("test_mesh_xyzrgbnormal_binary.ply", mesh_in_binary);
  ASSERT_EQ (mesh.cloud.data.size(), mesh_in_binary.cloud.data.size());
  for(std::size_t i=0; i<mesh.cloud.data.size(); ++i) {
    EXPECT_EQ (mesh.cloud.data[i], mesh_in_binary.cloud.data[i]);
  }
  remove ("test_mesh_xyzrgbnormal_binary.ply");

  pcl::io::savePLYFile("test_mesh_xyzrgbnormal_ascii.ply", mesh);
  pcl::PolygonMesh mesh_in_ascii;
  pcl::io::loadPLYFile("test_mesh_xyzrgbnormal_ascii.ply", mesh_in_ascii);
  ASSERT_EQ (mesh.cloud.data.size(), mesh_in_ascii.cloud.data.size());
  for(std::size_t i=0; i<mesh.cloud.data.size(); ++i) {
    EXPECT_EQ (mesh.cloud.data[i], mesh_in_ascii.cloud.data[i]);
  }
  remove ("test_mesh_xyzrgbnormal_ascii.ply");
}

TEST (PCL, PLYPolygonMeshUintIndices)
{
  std::ofstream fs;
  fs.open ("mesh_uint_indices.ply");
  fs <<
    "ply\n"
    "format ascii 1.0\n"
    "element vertex 3\n"
    "property float x\n"
    "property float y\n"
    "property float z\n"
    "element face 1\n"
    "property list uchar uint vertex_indices\n"
    "end_header\n"
    "0.0 0.0 0.0\n"
    "1.0 0.0 0.0\n"
    "1.0 1.1 0.0\n"
    "3 2 0 1\n";
  fs.close();
  pcl::PolygonMesh mesh;
  int const res = pcl::io::loadPLYFile("mesh_uint_indices.ply", mesh);
  EXPECT_NE (res, -1);
  EXPECT_EQ (mesh.cloud.width, 3);
  EXPECT_EQ (mesh.cloud.height, 1);
  EXPECT_EQ (mesh.polygons.size(), 1);
  EXPECT_EQ (mesh.polygons.front().vertices.size(), 3);

  remove("mesh_uint_indices.ply");
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
