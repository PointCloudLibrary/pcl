/*
* SPDX-License-Identifier: BSD-3-Clause
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2025-, Open Perception Inc.
*
*  All rights reserved
*/

#include <pcl/test/gtest.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/fitting_surface_pdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

#include <cmath>

using namespace pcl;
using namespace pcl::io;

using Point = pcl::PointXYZ;

PointCloud<Point>::Ptr cloud (new PointCloud<Point>);

void
PointCloud2Vector3d (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (const auto &p : *cloud)
  {
    if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
      data.emplace_back (p.x, p.y, p.z);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, on_nurbs_fitting_surface_pdm)
{
  // ############################################################################
  // fit B-spline surface
  pcl::on_nurbs::NurbsDataSurface data;
  PointCloud2Vector3d (cloud, data.interior);
  // parameters
  unsigned order (3);
  unsigned refinement (5);
  unsigned iterations (10);
  unsigned mesh_resolution (256);

  pcl::on_nurbs::FittingSurface::Parameter params;
  params.interior_smoothness = 0.2;
  params.interior_weight = 1.0;
  params.boundary_smoothness = 0.2;
  params.boundary_weight = 0.0;

  // initialize
  ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
  pcl::on_nurbs::FittingSurface fit (&data, nurbs);
  //  fit.setQuiet (false); // enable/disable debug output

  // mesh for visualization
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> mesh_vertices;
  std::string mesh_id = "mesh_nurbs";
  pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);

  // surface refinement
  for (unsigned i = 0; i < refinement; i++)
  {
    fit.refine (0);
    fit.refine (1);
    fit.assemble (params);
    fit.solve ();
    pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
  }

  // surface fitting with final refinement level
  for (unsigned i = 0; i < iterations; i++)
  {
    fit.assemble (params);
    fit.solve ();
    pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
  }

  ASSERT_EQ (mesh.polygons.size (), 131072);
  // All polygons should be triangles
  for (const auto & polygon : mesh.polygons)
    EXPECT_EQ (polygon.vertices.size (), 3);

  EXPECT_EQ (mesh.polygons[10].vertices[0], 5);
  EXPECT_EQ (mesh.polygons[10].vertices[1], 6);
  EXPECT_EQ (mesh.polygons[10].vertices[2], 263);

  EXPECT_EQ (mesh.polygons[200].vertices[0], 100);
  EXPECT_EQ (mesh.polygons[200].vertices[1], 101);
  EXPECT_EQ (mesh.polygons[200].vertices[2], 358);

  EXPECT_EQ (mesh.polygons[1000].vertices[0], 501);
  EXPECT_EQ (mesh.polygons[1000].vertices[1], 502);
  EXPECT_EQ (mesh.polygons[1000].vertices[2], 759);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load file
  loadPCDFile (argv[1], *cloud);

  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
