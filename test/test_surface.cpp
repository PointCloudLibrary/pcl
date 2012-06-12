/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *
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
 * $Id$
 *
 */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/mls_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/common/common.h>
#include <boost/random.hpp>

#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
using namespace pcl;
using namespace pcl::io;
using namespace std;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree;
search::KdTree<PointNormal>::Ptr tree2;

// add by ktran to test update functions
PointCloud<PointXYZ>::Ptr cloud1 (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals1 (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree3;
search::KdTree<PointNormal>::Ptr tree4;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, MarchingCubesTest)
{
  MarchingCubesHoppe<PointNormal> hoppe;
  hoppe.setIsoLevel (0);
  hoppe.setGridResolution (30, 30, 30);
  hoppe.setPercentageExtendGrid (0.3);
  hoppe.setInputCloud (cloud_with_normals);
  PointCloud<PointNormal> points;
  std::vector<Vertices> vertices;
  hoppe.reconstruct (points, vertices);

  EXPECT_NEAR (points.points[points.size()/2].x, -0.042528, 1e-3);
  EXPECT_NEAR (points.points[points.size()/2].y, 0.080196, 1e-3);
  EXPECT_NEAR (points.points[points.size()/2].z, 0.043159, 1e-3);
  EXPECT_EQ (vertices[vertices.size ()/2].vertices[0], 10854);
  EXPECT_EQ (vertices[vertices.size ()/2].vertices[1], 10855);
  EXPECT_EQ (vertices[vertices.size ()/2].vertices[2], 10856);


  MarchingCubesRBF<PointNormal> rbf;
  rbf.setIsoLevel (0);
  rbf.setGridResolution (20, 20, 20);
  rbf.setPercentageExtendGrid (0.1);
  rbf.setInputCloud (cloud_with_normals);
  rbf.setOffSurfaceDisplacement (0.02f);
  rbf.reconstruct (points, vertices);

  EXPECT_NEAR (points.points[points.size()/2].x, -0.033919, 1e-2);
  EXPECT_NEAR (points.points[points.size()/2].y, 0.127359, 1e-2);
  EXPECT_NEAR (points.points[points.size()/2].z, 0.034632, 1e-2);
  EXPECT_EQ (vertices[vertices.size ()/2].vertices[0], 4344);
  EXPECT_EQ (vertices[vertices.size ()/2].vertices[1], 4345);
  EXPECT_EQ (vertices[vertices.size ()/2].vertices[2], 4346);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, MovingLeastSquares)
{
  // Init objects
  PointCloud<PointXYZ> mls_points;
  PointCloud<PointNormal>::Ptr mls_normals (new PointCloud<PointNormal> ());
  MovingLeastSquares<PointXYZ, PointNormal> mls;

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setComputeNormals (true);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (*mls_normals);

  EXPECT_NEAR (mls_normals->points[0].x, 0.005417, 1e-3);
  EXPECT_NEAR (mls_normals->points[0].y, 0.113463, 1e-3);
  EXPECT_NEAR (mls_normals->points[0].z, 0.040715, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[0].normal[0]), 0.111894, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[0].normal[1]), 0.594906, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[0].normal[2]), 0.795969, 1e-3);
  EXPECT_NEAR (mls_normals->points[0].curvature, 0.012019, 1e-3);


  MovingLeastSquaresOMP<PointXYZ, PointNormal> mls_omp;

  mls_omp.setInputCloud (cloud);
  mls_omp.setComputeNormals (true);
  mls_omp.setPolynomialFit (true);
  mls_omp.setSearchMethod (tree);
  mls_omp.setSearchRadius (0.03);
  mls_omp.setNumberOfThreads (4);

  // Reconstruct
  mls_omp.process (*mls_normals);

  int count = 0;
  for (size_t i = 0; i < mls_normals->size (); ++i)
  {
	if (fabs (mls_normals->points[i].x - 0.005417) < 1e-3 &&
	    fabs (mls_normals->points[i].y - 0.113463) < 1e-3 &&
	    fabs (mls_normals->points[i].z - 0.040715) < 1e-3 &&
	    fabs (fabs (mls_normals->points[i].normal[0]) - 0.111894) < 1e-3 &&
		fabs (fabs (mls_normals->points[i].normal[1]) - 0.594906) < 1e-3 &&
		fabs (fabs (mls_normals->points[i].normal[2]) - 0.795969) < 1e-3 &&
		fabs (mls_normals->points[i].curvature - 0.012019) < 1e-3)
		count ++;
  }

  EXPECT_EQ (count, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GreedyProjectionTriangulation)
{
  // Init objects
  PolygonMesh triangles;
  GreedyProjectionTriangulation<PointNormal> gp3;

  // Set parameters
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.setSearchRadius (0.025);
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Reconstruct
  gp3.reconstruct (triangles);
  //saveVTKFile ("./test/bun0-gp3.vtk", triangles);
  EXPECT_EQ (triangles.cloud.width, cloud_with_normals->width);
  EXPECT_EQ (triangles.cloud.height, cloud_with_normals->height);
  EXPECT_NEAR (int (triangles.polygons.size ()), 685, 5);

  // Check triangles
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.size ()), 3);
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (0)), 0);
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (1)), 12);
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (2)), 198);
  EXPECT_EQ (int (triangles.polygons.at (684).vertices.size ()), 3);
  EXPECT_EQ (int (triangles.polygons.at (684).vertices.at (0)), 393);
  EXPECT_EQ (int (triangles.polygons.at (684).vertices.at (1)), 394);
  EXPECT_EQ (int (triangles.polygons.at (684).vertices.at (2)), 395);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  int nr_points = cloud_with_normals->width * cloud_with_normals->height;
  EXPECT_EQ (int (parts.size ()), nr_points);
  EXPECT_EQ (int (states.size ()), nr_points);
  EXPECT_EQ (parts[0], 0);
  EXPECT_EQ (states[0], gp3.COMPLETED);
  EXPECT_EQ (parts[393], 5);
  EXPECT_EQ (states[393], gp3.BOUNDARY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GreedyProjectionTriangulation_Merge2Meshes)
{
  // check if exist update cloud
  if(cloud_with_normals1->width * cloud_with_normals1->height > 0){
    // Init objects
    PolygonMesh triangles;
    PolygonMesh triangles1;
    GreedyProjectionTriangulation<PointNormal> gp3;
    GreedyProjectionTriangulation<PointNormal> gp31;

    // Set parameters
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.setSearchRadius (0.025);
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // for mesh 2
    // Set parameters
    gp31.setInputCloud (cloud_with_normals1);
    gp31.setSearchMethod (tree4);
    gp31.setSearchRadius (0.025);
    gp31.setMu (2.5);
    gp31.setMaximumNearestNeighbors (100);
    gp31.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp31.setMinimumAngle(M_PI/18); // 10 degrees
    gp31.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp31.setNormalConsistency(false);


    // Reconstruct
    //gp3.reconstruct (triangles);
    //saveVTKFile ("bun01.vtk", triangles);

    //gp31.reconstruct (triangles1);
    //saveVTKFile ("bun02.vtk", triangles1);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, UpdateMesh_With_TextureMapping)
{
  if(cloud_with_normals1->width * cloud_with_normals1->height > 0){
    // Init objects
    PolygonMesh triangles;
    PolygonMesh triangles1;
    GreedyProjectionTriangulation<PointNormal> gp3;
    GreedyProjectionTriangulation<PointNormal> gp31;

    // Set parameters
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.setSearchRadius (0.025);
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    gp3.reconstruct (triangles);

    EXPECT_EQ (triangles.cloud.width, cloud_with_normals->width);
    EXPECT_EQ (triangles.cloud.height, cloud_with_normals->height);
    EXPECT_EQ (int (triangles.polygons.size ()), 685);

    // update with texture mapping
    // set 2 texture for 2 mesh
    std::vector<std::string> tex_files;
    tex_files.push_back("tex4.jpg");

    // initialize texture mesh
    TextureMesh tex_mesh;
    tex_mesh.header = triangles.header;
    tex_mesh.cloud = triangles.cloud;

    // add the 1st mesh
    tex_mesh.tex_polygons.push_back(triangles.polygons);

    // update mesh and texture mesh
    //gp3.updateMesh(cloud_with_normals1, triangles, tex_mesh);
    // set texture for added cloud
    //tex_files.push_back("tex8.jpg");
    // save updated mesh
    //saveVTKFile ("update_bunny.vtk", triangles);

    //TextureMapping<PointXYZ> tm;

    //// set mesh scale control
    //tm.setF(0.01);

    //// set vector field
    //tm.setVectorField(1, 0, 0);

    //TexMaterial tex_material;

    //// default texture materials parameters
    //tex_material.tex_Ka.r = 0.2f;
    //tex_material.tex_Ka.g = 0.2f;
    //tex_material.tex_Ka.b = 0.2f;

    //tex_material.tex_Kd.r = 0.8f;
    //tex_material.tex_Kd.g = 0.8f;
    //tex_material.tex_Kd.b = 0.8f;

    //tex_material.tex_Ks.r = 1.0f;
    //tex_material.tex_Ks.g = 1.0f;
    //tex_material.tex_Ks.b = 1.0f;
    //tex_material.tex_d = 1.0f;
    //tex_material.tex_Ns = 0.0f;
    //tex_material.tex_illum = 2;

    //// set texture material paramaters
    //tm.setTextureMaterials(tex_material);

    //// set texture files
    //tm.setTextureFiles(tex_files);

    //// mapping
    //tm.mapTexture2Mesh(tex_mesh);

    //saveOBJFile ("update_bunny.obj", tex_mesh);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Organized)
{
  //construct dataset
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_organized (new pcl::PointCloud<pcl::PointXYZ> ());
  cloud_organized->width = 5;
  cloud_organized->height = 10;
  cloud_organized->points.resize (cloud_organized->width * cloud_organized->height);

  int npoints = 0;
  for (size_t i = 0; i < cloud_organized->height; i++)
  {
    for (size_t j = 0; j < cloud_organized->width; j++)
    {
      cloud_organized->points[npoints].x = static_cast<float> (i);
      cloud_organized->points[npoints].y = static_cast<float> (j);
      cloud_organized->points[npoints].z = static_cast<float> (cloud_organized->points.size ()); // to avoid shadowing
      npoints++;
    }
  }
  int nan_idx = cloud_organized->width*cloud_organized->height - 2*cloud_organized->width + 1;
  cloud_organized->points[nan_idx].x = numeric_limits<float>::quiet_NaN ();
  cloud_organized->points[nan_idx].y = numeric_limits<float>::quiet_NaN ();
  cloud_organized->points[nan_idx].z = numeric_limits<float>::quiet_NaN ();
  
  // Init objects
  PolygonMesh triangles;
  OrganizedFastMesh<PointXYZ> ofm;

  // Set parameters
  ofm.setInputCloud (cloud_organized);
  ofm.setMaxEdgeLength (1.5);
  ofm.setTrianglePixelSize (1);
  ofm.setTriangulationType (OrganizedFastMesh<PointXYZ>::TRIANGLE_ADAPTIVE_CUT);

  // Reconstruct
  ofm.reconstruct (triangles);
  //saveVTKFile ("./test/organized.vtk", triangles);

  // Check triangles
  EXPECT_EQ (triangles.cloud.width, cloud_organized->width);
  EXPECT_EQ (triangles.cloud.height, cloud_organized->height);
  EXPECT_EQ (int (triangles.polygons.size ()), 2*(triangles.cloud.width-1)*(triangles.cloud.height-1) - 4);
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.size ()), 3);
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (0)), 0);
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (1)), triangles.cloud.width+1);
  EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (2)), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GridProjection)
{
  // Init objects
  PolygonMesh grid;
  GridProjection<PointNormal> gp;

  // Set parameters
  gp.setInputCloud (cloud_with_normals);
  gp.setSearchMethod (tree2);
  gp.setResolution (0.005);
  gp.setPaddingSize (3);

  // Reconstruct
  gp.reconstruct (grid);
  //saveVTKFile ("./test/bun0-grid.vtk", grid);
  EXPECT_GE (grid.cloud.width, 5180);
  EXPECT_GE (int (grid.polygons.size ()), 1295);
  EXPECT_EQ (int (grid.polygons.at (0).vertices.size ()), 4);
  EXPECT_EQ (int (grid.polygons.at (0).vertices.at (0)), 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_bunny)
{
  pcl::PointCloud<pcl::PointXYZ> hull;
  std::vector<pcl::Vertices> polygons;

  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud);
  chull.reconstruct (hull, polygons);

  //PolygonMesh convex;
  //toROSMsg (hull, convex.cloud);
  //convex.polygons = polygons;
  //saveVTKFile ("./test/bun0-convex.vtk", convex);

  EXPECT_EQ (polygons.size (), 206);

  //check distance between min and max in the hull
  Eigen::Vector4f min_pt_hull, max_pt_hull;
  pcl::getMinMax3D (hull, min_pt_hull, max_pt_hull);

  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D (hull, min_pt, max_pt);

  EXPECT_NEAR ((min_pt - max_pt).norm (), (min_pt_hull - max_pt_hull).norm (), 1e-5);

  //
  // Test the face-vertices-only output variant
  //

  // construct the hull mesh
  std::vector<pcl::Vertices> polygons2;
  chull.reconstruct (polygons2);

  // compare the face vertices (polygons2) to the output from the original test --- they should be identical
  ASSERT_EQ (polygons.size (), polygons2.size ());
  for (size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = polygons2[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (size_t j = 0; j < face1.vertices.size (); ++j)
    {
      ASSERT_EQ (face1.vertices[j], face2.vertices[j]);
    }
  }


  //
  // Test the PolygonMesh output variant
  //

  // construct the hull mesh
  PolygonMesh mesh;
  chull.reconstruct (mesh);

  // convert the internal PointCloud2 to a PointCloud
  PointCloud<pcl::PointXYZ> hull2;
  pcl::fromROSMsg (mesh.cloud, hull2);

  // compare the PointCloud (hull2) to the output from the original test --- they should be identical
  ASSERT_EQ (hull.points.size (), hull2.points.size ());
  for (size_t i = 0; i < hull.points.size (); ++i)
  {
    const PointXYZ & p1 = hull.points[i];
    const PointXYZ & p2 = hull2.points[i];
    ASSERT_EQ (p1.x, p2.x);
    ASSERT_EQ (p1.y, p2.y);
    ASSERT_EQ (p1.z, p2.z);
  }

  // compare the face vertices (mesh.polygons) to the output from the original test --- they should be identical
  ASSERT_EQ (polygons.size (), mesh.polygons.size ());
  for (size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = mesh.polygons[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (size_t j = 0; j < face1.vertices.size (); ++j)
    {
      ASSERT_EQ (face1.vertices[j], face2.vertices[j]);
    }
  }    

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_LTable)
{
  //construct dataset
  pcl::PointCloud<pcl::PointXYZ> cloud_out_ltable;
  cloud_out_ltable.points.resize (100);

  int npoints = 0;
  for (size_t i = 0; i < 8; i++)
  {
    for (size_t j = 0; j <= 2; j++)
    {
      cloud_out_ltable.points[npoints].x = float (i) * 0.5f;
      cloud_out_ltable.points[npoints].y = -float (j) * 0.5f;
      cloud_out_ltable.points[npoints].z = 0.f;
      npoints++;
    }
  }

  for (size_t i = 0; i <= 2; i++)
  {
    for (size_t j = 3; j < 8; j++)
    {
      cloud_out_ltable.points[npoints].x = float (i) * 0.5f;
      cloud_out_ltable.points[npoints].y = -float (j) * 0.5f;
      cloud_out_ltable.points[npoints].z = 0.f;
      npoints++;
    }
  }

  // add the five points on the hull
  cloud_out_ltable.points[npoints].x = -0.5f;
  cloud_out_ltable.points[npoints].y = 0.5f;
  cloud_out_ltable.points[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable.points[npoints].x = 4.5f;
  cloud_out_ltable.points[npoints].y = 0.5f;
  cloud_out_ltable.points[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable.points[npoints].x = 4.5f;
  cloud_out_ltable.points[npoints].y = -1.0f;
  cloud_out_ltable.points[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable.points[npoints].x = 1.0f;
  cloud_out_ltable.points[npoints].y = -4.5f;
  cloud_out_ltable.points[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable.points[npoints].x = -0.5f;
  cloud_out_ltable.points[npoints].y = -4.5f;
  cloud_out_ltable.points[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable.points.resize (npoints);

  pcl::PointCloud<pcl::PointXYZ> hull;
  std::vector<pcl::Vertices> polygons;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudptr (new pcl::PointCloud<pcl::PointXYZ> (cloud_out_ltable));
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloudptr);
  chull.reconstruct (hull, polygons);

  EXPECT_EQ (polygons.size (), 1);
  EXPECT_EQ (hull.points.size (), 5);


  //
  // Test the face-vertices-only output variant
  //

  // construct the hull mesh
  std::vector<pcl::Vertices> polygons2;
  chull.reconstruct (polygons2);

  // compare the face vertices (polygons2) to the output from the original test --- they should be identical
  ASSERT_EQ (polygons.size (), polygons2.size ());
  for (size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = polygons2[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (size_t j = 0; j < face1.vertices.size (); ++j)
    {
      ASSERT_EQ (face1.vertices[j], face2.vertices[j]);
    }
  }


  //
  // Test the PolygonMesh output variant
  //

  // construct the hull mesh
  PolygonMesh mesh;
  chull.reconstruct (mesh);

  // convert the internal PointCloud2 to a PointCloud
  PointCloud<pcl::PointXYZ> hull2;
  pcl::fromROSMsg (mesh.cloud, hull2);

  // compare the PointCloud (hull2) to the output from the original test --- they should be identical
  ASSERT_EQ (hull.points.size (), hull2.points.size ());
  for (size_t i = 0; i < hull.points.size (); ++i)
  {
    const PointXYZ & p1 = hull.points[i];
    const PointXYZ & p2 = hull2.points[i];
    ASSERT_EQ (p1.x, p2.x);
    ASSERT_EQ (p1.y, p2.y);
    ASSERT_EQ (p1.z, p2.z);
  }

  // compare the face vertices (mesh.polygons) to the output from the original test --- they should be identical
  ASSERT_EQ (polygons.size (), mesh.polygons.size ());
  for (size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = mesh.polygons[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (size_t j = 0; j < face1.vertices.size (); ++j)
    {
      ASSERT_EQ (face1.vertices[j], face2.vertices[j]);
    }
  }    

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_2dsquare)
{
  //Generate data
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  input_cloud->width = 1000000;
  input_cloud->height = 1;
  input_cloud->points.resize (input_cloud->width * input_cloud->height);
  
  //rng
  boost::mt19937 rng_alg;
  boost::uniform_01<boost::mt19937> rng (rng_alg);
  rng.base ().seed (12345u);

  for (size_t i = 0; i < input_cloud->points.size (); i++)
  {
    input_cloud->points[i].x = (2.0f * float (rng ()))-1.0f;
    input_cloud->points[i].y = (2.0f * float (rng ()))-1.0f;
    input_cloud->points[i].z = 1.0f;
  }

  //Set up for creating a hull
  pcl::PointCloud<pcl::PointXYZ> hull;
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (input_cloud);
  //chull.setDim (2); //We'll skip this, so we can check auto-detection
  chull.reconstruct (hull);

  //Check that input was correctly detected as 2D input
  ASSERT_EQ (2, chull.getDimension ());
  
  //Verify that all points lie within the plane we generated
  //This plane has normal equal to the z-axis (parallel to the xy plane, 1m up)
  Eigen::Vector4f plane_normal (0.0, 0.0, -1.0, 1.0);

  //Make sure they're actually near some edge
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > facets;
  facets.push_back (Eigen::Vector4f (-1.0, 0.0, 0.0, 1.0));
  facets.push_back (Eigen::Vector4f (-1.0, 0.0, 0.0, -1.0));
  facets.push_back (Eigen::Vector4f (0.0, -1.0, 0.0, 1.0));
  facets.push_back (Eigen::Vector4f (0.0, -1.0, 0.0, -1.0));

  //Make sure they're in the plane
  for (size_t i = 0; i < hull.points.size (); i++)
  {
    float dist = fabs (hull.points[i].getVector4fMap ().dot (plane_normal));
    EXPECT_NEAR (dist, 0.0, 1e-2);

    float min_dist = std::numeric_limits<float>::infinity ();
    for (size_t j = 0; j < facets.size (); j++)
    {
      float d2 = fabs (hull.points[i].getVector4fMap ().dot (facets[j]));
      
      if (d2 < min_dist)
        min_dist = d2;
    }
    EXPECT_NEAR (min_dist, 0.0, 1e-2);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_3dcube)
{
  //Generate data
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  input_cloud->width = 10000000;
  input_cloud->height = 1;
  input_cloud->points.resize (input_cloud->width * input_cloud->height);
  
  //rng
  boost::mt19937 rng_alg;
  boost::uniform_01<boost::mt19937> rng (rng_alg);
  rng.base ().seed (12345u);

  for (size_t i = 0; i < input_cloud->points.size (); i++)
  {
    input_cloud->points[i].x =  (2.0f * float (rng ()))-1.0f;
    input_cloud->points[i].y =  (2.0f * float (rng ()))-1.0f;
    input_cloud->points[i].z =  (2.0f * float (rng ()))-1.0f;
  }

  //Set up for creating a hull
  pcl::PointCloud<pcl::PointXYZ> hull;
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (input_cloud);
  //chull.setDim (3); //We'll skip this, so we can check auto-detection
  chull.reconstruct (hull);

  //Check that input was correctly detected as 3D input
  ASSERT_EQ (3, chull.getDimension ());
  
  //Make sure they're actually near some edge
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > facets;
  facets.push_back (Eigen::Vector4f (-1.0f, 0.0f, 0.0f, 1.0f));
  facets.push_back (Eigen::Vector4f (-1.0f, 0.0f, 0.0f, -1.0f));
  facets.push_back (Eigen::Vector4f (0.0f, -1.0f, 0.0f, 1.0f));
  facets.push_back (Eigen::Vector4f (0.0f, -1.0f, 0.0f, -1.0f));
  facets.push_back (Eigen::Vector4f (0.0f, 0.0f, -1.0f, 1.0f));
  facets.push_back (Eigen::Vector4f (0.0f, 0.0f, -1.0f, -1.0f));

  //Make sure they're near a facet
  for (size_t i = 0; i < hull.points.size (); i++)
  {
    float min_dist = std::numeric_limits<float>::infinity ();
    for (size_t j = 0; j < facets.size (); j++)
    {
      float dist = fabs (hull.points[i].getVector4fMap ().dot (facets[j]));
      
      if (dist < min_dist)
        min_dist = dist;
    }
    EXPECT_NEAR (min_dist, 0.0, 1e-2);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConcaveHull_bunny)
{
  //construct dataset
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D (new pcl::PointCloud<pcl::PointXYZ> (*cloud));
  for (size_t i = 0; i < cloud2D->points.size (); i++)
    cloud2D->points[i].z = 0;

  pcl::PointCloud<pcl::PointXYZ> alpha_shape;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud (cloud2D);
  concave_hull.setAlpha (0.5);
  concave_hull.setVoronoiCenters (voronoi_centers);
  concave_hull.reconstruct (alpha_shape, polygons_alpha);

  EXPECT_EQ (alpha_shape.points.size (), 21);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers1 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha1;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull1;
  concave_hull1.setInputCloud (cloud2D);
  concave_hull1.setAlpha (1.5);
  concave_hull1.setVoronoiCenters (voronoi_centers1);
  concave_hull1.reconstruct (alpha_shape1, polygons_alpha1);

  EXPECT_EQ (alpha_shape1.points.size (), 20);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers2 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha2;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull2;
  concave_hull2.setInputCloud (cloud2D);
  concave_hull2.setAlpha (0.01);
  concave_hull2.setVoronoiCenters (voronoi_centers2);
  concave_hull2.reconstruct (alpha_shape2, polygons_alpha2);

  EXPECT_EQ (alpha_shape2.points.size (), 81);

  //PolygonMesh concave;
  //toROSMsg (alpha_shape2, concave.cloud);
  //concave.polygons = polygons_alpha2;
  //saveVTKFile ("./test/bun0-concave2d.vtk", concave);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConcaveHull_LTable)
{
  //construct dataset
  pcl::PointCloud<pcl::PointXYZ> cloud_out_ltable;
  cloud_out_ltable.points.resize (100);

  int npoints = 0;
  for (size_t i = 0; i < 8; i++)
  {
    for (size_t j = 0; j <= 2; j++)
    {
      cloud_out_ltable.points[npoints].x = float (i) * 0.5f;
      cloud_out_ltable.points[npoints].y = -float (j) * 0.5f;
      cloud_out_ltable.points[npoints].z = 0.f;
      npoints++;
    }
  }

  for (size_t i = 0; i <= 2; i++)
  {
    for(size_t j = 3; j < 8; j++)
    {
      cloud_out_ltable.points[npoints].x = float (i) * 0.5f;
      cloud_out_ltable.points[npoints].y = -float (j) * 0.5f;
      cloud_out_ltable.points[npoints].z = 0.f;
      npoints++;
    }
  }

  cloud_out_ltable.points.resize (npoints);

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudptr (new pcl::PointCloud<pcl::PointXYZ> (cloud_out_ltable));

  pcl::PointCloud<pcl::PointXYZ> alpha_shape;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud (cloudptr);
  concave_hull.setAlpha (0.5);
  concave_hull.setVoronoiCenters (voronoi_centers);
  concave_hull.reconstruct (alpha_shape, polygons_alpha);

  EXPECT_EQ (alpha_shape.points.size (), 27);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers1 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha1;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull1;
  concave_hull1.setInputCloud (cloudptr);
  concave_hull1.setAlpha (1.5);
  concave_hull1.setVoronoiCenters (voronoi_centers1);
  concave_hull1.reconstruct (alpha_shape1, polygons_alpha1);

  EXPECT_EQ (alpha_shape1.points.size (), 23);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers2 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha2;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull2;
  concave_hull2.setInputCloud (cloudptr);
  concave_hull2.setAlpha (3);
  concave_hull2.setVoronoiCenters (voronoi_centers2);
  concave_hull2.reconstruct (alpha_shape2, polygons_alpha2);

  EXPECT_EQ (alpha_shape2.points.size (), 19);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, EarClipping)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
  cloud->height = 1;
  cloud->points.push_back (PointXYZ ( 0.f, 0.f, 0.5f));
  cloud->points.push_back (PointXYZ ( 5.f, 0.f, 0.6f));
  cloud->points.push_back (PointXYZ ( 9.f, 4.f, 0.5f));
  cloud->points.push_back (PointXYZ ( 4.f, 7.f, 0.5f));
  cloud->points.push_back (PointXYZ ( 2.f, 5.f, 0.5f));
  cloud->points.push_back (PointXYZ (-1.f, 8.f, 0.5f));
  cloud->width = static_cast<uint32_t> (cloud->points.size ());

  Vertices vertices;
  vertices.vertices.resize (cloud->points.size ());
  for (int i = 0; i < static_cast<int> (vertices.vertices.size ()); ++i)
    vertices.vertices[i] = i;

  PolygonMesh::Ptr mesh (new PolygonMesh);
  toROSMsg (*cloud, mesh->cloud);
  mesh->polygons.push_back (vertices);

  EarClipping clipper;
  PolygonMesh::ConstPtr mesh_aux (mesh);
  clipper.setInputMesh (mesh_aux);

  PolygonMesh triangulated_mesh;
  clipper.process (triangulated_mesh);

  EXPECT_EQ (triangulated_mesh.polygons.size (), 4);
  for (int i = 0; i < static_cast<int> (triangulated_mesh.polygons.size ()); ++i)
    EXPECT_EQ (triangulated_mesh.polygons[i].vertices.size (), 3);

  const int truth[][3] = { {5, 0, 1},
                           {2, 3, 4},
                           {4, 5, 1},
                           {1, 2, 4} };

  for (int pi = 0; pi < static_cast<int> (triangulated_mesh.polygons.size ()); ++pi)
  for (int vi = 0; vi < 3; ++vi)
  {
    EXPECT_EQ (triangulated_mesh.polygons[pi].vertices[vi], truth[pi][vi]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Poisson)
{
  Poisson<PointNormal> poisson;
  poisson.setInputCloud (cloud_with_normals);
  PolygonMesh mesh;
  poisson.reconstruct (mesh);


//  io::saveVTKFile ("bunny_poisson.vtk", mesh);

  ASSERT_EQ (mesh.polygons.size (), 1051);
  // All polygons should be triangles
  for (size_t i = 0; i < mesh.polygons.size (); ++i)
    EXPECT_EQ (mesh.polygons[i].vertices.size (), 3);

  EXPECT_EQ (mesh.polygons[10].vertices[0], 121);
  EXPECT_EQ (mesh.polygons[10].vertices[1], 120);
  EXPECT_EQ (mesh.polygons[10].vertices[2], 23);

  EXPECT_EQ (mesh.polygons[200].vertices[0], 130);
  EXPECT_EQ (mesh.polygons[200].vertices[1], 119);
  EXPECT_EQ (mesh.polygons[200].vertices[2], 131);

  EXPECT_EQ (mesh.polygons[1000].vertices[0], 521);
  EXPECT_EQ (mesh.polygons[1000].vertices[1], 516);
  EXPECT_EQ (mesh.polygons[1000].vertices[2], 517);
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
  sensor_msgs::PointCloud2 cloud_blob;
  loadPCDFile (argv[1], cloud_blob);
  fromROSMsg (cloud_blob, *cloud);

  // Create search tree
  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud);

  // Normal estimation
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  n.setInputCloud (cloud);
  //n.setIndices (indices[B);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate XYZ and normal information
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      
  // Create search tree
  tree2.reset (new search::KdTree<PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Process for update cloud
  if(argc == 3){
    sensor_msgs::PointCloud2 cloud_blob1;
    loadPCDFile (argv[2], cloud_blob1);
    fromROSMsg (cloud_blob1, *cloud1);
        // Create search tree
    tree3.reset (new search::KdTree<PointXYZ> (false));
    tree3->setInputCloud (cloud1);

    // Normal estimation
    NormalEstimation<PointXYZ, Normal> n1;
    PointCloud<Normal>::Ptr normals1 (new PointCloud<Normal> ());
    n1.setInputCloud (cloud1);

    n1.setSearchMethod (tree3);
    n1.setKSearch (20);
    n1.compute (*normals1);

    // Concatenate XYZ and normal information
    pcl::concatenateFields (*cloud1, *normals1, *cloud_with_normals1);
    // Create search tree
    tree4.reset (new search::KdTree<PointNormal>);
    tree4->setInputCloud (cloud_with_normals1);
  }

  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
