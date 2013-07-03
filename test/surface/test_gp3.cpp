/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id: test_surface.cpp 6579 2012-07-27 18:57:32Z rusu $
 *
 */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>

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
  pcl::PCLPointCloud2 cloud_blob;
  loadPCDFile (argv[1], cloud_blob);
  fromPCLPointCloud2 (cloud_blob, *cloud);

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
    pcl::PCLPointCloud2 cloud_blob1;
    loadPCDFile (argv[2], cloud_blob1);
    fromPCLPointCloud2 (cloud_blob1, *cloud1);
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
