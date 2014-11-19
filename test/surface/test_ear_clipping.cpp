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
#include <pcl/surface/ear_clipping.h>
#include <pcl/common/common.h>

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
  toPCLPointCloud2 (*cloud, mesh->cloud);
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

TEST (PCL, EarClippingCubeTest)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
  cloud->height = 1;
  //bottom of cube (z=0)
  cloud->points.push_back (PointXYZ ( 0.f, 0.f, 0.f));
  cloud->points.push_back (PointXYZ ( 1.f, 0.f, 0.f));
  cloud->points.push_back (PointXYZ ( 1.f, 1.f, 0.f));
  cloud->points.push_back (PointXYZ ( 0.f, 1.f, 0.f));
  //top of cube (z=1.0)
  cloud->points.push_back (PointXYZ ( 0.f, 0.f, 1.f));
  cloud->points.push_back (PointXYZ ( 1.f, 0.f, 1.f));
  cloud->points.push_back (PointXYZ ( 1.f, 1.f, 1.f));
  cloud->points.push_back (PointXYZ ( 0.f, 1.f, 1.f));
  cloud->width = static_cast<uint32_t> (cloud->points.size ());

  Vertices vertices;
  vertices.vertices.resize(4);

  const int squares[][4] = { {1, 5, 6, 2},
                           {2, 6, 7, 3},
                           {3, 7, 4, 0},
                           {0, 4, 5, 1},
                           {4, 7, 6, 5},       
                           {0, 1, 2, 3} };

  const int truth[][3] = { {2, 1, 5},
                           {6, 2, 5},
                           {3, 2, 6}, 
                           {7, 3, 6},
                           {0, 3, 7}, 
                           {4, 0, 7},
                           {1, 0, 4}, 
                           {5, 1, 4},
                           {5, 4, 7}, 
                           {6, 5, 7},       
                           {3, 0, 1}, 
                           {2, 3, 1} };

  PolygonMesh::Ptr mesh (new PolygonMesh);
  toPCLPointCloud2 (*cloud, mesh->cloud);

  for (int i = 0; i < 6; ++i)
  {
    vertices.vertices[0] = squares[i][0];
    vertices.vertices[1] = squares[i][1];
    vertices.vertices[2] = squares[i][2];
    vertices.vertices[3] = squares[i][3];
    mesh->polygons.push_back (vertices);
  }

  EarClipping clipper;
  PolygonMesh::ConstPtr mesh_aux (mesh);
  clipper.setInputMesh (mesh_aux);

  PolygonMesh triangulated_mesh;
  clipper.process (triangulated_mesh);

  EXPECT_EQ (triangulated_mesh.polygons.size (), 12);
  for (int i = 0; i < static_cast<int> (triangulated_mesh.polygons.size ()); ++i)
    EXPECT_EQ (triangulated_mesh.polygons[i].vertices.size (), 3);

  

  for (int pi = 0; pi < static_cast<int> (triangulated_mesh.polygons.size ()); ++pi)
  {
      for (int vi = 0; vi < 3; ++vi)
      {
        EXPECT_EQ (triangulated_mesh.polygons[pi].vertices[vi], truth[pi][vi]);
      }
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
