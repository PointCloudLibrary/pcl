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

#include <pcl/test/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>

using namespace pcl;
using namespace pcl::io;

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
TEST (PCL, Organized)
{
  //construct dataset
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_organized (new pcl::PointCloud<pcl::PointXYZ> ());
  cloud_organized->width = 5;
  cloud_organized->height = 10;
  cloud_organized->points.resize (cloud_organized->width * cloud_organized->height);

  int npoints = 0;
  for (std::size_t i = 0; i < cloud_organized->height; i++)
  {
    for (std::size_t j = 0; j < cloud_organized->width; j++)
    {
      (*cloud_organized)[npoints].x = static_cast<float> (i);
      (*cloud_organized)[npoints].y = static_cast<float> (j);
      (*cloud_organized)[npoints].z = static_cast<float> (cloud_organized->size ()); // to avoid shadowing
      npoints++;
    }
  }
  int nan_idx = cloud_organized->width*cloud_organized->height - 2*cloud_organized->width + 1;
  (*cloud_organized)[nan_idx].x = std::numeric_limits<float>::quiet_NaN ();
  (*cloud_organized)[nan_idx].y = std::numeric_limits<float>::quiet_NaN ();
  (*cloud_organized)[nan_idx].z = std::numeric_limits<float>::quiet_NaN ();
  
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
