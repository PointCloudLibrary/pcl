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
//#include <pcl/features/normal_3d.h>
#include <pcl/surface/ball_pivoting.h>
#include <pcl/common/common.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

/*PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree;
search::KdTree<PointNormal>::Ptr tree2;

// add by ktran to test update functions
PointCloud<PointXYZ>::Ptr cloud1 (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals1 (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree3;
search::KdTree<PointNormal>::Ptr tree4;*/

// custom cloud and tree
PointCloud<PointNormal>::Ptr cloud_custom (new PointCloud<PointNormal>);
search::KdTree<PointNormal>::Ptr tree_custom;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BallPivotingTest)
{
  pcl::PolygonMesh mesh_fixed_radius;
  pcl::PolygonMesh mesh_found_radius;

  BallPivoting<PointNormal> pivoter;
  pivoter.setInputCloud (cloud_custom);
  pivoter.setSearchMethod (tree_custom);

  pivoter.setSearchRadius (0.02);
  pivoter.reconstruct (mesh_fixed_radius);

  pivoter.setEstimatedRadius (100, 5, 0.95f); 
  pivoter.reconstruct (mesh_found_radius);

  const std::vector<Vertices> &polygons_fixed_radius = mesh_fixed_radius.polygons;
  EXPECT_EQ (polygons_fixed_radius[100].vertices[0], 99);
  EXPECT_EQ (polygons_fixed_radius[100].vertices[1], 102);
  EXPECT_EQ (polygons_fixed_radius[100].vertices[2], 101);
  EXPECT_EQ (polygons_fixed_radius[200].vertices[0], 199);
  EXPECT_EQ (polygons_fixed_radius[200].vertices[1], 202);
  EXPECT_EQ (polygons_fixed_radius[200].vertices[2], 201);

  const std::vector<Vertices> &polygons_found_radius = mesh_found_radius.polygons;
  EXPECT_EQ (polygons_found_radius[100].vertices[0], 99);
  EXPECT_EQ (polygons_found_radius[100].vertices[1], 102);
  EXPECT_EQ (polygons_found_radius[100].vertices[2], 101);
  EXPECT_EQ (polygons_found_radius[200].vertices[0], 199);
  EXPECT_EQ (polygons_found_radius[200].vertices[1], 202);
  EXPECT_EQ (polygons_found_radius[200].vertices[2], 201);
}

/**
 * return a point cloud similar to triangle strip in computer graphics, 
 * it generally extends to +X direction, with normal in +Z and points on Z=0.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr
compose_strip_cloud (const size_t size_cloud)
{
  const Eigen::Vector3f normal (0.0f, 0.0f, 1.0f);
  const Eigen::Vector3f odd_shift = Eigen::Vector3f (0.5f, std::sqrt (3.0f) / 2.0f, 0.0f) / 100.0f;
  const Eigen::Vector3f even_shift = Eigen::Vector3f (1.0f, 0.0f, 0.0f) / 100.0f;
  Eigen::Vector3f base = Eigen::Vector3f::Zero ();

  pcl::PointCloud<pcl::PointNormal>::Ptr re (
      new pcl::PointCloud<pcl::PointNormal> ());
  re->reserve (size_cloud);

  for (size_t id_point = 0; id_point < size_cloud; ++id_point)
  {
    Eigen::Vector3f position = base;
    pcl::PointNormal point;
    if (id_point % 2 == 1)
    {
      // 1,3,5,7,9...-th points
      position += odd_shift;
    }
    else
    {
      // 0,2,4,6,8...-th points
      base += even_shift;
    }
    point.getVector3fMap () = position;
    point.getNormalVector3fMap () = normal;
    re->push_back (point);
  }

  return re;
}

/* ---[ */
int
main (int argc, char** argv)
{
/*  if (argc < 2)
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
  }*/

  // use composed point cloud for unit test
  cloud_custom= compose_strip_cloud (1000);
  tree_custom.reset (new search::KdTree<PointNormal>);
  tree_custom->setInputCloud (cloud_custom);

  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
