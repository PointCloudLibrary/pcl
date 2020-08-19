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

#include <random>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/model_types.h> // for SACMODEL_PLANE
#include <pcl/filters/project_inliers.h>

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
TEST (PCL, ConvexHull_bunny)
{
  pcl::PointCloud<pcl::PointXYZ> hull;
  std::vector<pcl::Vertices> polygons;

  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud);
  chull.reconstruct (hull, polygons);

  //PolygonMesh convex;
  //toPCLPointCloud2 (hull, convex.cloud);
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
  for (std::size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = polygons2[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (std::size_t j = 0; j < face1.vertices.size (); ++j)
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

  // convert the internal PCLPointCloud2 to a PointCloud
  PointCloud<pcl::PointXYZ> hull2;
  pcl::fromPCLPointCloud2 (mesh.cloud, hull2);

  // compare the PointCloud (hull2) to the output from the original test --- they should be identical
  ASSERT_EQ (hull.size (), hull2.size ());
  for (std::size_t i = 0; i < hull.size (); ++i)
  {
    const PointXYZ & p1 = hull[i];
    const PointXYZ & p2 = hull2[i];
    ASSERT_EQ (p1.x, p2.x);
    ASSERT_EQ (p1.y, p2.y);
    ASSERT_EQ (p1.z, p2.z);
  }

  // compare the face vertices (mesh.polygons) to the output from the original test --- they should be identical
  ASSERT_EQ (polygons.size (), mesh.polygons.size ());
  for (std::size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = mesh.polygons[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (std::size_t j = 0; j < face1.vertices.size (); ++j)
    {
      ASSERT_EQ (face1.vertices[j], face2.vertices[j]);
    }
  }    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_planar_bunny)
{
  ConvexHull<PointXYZ> convex_hull_3d;
  convex_hull_3d.setInputCloud (cloud);
  PointCloud<PointXYZ> hull_3d;
  convex_hull_3d.reconstruct (hull_3d);

  EXPECT_EQ (convex_hull_3d.getDimension (), 3);


  ModelCoefficients::Ptr plane_coefficients (new ModelCoefficients ());
  plane_coefficients->values.resize (4);
  plane_coefficients->values[0] = -0.010666f;
  plane_coefficients->values[1] = -0.793771f;
  plane_coefficients->values[2] = -0.607779f;
  plane_coefficients->values[3] = 0.993252f;

  /// Project segmented object points onto plane
  ProjectInliers<PointXYZ> project_inliers_filter;
  project_inliers_filter.setInputCloud (cloud);
  project_inliers_filter.setModelType (SACMODEL_PLANE);
  project_inliers_filter.setModelCoefficients (plane_coefficients);
  PointCloud<PointXYZ>::Ptr cloud_projected (new PointCloud<PointXYZ> ());
  project_inliers_filter.filter (*cloud_projected);

  ConvexHull<PointXYZ> convex_hull_2d;
  convex_hull_2d.setInputCloud (cloud_projected);
  PointCloud<PointXYZ> hull_2d;
  convex_hull_2d.reconstruct (hull_2d);

  EXPECT_EQ (convex_hull_2d.getDimension (), 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConvexHull_LTable)
{
  //construct dataset
  pcl::PointCloud<pcl::PointXYZ> cloud_out_ltable;
  cloud_out_ltable.resize (100);

  int npoints = 0;
  for (std::size_t i = 0; i < 8; i++)
  {
    for (std::size_t j = 0; j <= 2; j++)
    {
      cloud_out_ltable[npoints].x = float (i) * 0.5f;
      cloud_out_ltable[npoints].y = -float (j) * 0.5f;
      cloud_out_ltable[npoints].z = 0.f;
      npoints++;
    }
  }

  for (std::size_t i = 0; i <= 2; i++)
  {
    for (std::size_t j = 3; j < 8; j++)
    {
      cloud_out_ltable[npoints].x = float (i) * 0.5f;
      cloud_out_ltable[npoints].y = -float (j) * 0.5f;
      cloud_out_ltable[npoints].z = 0.f;
      npoints++;
    }
  }

  // add the five points on the hull
  cloud_out_ltable[npoints].x = -0.5f;
  cloud_out_ltable[npoints].y = 0.5f;
  cloud_out_ltable[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable[npoints].x = 4.5f;
  cloud_out_ltable[npoints].y = 0.5f;
  cloud_out_ltable[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable[npoints].x = 4.5f;
  cloud_out_ltable[npoints].y = -1.0f;
  cloud_out_ltable[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable[npoints].x = 1.0f;
  cloud_out_ltable[npoints].y = -4.5f;
  cloud_out_ltable[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable[npoints].x = -0.5f;
  cloud_out_ltable[npoints].y = -4.5f;
  cloud_out_ltable[npoints].z = 0.f;
  npoints++;

  cloud_out_ltable.resize (npoints);

  pcl::PointCloud<pcl::PointXYZ> hull;
  std::vector<pcl::Vertices> polygons;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudptr (new pcl::PointCloud<pcl::PointXYZ> (cloud_out_ltable));
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloudptr);
  chull.reconstruct (hull, polygons);

  EXPECT_EQ (polygons.size (), 1);
  EXPECT_EQ (hull.size (), 5);


  //
  // Test the face-vertices-only output variant
  //

  // construct the hull mesh
  std::vector<pcl::Vertices> polygons2;
  chull.reconstruct (polygons2);

  // compare the face vertices (polygons2) to the output from the original test --- they should be identical
  ASSERT_EQ (polygons.size (), polygons2.size ());
  for (std::size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = polygons2[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (std::size_t j = 0; j < face1.vertices.size (); ++j)
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

  // convert the internal PCLPointCloud2 to a PointCloud
  PointCloud<pcl::PointXYZ> hull2;
  pcl::fromPCLPointCloud2 (mesh.cloud, hull2);

  // compare the PointCloud (hull2) to the output from the original test --- they should be identical
  ASSERT_EQ (hull.size (), hull2.size ());
  for (std::size_t i = 0; i < hull.size (); ++i)
  {
    const PointXYZ & p1 = hull[i];
    const PointXYZ & p2 = hull2[i];
    ASSERT_EQ (p1.x, p2.x);
    ASSERT_EQ (p1.y, p2.y);
    ASSERT_EQ (p1.z, p2.z);
  }

  // compare the face vertices (mesh.polygons) to the output from the original test --- they should be identical
  ASSERT_EQ (polygons.size (), mesh.polygons.size ());
  for (std::size_t i = 0; i < polygons.size (); ++i)
  {
    const pcl::Vertices & face1 = polygons[i];
    const pcl::Vertices & face2 = mesh.polygons[i];
    ASSERT_EQ (face1.vertices.size (), face2.vertices.size ());
    for (std::size_t j = 0; j < face1.vertices.size (); ++j)
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
  std::mt19937 rng(12345u);
  std::uniform_real_distribution<float> rd (-1.0f, 1.0f);

  for (auto &point : input_cloud->points)
  {
    point.x = rd (rng);
    point.y = rd (rng);
    point.z = 1.0f;
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
  facets.emplace_back(-1.0, 0.0, 0.0, 1.0);
  facets.emplace_back(-1.0, 0.0, 0.0, -1.0);
  facets.emplace_back(0.0, -1.0, 0.0, 1.0);
  facets.emplace_back(0.0, -1.0, 0.0, -1.0);

  //Make sure they're in the plane
  for (const auto &point : hull.points)
  {
    float dist = std::abs (point.getVector4fMap ().dot (plane_normal));
    EXPECT_NEAR (dist, 0.0, 1e-2);

    float min_dist = std::numeric_limits<float>::infinity ();
    for (const auto &facet : facets)
    {
      float d2 = std::abs (point.getVector4fMap ().dot (facet));
      
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
  
  //rd
  std::mt19937 gen(12345u);
  std::uniform_real_distribution<float> rd (-1.0f, 1.0f);

  for (auto &point : input_cloud->points)
  {
    point.x = rd (gen);
    point.y = rd (gen);
    point.z = rd (gen);
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
  facets.emplace_back(-1.0f, 0.0f, 0.0f, 1.0f);
  facets.emplace_back(-1.0f, 0.0f, 0.0f, -1.0f);
  facets.emplace_back(0.0f, -1.0f, 0.0f, 1.0f);
  facets.emplace_back(0.0f, -1.0f, 0.0f, -1.0f);
  facets.emplace_back(0.0f, 0.0f, -1.0f, 1.0f);
  facets.emplace_back(0.0f, 0.0f, -1.0f, -1.0f);

  //Make sure they're near a facet
  for (const auto &point : hull.points)
  {
    float min_dist = std::numeric_limits<float>::infinity ();
    for (const auto &facet : facets)
    {
      float dist = std::abs (point.getVector4fMap ().dot (facet));
      
      if (dist < min_dist)
        min_dist = dist;
    }
    EXPECT_NEAR (min_dist, 0.0, 1e-2);
  }
}

TEST (PCL, ConvexHull_4points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_4 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ p;
  p.x = p.y = p.z = 0.f;
  cloud_4->push_back (p);

  p.x = 1.f;
  p.y = 0.f;
  p.z = 0.f;
  cloud_4->push_back (p);

  p.x = 0.f;
  p.y = 1.f;
  p.z = 0.f;
  cloud_4->push_back (p);

  p.x = 1.f;
  p.y = 1.f;
  p.z = 0.f;
  cloud_4->push_back (p);

  cloud_4->height = 1;
  cloud_4->width = cloud_4->size ();

  ConvexHull<PointXYZ> convex_hull;
  convex_hull.setComputeAreaVolume (true);
  convex_hull.setInputCloud (cloud_4);
  PolygonMesh mesh;
  convex_hull.reconstruct (mesh);

  EXPECT_EQ (mesh.polygons.size (), 1);

  /// TODO this should be 4, not 5 as it is now - fix that!!!
   EXPECT_EQ (mesh.polygons[0].vertices.size (), 4);

  PointCloud<PointXYZ> mesh_cloud;
  fromPCLPointCloud2 (mesh.cloud, mesh_cloud);

  EXPECT_NEAR (mesh_cloud[0].x, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[0].y, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[0].z, 0.f, 1e-6);

  EXPECT_NEAR (mesh_cloud[1].x, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[1].y, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[1].z, 0.f, 1e-6);

  EXPECT_NEAR (mesh_cloud[2].x, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[2].y, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[2].z, 0.f, 1e-6);

  EXPECT_NEAR (mesh_cloud[3].x, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[3].y, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[3].z, 0.f, 1e-6);

  EXPECT_NEAR (convex_hull.getTotalArea (), 1.0f, 1e-6);
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
  if (argc == 3)
  {
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
