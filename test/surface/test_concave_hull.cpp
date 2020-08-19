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
#include <pcl/surface/concave_hull.h>
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
TEST (PCL, ConcaveHull_bunny)
{
  //construct dataset
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D (new pcl::PointCloud<pcl::PointXYZ> (*cloud));
  for (auto &point : cloud2D->points)
    point.z = 0;

  pcl::PointCloud<pcl::PointXYZ> alpha_shape;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud (cloud2D);
  concave_hull.setAlpha (0.5);
  concave_hull.setVoronoiCenters (voronoi_centers);
  concave_hull.reconstruct (alpha_shape, polygons_alpha);

  EXPECT_EQ (alpha_shape.size (), 21);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers1 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha1;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull1;
  concave_hull1.setInputCloud (cloud2D);
  concave_hull1.setAlpha (1.5);
  concave_hull1.setVoronoiCenters (voronoi_centers1);
  concave_hull1.reconstruct (alpha_shape1, polygons_alpha1);

  EXPECT_EQ (alpha_shape1.size (), 20);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers2 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha2;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull2;
  concave_hull2.setInputCloud (cloud2D);
  concave_hull2.setAlpha (0.01);
  concave_hull2.setVoronoiCenters (voronoi_centers2);
  concave_hull2.reconstruct (alpha_shape2, polygons_alpha2);

  EXPECT_EQ (alpha_shape2.size (), 81);

  //PolygonMesh concave;
  //toPCLPointCloud2 (alpha_shape2, concave.cloud);
  //concave.polygons = polygons_alpha2;
  //saveVTKFile ("./test/bun0-concave2d.vtk", concave);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConcaveHull_planar_bunny)
{
  ConcaveHull<PointXYZ> concave_hull_3d;
  concave_hull_3d.setAlpha (0.5);
  concave_hull_3d.setInputCloud (cloud);
  PointCloud<PointXYZ> hull_3d;
  concave_hull_3d.reconstruct (hull_3d);

  EXPECT_EQ (concave_hull_3d.getDimension (), 3);

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

  ConcaveHull<PointXYZ> concave_hull_2d;
  concave_hull_2d.setAlpha (0.5);
  concave_hull_2d.setInputCloud (cloud_projected);
  PointCloud<PointXYZ> hull_2d;
  concave_hull_2d.reconstruct (hull_2d);

  EXPECT_EQ (concave_hull_2d.getDimension (), 2);
}


TEST (PCL, ConcaveHull_4points)
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

  ConcaveHull<PointXYZ> concave_hull;
  concave_hull.setInputCloud (cloud_4);
  concave_hull.setAlpha (10.);
  PolygonMesh mesh;
  concave_hull.reconstruct (mesh);

  EXPECT_EQ (mesh.polygons.size (), 1);
  EXPECT_EQ (mesh.polygons[0].vertices.size (), 4);

  PointCloud<PointXYZ> mesh_cloud;
  fromPCLPointCloud2 (mesh.cloud, mesh_cloud);

  EXPECT_NEAR (mesh_cloud[0].x, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[0].y, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[0].z, 0.f, 1e-6);

  EXPECT_NEAR (mesh_cloud[1].x, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[1].y, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[1].z, 0.f, 1e-6);

  EXPECT_NEAR (mesh_cloud[2].x, 0.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[2].y, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[2].z, 0.f, 1e-6);

  EXPECT_NEAR (mesh_cloud[3].x, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[3].y, 1.f, 1e-6);
  EXPECT_NEAR (mesh_cloud[3].z, 0.f, 1e-6);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ConcaveHull_LTable)
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
    for(std::size_t j = 3; j < 8; j++)
    {
      cloud_out_ltable[npoints].x = float (i) * 0.5f;
      cloud_out_ltable[npoints].y = -float (j) * 0.5f;
      cloud_out_ltable[npoints].z = 0.f;
      npoints++;
    }
  }

  cloud_out_ltable.resize (npoints);

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudptr (new pcl::PointCloud<pcl::PointXYZ> (cloud_out_ltable));

  pcl::PointCloud<pcl::PointXYZ> alpha_shape;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud (cloudptr);
  concave_hull.setAlpha (0.5);
  concave_hull.setVoronoiCenters (voronoi_centers);
  concave_hull.reconstruct (alpha_shape, polygons_alpha);

  EXPECT_EQ (alpha_shape.size (), 27);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers1 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha1;

  pcl::ConcaveHull<pcl::PointXYZ> concave_hull1;
  concave_hull1.setInputCloud (cloudptr);
  concave_hull1.setAlpha (1.5);
  concave_hull1.setVoronoiCenters (voronoi_centers1);
  concave_hull1.reconstruct (alpha_shape1, polygons_alpha1);

  EXPECT_EQ (alpha_shape1.size (), 23);

  pcl::PointCloud<pcl::PointXYZ> alpha_shape2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers2 (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons_alpha2;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull2;
  concave_hull2.setInputCloud (cloudptr);
  concave_hull2.setAlpha (3);
  concave_hull2.setVoronoiCenters (voronoi_centers2);
  concave_hull2.reconstruct (alpha_shape2, polygons_alpha2);

  EXPECT_EQ (alpha_shape2.size (), 19);
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
