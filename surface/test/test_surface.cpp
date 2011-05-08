/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id$
 *
 */
/** \author Zoltan-Csaba Marton */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/common.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
//boost::shared_ptr<vector<int> > indices (new vector<int>);
KdTree<PointXYZ>::Ptr tree;
KdTree<PointNormal>::Ptr tree2;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, MovingLeastSquares)
{
  // Init objects
  PointCloud<PointXYZ> mls_points;
  PointCloud<Normal>::Ptr mls_normals (new PointCloud<Normal> ());
  MovingLeastSquares<PointXYZ, Normal> mls;

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setOutputNormals (mls_normals);
  //mls.setIndices (indices);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.reconstruct (mls_points);
  EXPECT_NEAR (mls_points.points[0].x, 0.005, 1e-3);
  EXPECT_NEAR (mls_points.points[0].y, 0.111, 1e-3);
  EXPECT_NEAR (mls_points.points[0].z, 0.038, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[0].normal[0]), 0.1176, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[0].normal[1]), 0.6193, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[0].normal[2]), 0.7762, 1e-3);
  EXPECT_NEAR (mls_normals->points[0].curvature, 0.012, 1e-3);
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
  gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Reconstruct
  gp3.reconstruct (triangles);
  //saveVTKFile ("./test/bun0-gp3.vtk", triangles);
  EXPECT_EQ (triangles.cloud.width, cloud_with_normals->width);
  EXPECT_EQ (triangles.cloud.height, cloud_with_normals->height);
  EXPECT_NEAR ((int)triangles.polygons.size(), 685, 5);

  // Check triangles
  EXPECT_EQ ((int)triangles.polygons.at(0).vertices.size(), 3);
  EXPECT_EQ ((int)triangles.polygons.at(0).vertices.at(0), 0);
  EXPECT_EQ ((int)triangles.polygons.at(0).vertices.at(1), 12);
  EXPECT_EQ ((int)triangles.polygons.at(0).vertices.at(2), 198);
  EXPECT_EQ ((int)triangles.polygons.at(684).vertices.size(), 3);
  EXPECT_EQ ((int)triangles.polygons.at(684).vertices.at(0), 393);
  EXPECT_EQ ((int)triangles.polygons.at(684).vertices.at(1), 394);
  EXPECT_EQ ((int)triangles.polygons.at(684).vertices.at(2), 395);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  int nr_points = cloud_with_normals->width * cloud_with_normals->height;
  EXPECT_EQ ((int)parts.size (), nr_points);
  EXPECT_EQ ((int)states.size (), nr_points);
  EXPECT_EQ (parts[0], 0);
  EXPECT_EQ (states[0], gp3.COMPLETED);
  EXPECT_EQ (parts[393], 5);
  EXPECT_EQ (states[393], gp3.BOUNDARY);
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
  EXPECT_GE ((int)grid.polygons.size(), 1295);
  EXPECT_EQ ((int)grid.polygons.at(0).vertices.size(), 4);
  EXPECT_EQ ((int)grid.polygons.at(0).vertices.at(0), 0);
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
      cloud_out_ltable.points[npoints].x = (double)(i)*0.5;
      cloud_out_ltable.points[npoints].y = -(double)(j)*0.5;
      cloud_out_ltable.points[npoints].z = 0;
      npoints++;
    }
  }

  for (size_t i = 0; i <= 2; i++) 
  {
    for (size_t j = 3; j < 8; j++) 
    {
      cloud_out_ltable.points[npoints].x = (double)(i)*0.5;
      cloud_out_ltable.points[npoints].y = -(double)(j)*0.5;
      cloud_out_ltable.points[npoints].z = 0;
      npoints++;
    }
  }

  // add the five points on the hull
  cloud_out_ltable.points[npoints].x = -0.5;
  cloud_out_ltable.points[npoints].y = 0.5;
  cloud_out_ltable.points[npoints].z = 0;
  npoints++;
	
  cloud_out_ltable.points[npoints].x = 4.5;
  cloud_out_ltable.points[npoints].y = 0.5;
  cloud_out_ltable.points[npoints].z = 0;
  npoints++;
	
  cloud_out_ltable.points[npoints].x = 4.5;
  cloud_out_ltable.points[npoints].y = -1.0;
  cloud_out_ltable.points[npoints].z = 0;
  npoints++;
  
  cloud_out_ltable.points[npoints].x = 1.0;
  cloud_out_ltable.points[npoints].y = -4.5;
  cloud_out_ltable.points[npoints].z = 0;
  npoints++;
  
  cloud_out_ltable.points[npoints].x = -0.5;
  cloud_out_ltable.points[npoints].y = -4.5;
  cloud_out_ltable.points[npoints].z = 0;
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
      cloud_out_ltable.points[npoints].x = (double)(i)*0.5;
      cloud_out_ltable.points[npoints].y = -(double)(j)*0.5;
      cloud_out_ltable.points[npoints].z = 0;
      npoints++;
    }
  }

  for (size_t i = 0; i <= 2; i++) 
  {
    for(size_t j = 3; j < 8; j++) 
    {
      cloud_out_ltable.points[npoints].x = (double)(i)*0.5;
      cloud_out_ltable.points[npoints].y = -(double)(j)*0.5;
      cloud_out_ltable.points[npoints].z = 0;
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

  // Set up dummy indices
  //indices->resize (cloud->points.size ());
  //for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }

  // Create search tree
  tree.reset (new KdTreeFLANN<PointXYZ> (false));
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
  tree2.reset (new KdTreeFLANN<PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
