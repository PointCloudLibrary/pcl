/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: surface_example.cpp 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
boost::shared_ptr<vector<int> > indices (new vector<int>);
KdTreePtr tree;

/* ---[ */
int
  main (int argc, char** argv)
{
  // Input
  assert(argc > 1);
  char* file_name = argv[1];
  sensor_msgs::PointCloud2 cloud_blob;
  loadPCDFile (file_name, cloud_blob);
  fromROSMsg (cloud_blob, *cloud);

  // Indices
  indices->resize (cloud->points.size ());
  for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }

  // KD-Tree
  tree.reset (new KdTreeFLANN<PointXYZ>);
  tree->setInputCloud (cloud);

  // Init objects
  PointCloud<PointXYZ> mls_points;
  PointCloud<Normal>::Ptr mls_normals (new PointCloud<Normal> ());
  MovingLeastSquares<PointXYZ, Normal> mls;

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setIndices (indices);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.setOutputNormals (mls_normals);
  mls.reconstruct (mls_points);
  
  // Save output
  PointCloud<PointNormal>::Ptr mls_cloud (new PointCloud<PointNormal> ());
  pcl::concatenateFields (mls_points, *mls_normals, *mls_cloud);
  savePCDFile ("./test/bun0-mls.pcd", *mls_cloud);

  // Test triangulation
  std::cerr << "TESTING TRIANGULATION" << std::endl;
  KdTree<PointNormal>::Ptr tree2 (new KdTreeFLANN<PointNormal>);
  tree2->setInputCloud (mls_cloud);
  PolygonMesh triangles;
  PolygonMesh grid;

  // Initialize object
  GreedyProjectionTriangulation<PointNormal> gp3;
  gp3.setSearchRadius (0.025);
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (mls_cloud);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  saveVTKFile ("./test/bun0-mls-triangles.vtk", triangles);
  //std::cerr << "INPUT: ./test/bun0-mls.pcd" << std::endl;
  std::cerr << "OUTPUT: ./test/bun0-mls-triangles.vtk" << std::endl;

  // Initialize object
  GridProjection<PointNormal> gp;
  gp.setResolution (0.005);
  gp.setPaddingSize (3);

  // Get result
  gp.setInputCloud (mls_cloud);
  gp.setSearchMethod (tree2);
  gp.reconstruct (grid);
  saveVTKFile ("./test/bun0-mls-grid.vtk", grid);
  //std::cerr << "INPUT: ./test/bun0-mls.pcd" << std::endl;
  std::cerr << "OUTPUT: ./test/bun0-mls-grid.vtk" << std::endl;

  // Finish
  return (0);
}
/* ]--- */
