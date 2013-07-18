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
#include <pcl/surface/mls.h>
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

#ifdef _OPENMP
  // Testing OpenMP version
  MovingLeastSquaresOMP<PointXYZ, PointNormal> mls_omp;
  mls_omp.setInputCloud (cloud);
  mls_omp.setComputeNormals (true);
  mls_omp.setPolynomialFit (true);
  mls_omp.setSearchMethod (tree);
  mls_omp.setSearchRadius (0.03);
  mls_omp.setNumberOfThreads (4);

  // Reconstruct
  mls_normals->clear ();
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

#endif

  // Testing upsampling
  MovingLeastSquares<PointXYZ, PointNormal> mls_upsampling;
  // Set parameters
  mls_upsampling.setInputCloud (cloud);
  mls_upsampling.setComputeNormals (true);
  mls_upsampling.setPolynomialFit (true);
  mls_upsampling.setSearchMethod (tree);
  mls_upsampling.setSearchRadius (0.03);
  mls_upsampling.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
  mls_upsampling.setUpsamplingRadius (0.025);
  mls_upsampling.setUpsamplingStepSize (0.01);

  mls_normals->clear ();
  mls_upsampling.process (*mls_normals);

  EXPECT_NEAR (mls_normals->points[10].x, -0.000538, 1e-3);
  EXPECT_NEAR (mls_normals->points[10].y, 0.110080, 1e-3);
  EXPECT_NEAR (mls_normals->points[10].z, 0.043602, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[10].normal[0]), 0.022678, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[10].normal[1]), 0.554978, 1e-3);
  EXPECT_NEAR (fabs (mls_normals->points[10].normal[2]), 0.831556, 1e-3);
  EXPECT_NEAR (mls_normals->points[10].curvature, 0.012019, 1e-3);
  EXPECT_EQ (mls_normals->size (), 6352);


  /// TODO Would need to set a seed point here for the random number generator
  /// But as long as the other 2 upsampling methods work fine, this should have no issues.
  /// The RANDOM_UNIFORM_DENSITY upsampling will be changed soon anyway, hopefully in PCL 1.6.1
//  mls_upsampling.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::RANDOM_UNIFORM_DENSITY);
//  mls_upsampling.setPointDensity (100);
//  mls_normals->clear ();
//  mls_upsampling.process (*mls_normals);
//
//  EXPECT_NEAR (mls_normals->points[10].x, 0.018806, 1e-3);
//  EXPECT_NEAR (mls_normals->points[10].y, 0.114685, 1e-3);
//  EXPECT_NEAR (mls_normals->points[10].z, 0.037500, 1e-3);
//  EXPECT_NEAR (fabs (mls_normals->points[10].normal[0]), 0.351352, 1e-3);
//  EXPECT_NEAR (fabs (mls_normals->points[10].normal[1]), 0.537741, 1e-3);
//  EXPECT_NEAR (fabs (mls_normals->points[10].normal[2]), 0.766411, 1e-3);
//  EXPECT_NEAR (mls_normals->points[10].curvature, 0.019003, 1e-3);
//  EXPECT_EQ (mls_normals->size (), 457);


  mls_upsampling.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::VOXEL_GRID_DILATION);
  mls_upsampling.setDilationIterations (5);
  mls_upsampling.setDilationVoxelSize (0.005f);
  mls_normals->clear ();
  mls_upsampling.process (*mls_normals);
  EXPECT_NEAR (mls_normals->points[10].x, -0.070005938410758972, 2e-3);
  EXPECT_NEAR (mls_normals->points[10].y, 0.028887597844004631, 2e-3);
  EXPECT_NEAR (mls_normals->points[10].z, 0.01788550429046154, 2e-3);
  EXPECT_NEAR (mls_normals->points[10].curvature, 0.107273, 1e-1);
  EXPECT_NEAR (double (mls_normals->size ()), 29394, 2);
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
