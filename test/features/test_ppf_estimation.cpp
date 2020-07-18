/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
pcl::Indices indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PPFEstimation)
{
  // Estimate normals
  NormalEstimation<PointXYZ, Normal> normal_estimation;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  normal_estimation.setInputCloud (cloud.makeShared ());
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  normal_estimation.setIndices (indicesptr);
  normal_estimation.setSearchMethod (tree);
  normal_estimation.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  normal_estimation.compute (*normals);

  PPFEstimation <PointXYZ, Normal, PPFSignature> ppf_estimation;
  ppf_estimation.setInputCloud (cloud.makeShared ());
  ppf_estimation.setInputNormals (normals);
  PointCloud<PPFSignature>::Ptr feature_cloud (new PointCloud<PPFSignature> ());
  ppf_estimation.compute (*feature_cloud);

  // Check for size of output
  EXPECT_EQ (feature_cloud->size (), indices.size () * cloud.size ());

  // Now check for a few values in the feature cloud
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f1));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f2));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f3));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f4));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].alpha_m));

  EXPECT_NEAR ((*feature_cloud)[15127].f1, -2.51637, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[15127].f2, -0.00365916, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[15127].f3, -0.521141, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[15127].f4, 0.0106809, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[15127].alpha_m, -0.255664, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[30254].f1, 0.185142, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[30254].f2, 0.0425001, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[30254].f3, -0.191276, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[30254].f4, 0.0138508, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[30254].alpha_m, 2.42955, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[45381].f1, -1.96263, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[45381].f2, -0.431919, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[45381].f3, 0.868716, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[45381].f4, 0.140129, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[45381].alpha_m, -1.97276, 1e-4);
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

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.size ());
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
    indices[i] = i;

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
