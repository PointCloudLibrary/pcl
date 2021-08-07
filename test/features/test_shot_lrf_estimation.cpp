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
#include <pcl/pcl_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_lrf.h>

using namespace pcl;
using namespace pcl::test;
using namespace pcl::io;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
pcl::Indices indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTLocalReferenceFrameEstimation)
{
  PointCloud<ReferenceFrame> bunny_LRF;

  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));

  // Compute SHOT LRF
  SHOTLocalReferenceFrameEstimation<PointXYZ, ReferenceFrame> lrf_estimator;

  float radius = 0.01f;

  lrf_estimator.setRadiusSearch (radius);

  lrf_estimator.setInputCloud (cloud.makeShared ());
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setIndices (indicesptr);

  lrf_estimator.compute (bunny_LRF);

  // TESTS
  EXPECT_EQ (indices.size (), bunny_LRF.size ());

  EXPECT_FALSE (bunny_LRF.is_dense);

  // NaN result for point 24
  //EXPECT_EQ (std::numeric_limits<float>::max (), bunny_LRF.at (24).confidence);
  EXPECT_TRUE (std::isnan (bunny_LRF.at (24).x_axis[0]));

  // Expected Results
  // point 15: tangent disambiguation
  //float point_15_conf = 0;
  Eigen::Vector3f point_15_x (-0.849213f, 0.528016f, 0.00593846f);
  Eigen::Vector3f point_15_y (0.274564f, 0.451135f, -0.849171f);
  Eigen::Vector3f point_15_z (-0.451055f, -0.719497f, -0.528084f);

  //float point_45_conf = 0;
  Eigen::Vector3f point_45_x (0.950556f, 0.0673042f, 0.303171f);
  Eigen::Vector3f point_45_y (0.156242f, -0.947328f, -0.279569f);
  Eigen::Vector3f point_45_z (0.268386f, 0.313114f, -0.911004f);

  //float point_163_conf = 0;
  Eigen::Vector3f point_163_x (0.816369f, 0.309943f, -0.487317f);
  Eigen::Vector3f point_163_y (0.235273f, -0.949082f, -0.209498f);
  Eigen::Vector3f point_163_z (-0.527436f, 0.0563754f, -0.847722f);

  // point 311: normal disambiguation
  //float point_311_conf = 0;
  Eigen::Vector3f point_311_x (0.77608663f, -0.60673451f, 0.17193851f);
  Eigen::Vector3f point_311_y (0.49546647f, 0.75532055f, 0.42895663f);
  Eigen::Vector3f point_311_z (-0.39013144f, -0.24771771f, 0.88681078f);

  //Test Results
  //EXPECT_NEAR (point_15_conf,bunny_LRF.at (15).confidence, 1E-3);
  for (int d = 0; d < 3; ++d)
  {
    EXPECT_NEAR (point_15_x[d], bunny_LRF.at (15).x_axis[d], 1E-3);
    EXPECT_NEAR (point_15_y[d], bunny_LRF.at (15).y_axis[d], 1E-3);
    EXPECT_NEAR (point_15_z[d], bunny_LRF.at (15).z_axis[d], 1E-3);

    EXPECT_NEAR (point_45_x[d], bunny_LRF.at (45).x_axis[d], 1E-3);
    EXPECT_NEAR (point_45_y[d], bunny_LRF.at (45).y_axis[d], 1E-3);
    EXPECT_NEAR (point_45_z[d], bunny_LRF.at (45).z_axis[d], 1E-3);

    EXPECT_NEAR (point_163_x[d], bunny_LRF.at (163).x_axis[d], 1E-3);
    EXPECT_NEAR (point_163_y[d], bunny_LRF.at (163).y_axis[d], 1E-3);
    EXPECT_NEAR (point_163_z[d], bunny_LRF.at (163).z_axis[d], 1E-3);

    EXPECT_NEAR (point_311_x[d], bunny_LRF.at (311).x_axis[d], 1E-3);
    EXPECT_NEAR (point_311_y[d], bunny_LRF.at (311).y_axis[d], 1E-3);
    EXPECT_NEAR (point_311_z[d], bunny_LRF.at (311).z_axis[d], 1E-3);
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

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.size ());
  for (std::size_t i = 0; i < indices.size (); ++i)
    indices[i] = static_cast<int> (i);

  tree.reset (new search::KdTree<PointXYZ> (true));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
