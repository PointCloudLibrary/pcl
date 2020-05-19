/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: test_segmentation.cpp 3564 2011-12-16 06:11:13Z rusu $
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace pcl;
using namespace pcl::io;

PointCloud<PointXYZ>::Ptr cloud_;

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (SACSegmentation, Segmentation)
{
  ModelCoefficients::Ptr sphere_coefficients (new ModelCoefficients);
  PointIndices::Ptr inliers (new PointIndices);

  SACSegmentation<PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_SPHERE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.01);
  seg.setRadiusLimits (0.03, 0.07); // true radius is 0.05
  seg.setInputCloud (cloud_);
  seg.segment (*inliers, *sphere_coefficients);

  EXPECT_NEAR (sphere_coefficients->values[0], 0.998776,  1e-2);
  EXPECT_NEAR (sphere_coefficients->values[1], 0.752023,  1e-2);
  EXPECT_NEAR (sphere_coefficients->values[2], 1.24558,   1e-2);
  EXPECT_NEAR (sphere_coefficients->values[3], 0.0536238, 1e-2);

  EXPECT_NEAR (static_cast<int> (inliers->indices.size ()), 3516, 15);
}

//* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `noisy_slice_displaced.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load a standard PCD file from disk
  PointCloud<PointXYZ> cloud;
  if (loadPCDFile (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `noisy_slice_displaced.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  cloud_   = cloud.makeShared ();

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
