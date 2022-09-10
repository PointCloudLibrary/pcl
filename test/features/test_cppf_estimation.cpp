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
#include <pcl/features/cppf.h>
#include <pcl/io/pcd_io.h>

using PointT = pcl::PointXYZRGBNormal;
static pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CPPFEstimation)
{
  pcl::CPPFEstimation <PointT, PointT, pcl::CPPFSignature> cppf_estimation;
  cppf_estimation.setInputCloud (cloud);
  cppf_estimation.setInputNormals (cloud);
  pcl::PointCloud<pcl::CPPFSignature>::Ptr feature_cloud (new pcl::PointCloud<pcl::CPPFSignature> ());
  cppf_estimation.compute (*feature_cloud);

  // Check for size of output
  EXPECT_EQ (feature_cloud->size (), cloud->size () * cloud->size ());

  // Now check for a few values in the feature cloud
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f1));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f2));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f3));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f4));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f5));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f6));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f7));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f8));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f9));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].f10));
  EXPECT_TRUE (std::isnan ((*feature_cloud)[0].alpha_m));

  EXPECT_NEAR ((*feature_cloud)[2572].f1, 0.0568356, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f2, -0.1988939, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f3, 0.7854938, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f4, 0.0533117, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f5, 0.1875000, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f6, 0.0733944, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f7, 0.4274509, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f8, 0.2380952, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f9, 0.0619469, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].f10, 0.4431372, 1e-4);
  EXPECT_NEAR ((*feature_cloud)[2572].alpha_m, -1.847514, 1e-4);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `colored_cloud.pcd` and pass its path to the test." << std::endl;
    return (1);
  }

  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `colored_cloud.pcd` and pass its path to the test." << std::endl;
    return (1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
