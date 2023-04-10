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
#include <pcl/features/regression_estimation.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud <pcl::PointXYZ>::Ptr cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Regression, FeatureExtraction)
{
  pcl::RegressionEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  float dot_product = 0.0f;
  Eigen::Vector3f major_vec, middle_vec, minor_vec;
  feature_extractor.getRegressionVectors (major_vec, middle_vec, minor_vec);
  dot_product = major_vec.dot (middle_vec);
  EXPECT_NEAR (0.0f, dot_product, 0.00001f);
  dot_product = major_vec.dot (minor_vec);
  EXPECT_NEAR (0.0f, dot_product, 0.00001f);
  dot_product = middle_vec.dot (minor_vec);
  EXPECT_NEAR (0.0f, dot_product, 0.00001f);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(Regression, InvalidParameters)
{
  bool essential = false;

  pcl::RegressionEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);

  essential = false;
  feature_extractor.setEssential(essential);
  essential = feature_extractor.getEssential();
  EXPECT_LT(false, essential);

  essential = true;
  feature_extractor.setEssential(essential);
  essential = feature_extractor.getEssential();
  EXPECT_LT(true, essential);

}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `lamppost.pcd` and pass its paths to the test." << std::endl;
    return (-1);
  }

  cloud.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `lamppost.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
