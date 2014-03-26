/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#include <gtest/gtest.h>

#include <pcl/pcl_tests.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

using namespace pcl;

typedef SampleConsensusModelLine<PointXYZ>::Ptr SampleConsensusModelLinePtr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelLine, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (10);

  cloud.points[0].getVector3fMap () <<  1.0f,  2.00f,  3.00f;
  cloud.points[1].getVector3fMap () <<  4.0f,  5.00f,  6.00f;
  cloud.points[2].getVector3fMap () <<  7.0f,  8.00f,  9.00f;
  cloud.points[3].getVector3fMap () << 10.0f, 11.00f, 12.00f;
  cloud.points[4].getVector3fMap () << 13.0f, 14.00f, 15.00f;
  cloud.points[5].getVector3fMap () << 16.0f, 17.00f, 18.00f;
  cloud.points[6].getVector3fMap () << 19.0f, 20.00f, 21.00f;
  cloud.points[7].getVector3fMap () << 22.0f, 23.00f, 24.00f;
  cloud.points[8].getVector3fMap () << -5.0f,  1.57f,  0.75f;
  cloud.points[9].getVector3fMap () <<  4.0f,  2.00f,  3.00f;

  // Create a shared line model pointer directly
  SampleConsensusModelLinePtr model (new SampleConsensusModelLine<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.001);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (2, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (8, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (6, coeff.size ());
  EXPECT_NEAR (1, coeff[4] / coeff[3], 1e-4);
  EXPECT_NEAR (1, coeff[5] / coeff[3], 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (6, coeff_refined.size ());
  EXPECT_NEAR (1, coeff[4] / coeff[3], 1e-4);
  EXPECT_NEAR (1, coeff[5] / coeff[3], 1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_XYZ_NEAR (PointXYZ ( 7.0,  8.0,  9.0), proj_points.points[2], 1e-4);
  EXPECT_XYZ_NEAR (PointXYZ (10.0, 11.0, 12.0), proj_points.points[3], 1e-4);
  EXPECT_XYZ_NEAR (PointXYZ (16.0, 17.0, 18.0), proj_points.points[5], 1e-4);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

