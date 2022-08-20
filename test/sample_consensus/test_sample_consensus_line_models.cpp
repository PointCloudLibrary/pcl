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

#include <pcl/test/gtest.h>

#include <pcl/pcl_tests.h>

#include <pcl/common/common.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>

using namespace pcl;

using SampleConsensusModelLinePtr = SampleConsensusModelLine<PointXYZ>::Ptr;
using SampleConsensusModelParallelLinePtr = SampleConsensusModelParallelLine<PointXYZ>::Ptr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelLine, RANSAC)
{
  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.resize (10);

  cloud[0].getVector3fMap () <<  1.0f,  2.00f,  3.00f;
  cloud[1].getVector3fMap () <<  4.0f,  5.00f,  6.00f;
  cloud[2].getVector3fMap () <<  7.0f,  8.00f,  9.00f;
  cloud[3].getVector3fMap () << 10.0f, 11.00f, 12.00f;
  cloud[4].getVector3fMap () << 13.0f, 14.00f, 15.00f;
  cloud[5].getVector3fMap () << 16.0f, 17.00f, 18.00f;
  cloud[6].getVector3fMap () << 19.0f, 20.00f, 21.00f;
  cloud[7].getVector3fMap () << 22.0f, 23.00f, 24.00f;
  cloud[8].getVector3fMap () << -5.0f,  1.57f,  0.75f;
  cloud[9].getVector3fMap () <<  4.0f,  2.00f,  3.00f;

  // Create a shared line model pointer directly
  SampleConsensusModelLinePtr model (new SampleConsensusModelLine<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.001);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (2, sample.size ());

  pcl::Indices inliers;
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

  EXPECT_XYZ_NEAR (PointXYZ ( 7.0,  8.0,  9.0), proj_points[2], 1e-4);
  EXPECT_XYZ_NEAR (PointXYZ (10.0, 11.0, 12.0), proj_points[3], 1e-4);
  EXPECT_XYZ_NEAR (PointXYZ (16.0, 17.0, 18.0), proj_points[5], 1e-4);
}

TEST (SampleConsensusModelLine, OnGroundPlane)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (10);

  // All the points are on the ground plane (z=0).
  // The line is parallel to the x axis, so all the inlier points have the same z and y coordinates.
  cloud[0].getVector3fMap () <<  0.0f,  0.0f,  0.0f;
  cloud[1].getVector3fMap () <<  1.0f,  0.0f,  0.0f;
  cloud[2].getVector3fMap () <<  2.0f,  0.0f,  0.0f;
  cloud[3].getVector3fMap () <<  3.0f,  0.0f,  0.0f;
  cloud[4].getVector3fMap () <<  4.0f,  0.0f,  0.0f;
  cloud[5].getVector3fMap () <<  5.0f,  0.0f,  0.0f;
  // Outliers
  cloud[6].getVector3fMap () <<  2.1f,  2.0f,  0.0f;
  cloud[7].getVector3fMap () <<  5.0f,  4.1f,  0.0f;
  cloud[8].getVector3fMap () <<  0.4f,  1.3f,  0.0f;
  cloud[9].getVector3fMap () <<  3.3f,  0.1f,  0.0f;

  // Create a shared line model pointer directly
  SampleConsensusModelLinePtr model (new SampleConsensusModelLine<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.001);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (6, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (6, coeff.size ());

  EXPECT_NE (0, coeff[3]);
  EXPECT_NEAR (0, coeff[4], 1e-4);
  EXPECT_NEAR (0, coeff[5], 1e-4);
}

TEST (SampleConsensusModelLine, SampleValidationPointsEqual)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (3);

  // The "cheat point" makes it possible to find a set of valid samples and
  // therefore avoids the log message of an unsuccessful sample validation
  // being printed a 1000 times without any chance of success.
  // The order is chosen such that with a known, fixed rng-state/-seed all
  // validation steps are actually exercised.
  const pcl::index_t firstKnownEqualPoint = 0;
  const pcl::index_t secondKnownEqualPoint = 1;
  const pcl::index_t cheatPointIndex = 2;

  cloud[firstKnownEqualPoint].getVector3fMap () <<  0.1f,  0.0f,  0.0f;
  cloud[secondKnownEqualPoint].getVector3fMap () <<  0.1f,  0.0f,  0.0f;
  cloud[cheatPointIndex].getVector3fMap () <<  0.0f,  0.1f,  0.0f; // <-- cheat point

  // Create a shared line model pointer directly and explicitly disable the
  // random seed for the reasons mentioned above
  SampleConsensusModelLinePtr model (
    new SampleConsensusModelLine<PointXYZ> (cloud.makeShared (), /* random = */ false));

  // Algorithm tests
  pcl::Indices samples;
  int iterations = 0;
  model->getSamples(iterations, samples);
  EXPECT_EQ (samples.size(), 2);
  // The "cheat point" has to be part of the sample, otherwise something is wrong.
  // The best option would be to assert on stderr output here, but that doesn't
  // seem to be that simple.
  EXPECT_TRUE (std::find(samples.begin (), samples.end (), cheatPointIndex) != samples.end ());

  pcl::Indices forcedSamples = {firstKnownEqualPoint, secondKnownEqualPoint};
  Eigen::VectorXf modelCoefficients;
  EXPECT_FALSE (model->computeModelCoefficients (forcedSamples, modelCoefficients));
}

TEST (SampleConsensusModelLine, SampleValidationPointsValid)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (2);

  // These two points only differ in one coordinate so this also acts as a
  // regression test for 36c2bd6209f87dc7c6f56e2c0314e19f9cab95ec
  cloud[0].getVector3fMap () <<  0.0f,  0.0f,  0.0f;
  cloud[1].getVector3fMap () <<  0.1f,  0.0f,  0.0f;

  // Create a shared line model pointer directly
  SampleConsensusModelLinePtr model (new SampleConsensusModelLine<PointXYZ> (cloud.makeShared ()));

  // Algorithm tests
  pcl::Indices samples;
  int iterations = 0;
  model->getSamples(iterations, samples);
  EXPECT_EQ (samples.size(), 2);

  pcl::Indices forcedSamples = {0, 1};
  Eigen::VectorXf modelCoefficients;
  EXPECT_TRUE (model->computeModelCoefficients (forcedSamples, modelCoefficients));
}

TEST (SampleConsensusModelLine, SampleValidationNotEnoughSamples)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (1);

  cloud[0].getVector3fMap () <<  0.1f,  0.0f,  0.0f;

  // Create a shared line model pointer directly
  SampleConsensusModelLinePtr model (new SampleConsensusModelLine<PointXYZ> (cloud.makeShared ()));

  // Algorithm tests
  pcl::Indices samples;
  int iterations = 0;
  model->getSamples(iterations, samples);
  EXPECT_EQ (samples.size(), 0);

  pcl::Indices forcedSamples = {0,};
  Eigen::VectorXf modelCoefficients;
  EXPECT_FALSE (model->computeModelCoefficients (forcedSamples, modelCoefficients));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelParallelLine, RANSAC)
{
  PointCloud<PointXYZ> cloud (16, 1);

  // Line 1
  cloud[0].getVector3fMap () <<  1.0f,  2.00f,  3.00f;
  cloud[1].getVector3fMap () <<  4.0f,  5.00f,  6.00f;
  cloud[2].getVector3fMap () <<  7.0f,  8.00f,  9.00f;
  cloud[3].getVector3fMap () << 10.0f, 11.00f, 12.00f;
  cloud[4].getVector3fMap () << 13.0f, 14.00f, 15.00f;
  cloud[5].getVector3fMap () << 16.0f, 17.00f, 18.00f;
  cloud[6].getVector3fMap () << 19.0f, 20.00f, 21.00f;
  cloud[7].getVector3fMap () << 22.0f, 23.00f, 24.00f;
  // Random points
  cloud[8].getVector3fMap () << -5.0f,  1.57f,  0.75f;
  cloud[9].getVector3fMap () <<  4.0f,  2.00f,  3.00f;
  // Line 2 (parallel to the Z axis)
  cloud[10].getVector3fMap () << -1.00f,  5.00f,  0.0f;
  cloud[11].getVector3fMap () << -1.05f,  5.01f,  1.0f;
  cloud[12].getVector3fMap () << -1.01f,  5.05f,  2.0f;
  cloud[13].getVector3fMap () << -1.05f,  5.01f,  3.0f;
  cloud[14].getVector3fMap () << -1.01f,  5.05f,  4.0f;
  cloud[15].getVector3fMap () << -1.05f,  5.01f,  5.0f;

  // Create a shared line model pointer directly
  const double eps = 0.1; //angle eps in radians
  const Eigen::Vector3f axis (0, 0, 1);
  SampleConsensusModelParallelLinePtr model (new SampleConsensusModelParallelLine<PointXYZ> (cloud.makeShared ()));
  model->setAxis (axis);
  model->setEpsAngle (eps);

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.1);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (2, sample.size ());

  pcl::Indices inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (6, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (6, coeff.size ());

  // Make sure the returned direction respects the angular constraint
  double angle_diff = getAngle3D (axis, coeff.tail<3> ());
  angle_diff = std::min (angle_diff, M_PI - angle_diff);
  EXPECT_GT (eps, angle_diff);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff, proj_points);

  EXPECT_XYZ_NEAR (PointXYZ (-1.05, 5.05, 3.0), proj_points[13], 0.1);
  EXPECT_XYZ_NEAR (PointXYZ (-1.05, 5.05, 4.0), proj_points[14], 0.1);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

