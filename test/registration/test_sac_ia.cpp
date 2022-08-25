/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2011, Willow Garage, Inc.
*  Copyright (c) 2018-, Open Perception, Inc.
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

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>

using namespace pcl;
using namespace pcl::io;

PointCloud<PointXYZ> cloud_source, cloud_target, cloud_reg;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SampleConsensusInitialAlignment)
{
  // Transform the source cloud by a large amount
  Eigen::Vector3f initial_offset (100, 0, 0);
  float angle = static_cast<float> (M_PI) / 2.0f;
  Eigen::Quaternionf initial_rotation (std::cos (angle / 2), 0, 0, std::sin (angle / 2));
  PointCloud<PointXYZ> cloud_source_transformed;
  transformPointCloud (cloud_source, cloud_source_transformed, initial_offset, initial_rotation);

  // Create shared pointers
  PointCloud<PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
  cloud_source_ptr = cloud_source_transformed.makeShared ();
  cloud_target_ptr = cloud_target.makeShared ();

  // Initialize estimators for surface normals and FPFH features
  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);

  NormalEstimation<PointXYZ, Normal> norm_est;
  norm_est.setSearchMethod (tree);
  norm_est.setRadiusSearch (0.05);
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);

  FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
  fpfh_est.setSearchMethod (tree);
  fpfh_est.setRadiusSearch (0.05);
  PointCloud<FPFHSignature33>::Ptr features_source(new PointCloud<FPFHSignature33>), features_target(new PointCloud<FPFHSignature33>);

  // Estimate the FPFH features for the source cloud
  norm_est.setInputCloud (cloud_source_ptr);
  norm_est.compute (*normals);
  fpfh_est.setInputCloud (cloud_source_ptr);
  fpfh_est.setInputNormals (normals);
  fpfh_est.compute (*features_source);

  // Estimate the FPFH features for the target cloud
  norm_est.setInputCloud (cloud_target_ptr);
  norm_est.compute (*normals);
  fpfh_est.setInputCloud (cloud_target_ptr);
  fpfh_est.setInputNormals (normals);
  fpfh_est.compute (*features_target);

  // Initialize Sample Consensus Initial Alignment (SAC-IA)
  SampleConsensusInitialAlignment<PointXYZ, PointXYZ, FPFHSignature33> reg;
  reg.setMinSampleDistance (0.05f);
  reg.setMaxCorrespondenceDistance (0.1);
  reg.setMaximumIterations (1000);

  reg.setInputSource (cloud_source_ptr);
  reg.setInputTarget (cloud_target_ptr);
  reg.setSourceFeatures (features_source);
  reg.setTargetFeatures (features_target);

  // Register
  reg.align (cloud_reg);
  EXPECT_EQ (cloud_reg.size (), cloud_source.size ());
  EXPECT_LT (reg.getFitnessScore (), 0.0005);

  // Check again, for all possible caching schemes
  using PointT = pcl::PointXYZ;
  for (int iter = 0; iter < 4; iter++)
  {
    bool force_cache = static_cast<bool> (iter/2);
    bool force_cache_reciprocal = static_cast<bool> (iter%2);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // Ensure that, when force_cache is not set, we are robust to the wrong input
    if (force_cache)
      tree->setInputCloud (cloud_target_ptr);
    reg.setSearchMethodTarget (tree, force_cache);

    pcl::search::KdTree<PointT>::Ptr tree_recip (new pcl::search::KdTree<PointT>);
    if (force_cache_reciprocal)
      tree_recip->setInputCloud (cloud_source_ptr);
    reg.setSearchMethodSource(tree_recip, force_cache_reciprocal);

    // Register
    reg.align (cloud_reg);
    EXPECT_EQ (cloud_reg.size (), cloud_source.size ());
    EXPECT_LT (reg.getFitnessScore (), 0.0005);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SampleConsensusPrerejective)
{
  /*
   * This test is a near-exact copy of the SampleConsensusInitialAlignment test,
   * with the only modifications that:
   *   1) the number of iterations is increased 1000 --> 5000
   *   2) the feature correspondence randomness (the number of kNNs) is decreased 10 --> 2
   */

  // Transform the source cloud by a large amount
  Eigen::Vector3f initial_offset (100, 0, 0);
  float angle = static_cast<float> (M_PI) / 2.0f;
  Eigen::Quaternionf initial_rotation (std::cos (angle / 2), 0, 0, std::sin (angle / 2));
  PointCloud<PointXYZ> cloud_source_transformed;
  transformPointCloud (cloud_source, cloud_source_transformed, initial_offset, initial_rotation);

  // Create shared pointers
  PointCloud<PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
  cloud_source_ptr = cloud_source_transformed.makeShared ();
  cloud_target_ptr = cloud_target.makeShared ();

  // Initialize estimators for surface normals and FPFH features
  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);

  // Normal estimator
  NormalEstimation<PointXYZ, Normal> norm_est;
  norm_est.setSearchMethod (tree);
  norm_est.setRadiusSearch (0.005);
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);

  // FPFH estimator
  FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
  fpfh_est.setSearchMethod (tree);
  fpfh_est.setRadiusSearch (0.05);
  PointCloud<FPFHSignature33>::Ptr features_source(new PointCloud<FPFHSignature33>), features_target(new PointCloud<FPFHSignature33>);

  // Estimate the normals and the FPFH features for the source cloud
  norm_est.setInputCloud (cloud_source_ptr);
  norm_est.compute (*normals);
  fpfh_est.setInputCloud (cloud_source_ptr);
  fpfh_est.setInputNormals (normals);
  fpfh_est.compute (*features_source);

  // Estimate the normals and the FPFH features for the target cloud
  norm_est.setInputCloud (cloud_target_ptr);
  norm_est.compute (*normals);
  fpfh_est.setInputCloud (cloud_target_ptr);
  fpfh_est.setInputNormals (normals);
  fpfh_est.compute (*features_target);

  // Initialize Sample Consensus Prerejective with 5x the number of iterations and 1/5 feature kNNs as SAC-IA
  SampleConsensusPrerejective<PointXYZ, PointXYZ, FPFHSignature33> reg;
  reg.setMaxCorrespondenceDistance (0.1);
  reg.setMaximumIterations (5000);
  reg.setSimilarityThreshold (0.6f);
  reg.setCorrespondenceRandomness (2);

  // Set source and target cloud/features
  reg.setInputSource (cloud_source_ptr);
  reg.setInputTarget (cloud_target_ptr);
  reg.setSourceFeatures (features_source);
  reg.setTargetFeatures (features_target);

  // Register
  reg.align (cloud_reg);

  // Check output consistency and quality of alignment
  EXPECT_EQ (cloud_reg.size (), cloud_source.size ());
  float inlier_fraction = static_cast<float> (reg.getInliers ().size ()) / static_cast<float> (cloud_source.size ());
  EXPECT_GT (inlier_fraction, 0.95f);

  // Check again, for all possible caching schemes
  using PointT = pcl::PointXYZ;
  for (int iter = 0; iter < 4; iter++)
  {
    bool force_cache = static_cast<bool> (iter/2);
    bool force_cache_reciprocal = static_cast<bool> (iter%2);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // Ensure that, when force_cache is not set, we are robust to the wrong input
    if (force_cache)
      tree->setInputCloud (cloud_target_ptr);
    reg.setSearchMethodTarget (tree, force_cache);

    pcl::search::KdTree<PointT>::Ptr tree_recip (new pcl::search::KdTree<PointT>);
    if (force_cache_reciprocal)
      tree_recip->setInputCloud (cloud_source_ptr);
    reg.setSearchMethodSource(tree_recip, force_cache_reciprocal);

    // Register
    reg.align (cloud_reg);

    // Check output consistency and quality of alignment
    EXPECT_EQ (cloud_reg.size (), cloud_source.size ());
    inlier_fraction = static_cast<float> (reg.getInliers ().size ()) / static_cast<float> (cloud_source.size ());
    EXPECT_GT (inlier_fraction, 0.95f);
  }
}

int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd` pass their path to the test." << std::endl;
    return (-1);
  }

  // Input
  if (loadPCDFile (argv[1], cloud_source) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (loadPCDFile (argv[2], cloud_target) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun4.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
