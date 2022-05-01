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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/random.h> // NormalGenerator
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_poly.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CorrespondenceRejectors, CorrespondenceRejectionMedianDistance)
{
  pcl::CorrespondencesPtr corresps (new pcl::Correspondences ());
  for (int i = 0; i <= 10; ++i)
  {
    pcl::Correspondence c;
    c.distance = static_cast<float> (i * i);
    corresps->push_back (c);
  }

  pcl::registration::CorrespondenceRejectorMedianDistance rejector;
  rejector.setInputCorrespondences (corresps);
  rejector.setMedianFactor (2.0);

  pcl::Correspondences corresps_filtered;
  rejector.getCorrespondences (corresps_filtered);

  EXPECT_EQ (corresps_filtered.size (), 8);
  for (int i = 0; i < 8; ++i)
    EXPECT_NEAR (corresps_filtered[i].distance, static_cast<float> (i * i), 1e-5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CorrespondenceRejectors, CorrespondenceRejectionPoly)
{
  // Size of point cloud
  const int size = static_cast<int> (cloud->size ());
  
  // Ground truth correspondences
  pcl::Correspondences corr (size);
  for (int i = 0; i < size; ++i)
    corr[i].index_query = corr[i].index_match = i;
  
  // Scramble the first floor(size*3/4) correspondences by adding floor(size/8) to the match index
  const int last = 3*size/4;
  const int inc = size/8;
  for (int i = 0; i < last; ++i)
    corr[i].index_match += inc;  
  
  // Transform the target
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3f t(0.1f, 0.2f, 0.3f);
  Eigen::Quaternionf q (float (std::cos (0.5*M_PI_4)), 0.0f, 0.0f, float (std::sin (0.5*M_PI_4)));
  pcl::transformPointCloud (*cloud, *target, t, q);
  
  // Noisify the target with a known seed and N(0, 0.005) using deterministic sampling
  pcl::common::NormalGenerator<float> nd(0, 0.005, 1e6);
  for (auto &point : *target)
  {
    point.x += nd.run();
    point.y += nd.run();
    point.z += nd.run();
  }
  
  // Test rejector with varying seeds
  const unsigned int seed = std::time(nullptr);
  std::srand (seed);
  
  // Create a rejection object
  pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZ, pcl::PointXYZ> reject;
  reject.setIterations (20000);
  reject.setCardinality (3);
  reject.setSimilarityThreshold (0.8f);
  reject.setInputSource (cloud);
  reject.setInputTarget (target);
  
  // Run rejection
  pcl::Correspondences result;
  reject.getRemainingCorrespondences (corr, result);
  
  // Ground truth fraction of inliers and estimated fraction of inliers
  const float ground_truth_frac = float (size-last) / float (size);
  const float accepted_frac = float (result.size()) / float (size);

  /*
   * Test criterion 1: verify that the method accepts at least 25 % of the input correspondences,
   * but not too many
   */
  EXPECT_GE(accepted_frac, ground_truth_frac);
  // Factor 1.5 raised to 1.6 as there is a variance in the noise added from the various standard implementations
  // See #2995 for details
  EXPECT_LE(accepted_frac, 1.6f*ground_truth_frac);

  /*
   * Test criterion 2: expect high precision/recall. The true positives are the unscrambled correspondences
   * where the query/match index are equal.
   */
  std::size_t true_positives = 0;
  for (auto &i : result)
    if (i.index_query == i.index_match)
      ++true_positives;
  const std::size_t false_positives = result.size() - true_positives;

  const double precision = double(true_positives) / double(true_positives+false_positives);
  const double recall = double(true_positives) / double(size-last);
  EXPECT_NEAR(precision, 1.0, 0.4);
  EXPECT_NEAR(recall, 1.0, 0.2);
}


/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test files given. Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Input
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
