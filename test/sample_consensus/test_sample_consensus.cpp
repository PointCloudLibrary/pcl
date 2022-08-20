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

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

using namespace pcl;

using SampleConsensusModelSpherePtr = SampleConsensusModelSphere<PointXYZ>::Ptr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensus, Base)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  // Create a shared sphere model pointer directly
  SampleConsensusModelSpherePtr model (new SampleConsensusModelSphere<PointXYZ> (cloud));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Basic tests
  ASSERT_EQ (0.03, sac.getDistanceThreshold ());
  sac.setDistanceThreshold (0.03);
  ASSERT_EQ (0.03, sac.getDistanceThreshold ());

  sac.setProbability (0.99);
  ASSERT_EQ (0.99, sac.getProbability ());

  sac.setMaxIterations (10000);
  ASSERT_EQ (10000, sac.getMaxIterations ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Test if RANSAC finishes within a second.
template <typename SacT>
class SacTest : public ::testing::Test {};

using sacTypes = ::testing::Types<
  RandomSampleConsensus<PointXYZ>,
  LeastMedianSquares<PointXYZ>,
  MEstimatorSampleConsensus<PointXYZ>,
  RandomizedRandomSampleConsensus<PointXYZ>,
  RandomizedMEstimatorSampleConsensus<PointXYZ>,
  MaximumLikelihoodSampleConsensus<PointXYZ>
>;
TYPED_TEST_SUITE(SacTest, sacTypes);

TYPED_TEST(SacTest, InfiniteLoop)
{
  using namespace std::chrono_literals;

  const unsigned point_count = 100;
  PointCloud<PointXYZ> cloud;
  cloud.resize (point_count);
  for (unsigned idx = 0; idx < point_count; ++idx)
  {
    cloud[idx].x = static_cast<float> (idx);
    cloud[idx].y = 0.0;
    cloud[idx].z = 0.0;
  }

  SampleConsensusModelSpherePtr model (new SampleConsensusModelSphere<PointXYZ> (cloud.makeShared ()));
  TypeParam sac (model, 0.03);

  // This test sometimes fails for LMedS on azure, but always passes when run locally.
  // Enable all output for LMedS, so that when it fails next time, we hopefully see why.
  // This can be removed again when the failure reason is found and fixed.
  int debug_verbosity_level = 0;
  const auto previous_verbosity_level = pcl::console::getVerbosityLevel();
  if (std::is_same<TypeParam, LeastMedianSquares<PointXYZ>>::value) {
    debug_verbosity_level = 2;
    pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
  }

  // Set up timed conditions
  std::condition_variable cv;
  std::mutex mtx;

  // Create the RANSAC object
  std::thread thread ([&] ()
  {
    sac.computeModel (debug_verbosity_level);

    // Notify things are done
    std::lock_guard<std::mutex> lock (mtx);
    cv.notify_one ();
  });


  // Waits for the delay
  std::unique_lock<std::mutex> lock (mtx);
  #if defined(DEBUG) || defined(_DEBUG)
    EXPECT_EQ (std::cv_status::no_timeout, cv.wait_for (lock, 15s));
  #else
    EXPECT_EQ (std::cv_status::no_timeout, cv.wait_for (lock, 2s));
  #endif
  // release lock to avoid deadlock
  lock.unlock();
  thread.join ();

  pcl::console::setVerbosityLevel(previous_verbosity_level); // reset verbosity level
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
