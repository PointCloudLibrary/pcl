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

#include <boost/thread.hpp>

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

using namespace pcl;

typedef SampleConsensusModelSphere<PointXYZ>::Ptr SampleConsensusModelSpherePtr;

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
TEST (SampleConsensus, InfiniteLoop)
{
  const unsigned point_count = 100;
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (point_count);
  for (unsigned idx = 0; idx < point_count; ++idx)
  {
    cloud.points[idx].x = static_cast<float> (idx);
    cloud.points[idx].y = 0.0;
    cloud.points[idx].z = 0.0;
  }

  boost::posix_time::time_duration delay (0, 0, 1, 0);
  boost::function<bool ()> sac_function;
  SampleConsensusModelSpherePtr model (new SampleConsensusModelSphere<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> ransac (model, 0.03);
  sac_function = boost::bind (&RandomSampleConsensus<PointXYZ>::computeModel, &ransac, 0);
  boost::thread thread1 (sac_function);
  ASSERT_TRUE (thread1.timed_join (delay));

  // Create the LMSAC object
  LeastMedianSquares<PointXYZ> lmsac (model, 0.03);
  sac_function = boost::bind (&LeastMedianSquares<PointXYZ>::computeModel, &lmsac, 0);
  boost::thread thread2 (sac_function);
  ASSERT_TRUE (thread2.timed_join (delay));

  // Create the MSAC object
  MEstimatorSampleConsensus<PointXYZ> mesac (model, 0.03);
  sac_function = boost::bind (&MEstimatorSampleConsensus<PointXYZ>::computeModel, &mesac, 0);
  boost::thread thread3 (sac_function);
  ASSERT_TRUE (thread3.timed_join (delay));

  // Create the RRSAC object
  RandomizedRandomSampleConsensus<PointXYZ> rrsac (model, 0.03);
  sac_function = boost::bind (&RandomizedRandomSampleConsensus<PointXYZ>::computeModel, &rrsac, 0);
  boost::thread thread4 (sac_function);
  ASSERT_TRUE (thread4.timed_join (delay));

  // Create the RMSAC object
  RandomizedMEstimatorSampleConsensus<PointXYZ> rmsac (model, 0.03);
  sac_function = boost::bind (&RandomizedMEstimatorSampleConsensus<PointXYZ>::computeModel, &rmsac, 0);
  boost::thread thread5 (sac_function);
  ASSERT_TRUE (thread5.timed_join (delay));

  // Create the MLESAC object
  MaximumLikelihoodSampleConsensus<PointXYZ> mlesac (model, 0.03);
  sac_function = boost::bind (&MaximumLikelihoodSampleConsensus<PointXYZ>::computeModel, &mlesac, 0);
  boost::thread thread6 (sac_function);
  ASSERT_TRUE (thread6.timed_join (delay));
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

