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
#include <pcl/registration/ndt.h>

using namespace pcl;
using namespace pcl::io;

PointCloud<PointXYZ>::Ptr cloud_source, cloud_target;

class NormalDistributionsTransformTest : public testing::TestWithParam<NeighborSearchMethod> {};

TEST_P(NormalDistributionsTransformTest, RegistrationTest) {
  NormalDistributionsTransform<PointXYZ, PointXYZ> reg;
  reg.setNeighborSearchMethod (GetParam());
  reg.setStepSize (0.05);
  reg.setResolution (0.025f);
  reg.setInputSource (cloud_source);
  reg.setInputTarget (cloud_target);
  reg.setMaximumIterations (50);
  reg.setTransformationEpsilon (1e-8);

  // Registration test
  PointCloud<PointXYZ> output;
  reg.align (output);
  EXPECT_EQ (output.size(), cloud_source->size());
  EXPECT_LT (reg.getFitnessScore (), 0.001);
  Eigen::Matrix4f transform = reg.getFinalTransformation();

  // Check if the single thread result is consistent
  reg.setNumberOfThreads(4);
  reg.align(output);
  EXPECT_LT((reg.getFinalTransformation() - transform).array().abs().maxCoeff(), 1e-6);
}

INSTANTIATE_TEST_SUITE_P(PCL, NormalDistributionsTransformTest,
  testing::Values(
    NeighborSearchMethod::KDTREE,
    NeighborSearchMethod::DIRECT1,
    NeighborSearchMethod::DIRECT7,
    NeighborSearchMethod::DIRECT27
  ),
  [](const auto& info) -> std::string {
    switch(info.param) {
      case NeighborSearchMethod::KDTREE: return "KDTREE";
      case NeighborSearchMethod::DIRECT1: return "DIRECT1";
      case NeighborSearchMethod::DIRECT7: return "DIRECT7";
      case NeighborSearchMethod::DIRECT27: return "DIRECT27";
    }
  }
);

int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd`pass their path to the test." << std::endl;
    return (-1);
  }

  cloud_source.reset(new PointCloud<PointXYZ>);
  cloud_target.reset(new PointCloud<PointXYZ>);

  if (loadPCDFile (argv[1], *cloud_source) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (loadPCDFile (argv[2], *cloud_target) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun4.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
