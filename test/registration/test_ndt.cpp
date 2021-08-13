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

PointCloud<PointXYZ> cloud_source, cloud_target;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalDistributionsTransform)
{
  using PointT = PointNormal;
  PointCloud<PointT>::Ptr src (new PointCloud<PointT>);
  copyPointCloud (cloud_source, *src);
  PointCloud<PointT>::Ptr tgt (new PointCloud<PointT>);
  copyPointCloud (cloud_target, *tgt);
  PointCloud<PointT> output;

  NormalDistributionsTransform<PointT, PointT> reg;
  reg.setStepSize (0.05);
  reg.setResolution (0.025f);
  reg.setInputSource (src);
  reg.setInputTarget (tgt);
  reg.setMaximumIterations (50);
  reg.setTransformationEpsilon (1e-8);
  // Register
  reg.align (output);
  EXPECT_EQ (output.size (), cloud_source.size ());
  EXPECT_LT (reg.getFitnessScore (), 0.001);

  // Check again, for all possible caching schemes
  for (int iter = 0; iter < 4; iter++)
  {
    bool force_cache = static_cast<bool> (iter/2);
    bool force_cache_reciprocal = static_cast<bool> (iter%2);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // Ensure that, when force_cache is not set, we are robust to the wrong input
    if (force_cache)
      tree->setInputCloud (tgt);
    reg.setSearchMethodTarget (tree, force_cache);

    pcl::search::KdTree<PointT>::Ptr tree_recip (new pcl::search::KdTree<PointT>);
    if (force_cache_reciprocal)
      tree_recip->setInputCloud (src);
    reg.setSearchMethodSource (tree_recip, force_cache_reciprocal);

    // Register
    reg.align (output);
    EXPECT_EQ (output.size (), cloud_source.size ());
    EXPECT_LT (reg.getFitnessScore (), 0.001);
  }
}

int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd`pass their path to the test." << std::endl;
    return (-1);
  }

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
