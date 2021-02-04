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
#include <pcl/io/pcd_io.h>
#include <pcl/features/moment_invariants.h>

using namespace pcl;
using namespace pcl::io;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
pcl::Indices indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, MomentInvariantsEstimation)
{
  float j1, j2, j3;

  MomentInvariantsEstimation<PointXYZ, MomentInvariants> mi;

  // computePointMomentInvariants (indices))
  mi.computePointMomentInvariants (cloud, indices, j1, j2, j3);
  EXPECT_NEAR (j1, 1.59244, 1e-4);
  EXPECT_NEAR (j2, 0.652063, 1e-4);
  EXPECT_NEAR (j3, 0.053917, 1e-4);

  // computePointMomentInvariants
  mi.computePointMomentInvariants (cloud, indices, j1, j2, j3);
  EXPECT_NEAR (j1, 1.59244, 1e-4);
  EXPECT_NEAR (j2, 0.652063, 1e-4);
  EXPECT_NEAR (j3, 0.053917, 1e-4);

  // Object
  PointCloud<MomentInvariants>::Ptr moments (new PointCloud<MomentInvariants> ());

  // set parameters
  mi.setInputCloud (cloud.makeShared ());
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  mi.setIndices (indicesptr);
  mi.setSearchMethod (tree);
  mi.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  mi.compute (*moments);
  EXPECT_EQ (moments->size (), indices.size ());

  for (const auto &point : moments->points)
  {
    EXPECT_NEAR (point.j1, 1.59244, 1e-4);
    EXPECT_NEAR (point.j2, 0.652063, 1e-4);
    EXPECT_NEAR (point.j3, 0.053917, 1e-4);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.size ());
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
    indices[i] = i;

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
