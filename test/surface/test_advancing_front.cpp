/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2017-, Southwest Research Institute
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/advancing_front.h>

// This test shows the results of meshing on a square grid that has a sinusoidal
// variability in the z axis.  Red arrows show the surface normal for each triangle
// in the mesh, and cyan boxes show the points used to seed the mesh creation algorithm
TEST (PCL, AdvancingFront)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int gridSize = 50;

  for (unsigned int x = 0; x < gridSize; x++)
  {
    for (unsigned int y = 0; y < gridSize; y++)
    {
      double d = 0.001 * ((double)rand () / (double)RAND_MAX);
      pcl::PointXYZ pt (x / 10.0, y / 10.0, 0.5 * cos (double(x) / 10.0) - 0.5 * sin (double(y) / 10.0) + d);
      cloud.push_back (pt);
    }
  }
  cloud.is_dense = false;

  pcl::AdvancingFront<pcl::PointXYZ> mesher;
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ> (cloud));
  pcl::PolygonMesh output;

  mesher.setRho (0.5);
  mesher.setReduction (0.8);
  mesher.setSearchRadius (1);
  mesher.setPolynomialOrder (2);
  mesher.setBoundaryAngleThreshold (M_PI_2);
  mesher.setInputCloud (in_cloud);
  mesher.reconstruct (output);

  EXPECT_TRUE (output.polygons.size() == 93);
}

// Run all the tests that were declared with TEST()
int
main (int argc, char **argv)
{
  testing::InitGoogleTest (&argc, argv);
  return RUN_ALL_TESTS ();
}
