/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: test_moving_least_squares_problem.cpp 6579 2014-07-03 21:30:00Z mohawkjohn $
 *
 */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/common/common.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
search::KdTree<PointXYZ>::Ptr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, MovingLeastSquares)
{
  // Init objects
  PointCloud<PointXYZ> mls_points;
  MovingLeastSquares<PointXYZ, PointXYZ> mls;
  
  // Set parameters
  mls.setInputCloud (cloud);
  //mls.setComputeNormals (true);
  //mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (4.65032);
  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
  mls.setUpsamplingRadius (0.8);
  mls.setUpsamplingStepSize (0.4);
  
  // Reconstruct
  mls.process (mls_points);
  
  for (size_t i = 0; i < mls_points.points.size (); ++i)
  {
    // Check for NaNs in output
    EXPECT_TRUE (pcl::isFinite(mls_points.points[i]));
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load file
  cloud->points.push_back(pcl::PointXYZ(-89.546,  4.03964, 450.883));
  cloud->points.push_back(pcl::PointXYZ(-88.8728, 4.03964, 450.883));
  cloud->points.push_back(pcl::PointXYZ(-86.8529, 4.03964, 450.883));
  cloud->points.push_back(pcl::PointXYZ(-85.5064, 4.03964, 450.883));
  
  // Create search tree
  tree.reset (new search::KdTree<PointXYZ>);
  
  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
