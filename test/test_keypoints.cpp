/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Michael Dixon */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/keypoints/sift_keypoint.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

struct KeypointT
{
  float x, y, z, scale;
};
  
PointCloud<PointXYZI>::Ptr cloud_xyzi (new PointCloud<PointXYZI>);

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SIFTKeypoint)
{
  PointCloud<KeypointT> keypoints;

  // Compute the SIFT keypoints
  SIFTKeypoint<PointXYZI, KeypointT> sift_detector;
  KdTreeFLANN<PointXYZI>::Ptr tree (new KdTreeFLANN<PointXYZI>);
  sift_detector.setSearchMethod (tree);
  sift_detector.setScales (0.02, 5, 3);
  sift_detector.setMinimumContrast (0.03);

  sift_detector.setInputCloud (cloud_xyzi);
  sift_detector.compute (keypoints);

  EXPECT_EQ ((int)keypoints.points.size (), 169);

  // Change the values and re-compute
  sift_detector.setScales (0.05, 5, 3);
  sift_detector.setMinimumContrast (0.06);
  sift_detector.compute (keypoints);

  // Compare to previously validated output
  const int correct_nr_keypoints = 5;
  const float correct_keypoints[correct_nr_keypoints][4] = 
    { 
      // { x,  y,  z,  scale }
      {-0.9425, -0.6381,  1.6445,  0.0794},
      {-0.5083, -0.5587,  1.8519,  0.0500},
      { 1.0265,  0.0500,  1.7154,  0.1000},
      { 0.3005, -0.3007,  1.9526,  0.2000},
      {-0.1002, -0.1002,  1.9933,  0.3175}
    };

  ASSERT_EQ ((int)keypoints.points.size (), correct_nr_keypoints);
  for (int i = 0; i < correct_nr_keypoints; ++i)
  {
    EXPECT_NEAR (keypoints.points[i].x, correct_keypoints[i][0], 1e-4);
    EXPECT_NEAR (keypoints.points[i].y, correct_keypoints[i][1], 1e-4);
    EXPECT_NEAR (keypoints.points[i].z, correct_keypoints[i][2], 1e-4);
    EXPECT_NEAR (keypoints.points[i].scale, correct_keypoints[i][3], 1e-4);
  }

}

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `cturtle.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load a sample point cloud
  if (io::loadPCDFile (argv[1], *cloud_xyzi) < 0)
  {
    std::cerr << "Failed to read test file. Please download `cturtle.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
