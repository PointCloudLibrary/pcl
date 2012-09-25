/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

/** \author Gioia Ballin */

#include <gtest/gtest.h>
#include <pcl/pcl_tests.h>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>

using namespace pcl;
using namespace pcl::io;

//
// Main variables
//
double cloud_resolution (0.0058329);
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ISSKeypoint3D_WBE)
{
  PointCloud<PointXYZ> keypoints;

  //
  // Compute the ISS 3D keypoints - Without Boundary Estimation
  //
  ISSKeypoint3D<PointXYZ, PointXYZ> iss_detector;
  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (6 * cloud_resolution);
  iss_detector.setNonMaxRadius (4 * cloud_resolution);

  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (5);
  iss_detector.setNumberOfThreads (1);
  iss_detector.setInputCloud (cloud);
  iss_detector.compute (keypoints);

  //
  // Compare to previously validated output
  //
  const size_t correct_nr_keypoints = 6;
  const float correct_keypoints[correct_nr_keypoints][3] =
    {
      // { x,  y,  z}
      {-0.071112f,  0.137670f,  0.047518f},
      {-0.041733f,  0.127960f,  0.016650f},
      {-0.011943f,  0.086771f,  0.057009f},
      { 0.031733f,  0.099372f,  0.038505f},
      {-0.062116f,  0.045145f,  0.037802f},
      {-0.048250f,  0.167480f, -0.000152f}
    };


  ASSERT_EQ (keypoints.points.size (), correct_nr_keypoints);

  for (size_t i = 0; i < correct_nr_keypoints; ++i)
  {
    EXPECT_NEAR (keypoints.points[i].x, correct_keypoints[i][0], 1e-6);
    EXPECT_NEAR (keypoints.points[i].y, correct_keypoints[i][1], 1e-6);
    EXPECT_NEAR (keypoints.points[i].z, correct_keypoints[i][2], 1e-6);
  }

  tree.reset (new search::KdTree<PointXYZ> ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ISSKeypoint3D_BE)
{
  PointCloud<PointXYZ> keypoints;

  //
  // Compute the ISS 3D keypoints - By first performing the Boundary Estimation
  //

  ISSKeypoint3D<PointXYZ, PointXYZ> iss_detector;

  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (6 * cloud_resolution);
  iss_detector.setNonMaxRadius (4 * cloud_resolution);

  iss_detector.setNormalRadius (4 * cloud_resolution);
  iss_detector.setBorderRadius (4 * cloud_resolution);

  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (5);
  iss_detector.setAngleThreshold (static_cast<float> (M_PI) / 3.0);
  iss_detector.setNumberOfThreads (1);

  iss_detector.setInputCloud (cloud);
  iss_detector.compute (keypoints);


  //
  // Compare to previously validated output
  //
  const size_t correct_nr_keypoints = 5;
  const float correct_keypoints[correct_nr_keypoints][3] =
    {
      // { x,  y,  z}
      {-0.052037f,  0.116800f,  0.034582f},
      { 0.027420f,  0.096386f,  0.043312f},
      {-0.011943f,  0.086771f,  0.057009f},
      {-0.070344f,  0.087352f,  0.041908f},
      {-0.030035f,  0.066130f,  0.038942f}
    };

  ASSERT_EQ (keypoints.points.size (), correct_nr_keypoints);

  for (size_t i = 0; i < correct_nr_keypoints; ++i)
  {
    EXPECT_NEAR (keypoints.points[i].x, correct_keypoints[i][0], 1e-6);
    EXPECT_NEAR (keypoints.points[i].y, correct_keypoints[i][1], 1e-6);
    EXPECT_NEAR (keypoints.points[i].z, correct_keypoints[i][2], 1e-6);
  }

  tree.reset (new search::KdTree<PointXYZ> ());
}

//* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load a sample point cloud
  if (io::loadPCDFile (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
