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
 */


#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/features/brisk_2d.h>
#include <set>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef PointXYZRGBA PointT;
typedef PointWithScale KeyPointT;


PointCloud<PointT>::Ptr cloud_image (new PointCloud<PointT>);
PointCloud<PointWithScale>::Ptr cloud_keypoints (new PointCloud<PointWithScale>);
PointCloud<BRISKSignature512>::Ptr cloud_descriptors (new PointCloud<BRISKSignature512>);
PointCloud<PointWithScale>::Ptr cloud_keypoints_gt (new PointCloud<PointWithScale>);
PointCloud<BRISKSignature512>::Ptr cloud_descriptors_gt (new PointCloud<BRISKSignature512>);

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BRISK_2D)
{
#if defined(__SSSE3__) && !defined(__i386__)
  // Compute BRISK keypoints 
  BriskKeypoint2D<PointT> brisk_keypoint_estimation;
  brisk_keypoint_estimation.setThreshold (60);
  brisk_keypoint_estimation.setOctaves (4);
  brisk_keypoint_estimation.setInputCloud (cloud_image);

  cloud_keypoints.reset (new PointCloud<KeyPointT>);
  brisk_keypoint_estimation.compute (*cloud_keypoints);

  //io::savePCDFileBinary ("brisk_keypoints.pcd", *cloud_keypoints);

  const int num_of_keypoints = int (cloud_keypoints->size ());
  const int num_of_keypoints_gt = int (cloud_keypoints_gt->size ());
  EXPECT_EQ (num_of_keypoints_gt, num_of_keypoints);


  for (size_t point_index = 0; point_index < cloud_keypoints->size (); ++point_index)
  {
    PointWithScale & point = (*cloud_keypoints) [point_index];

    const float dx = point.x - point.x;
    const float dy = point.y - point.y;
    const float dz = point.z - point.z;

    const float sqr_distance = (dx*dx + dy*dy + dz*dz);

    EXPECT_NEAR (0.0f, sqr_distance, 1e-4);
  }

  BRISK2DEstimation<PointT> brisk_descriptor_estimation;
  brisk_descriptor_estimation.setInputCloud (cloud_image);
  brisk_descriptor_estimation.setKeypoints (cloud_keypoints);


  cloud_descriptors.reset (new PointCloud<BRISKSignature512>);
  brisk_descriptor_estimation.compute (*cloud_descriptors);

  const int num_of_descriptors = int (cloud_descriptors->size ());
  const int num_of_descriptors_gt = int (cloud_descriptors_gt->size ());
  EXPECT_EQ (num_of_descriptors_gt, num_of_descriptors);


  //io::savePCDFileBinary ("brisk_descriptors.pcd", *cloud_descriptors);
  //for (size_t point_index = 0; point_index < cloud_keypoints->size (); ++point_index)
  for (size_t point_index = 0; point_index < cloud_descriptors->size (); ++point_index)
  {
    BRISKSignature512 & descriptor = (*cloud_descriptors) [point_index];
    BRISKSignature512 & descriptor_gt = (*cloud_descriptors_gt) [point_index];

    float sqr_dist = 0.0f;
    for (size_t index = 0; index < 33; ++index)
    {
      const float dist = float (descriptor.descriptor[index] - descriptor_gt.descriptor[index]);
      sqr_dist += dist * dist;
    }

    EXPECT_NEAR (0.0f, sqr_dist, 1e-4);
  }
#else
  PCL_WARN ("Not compiled with SSSE3, skipping test of Brisk.\n");
#endif
}



/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `image_gt.pcd`, `keypoints_gt.pcd`, as well as `descriptors_gt.pcd`, and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load a sample point cloud
  if (io::loadPCDFile (argv[1], *cloud_image) < 0)
  {
    std::cerr << "Failed to read test file.  Please download `image_gt.pcd`, `keypoints_gt.pcd`, as well as `descriptors_gt.pcd`, and pass its path to the test." << std::endl;
    return (-1);
  }

  if (io::loadPCDFile (argv[2], *cloud_keypoints_gt) < 0)
  {
    std::cerr << "Failed to read test file.  Please download `image_gt.pcd`, `keypoints_gt.pcd`, as well as `descriptors_gt.pcd`, and pass its path to the test." << std::endl;
    return (-1);
  }

  if (io::loadPCDFile (argv[3], *cloud_descriptors_gt) < 0)
  {
    std::cerr << "Failed to read test file.  Please download `image_gt.pcd`, `keypoints_gt.pcd`, as well as `descriptors_gt.pcd`, and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
