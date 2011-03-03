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
 * $Id: test_segmentation.cpp 34668 2010-12-11 22:18:23Z rusu $
 *
 */
/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/segmentation/segment_differences.h>

using namespace pcl;
using namespace pcl::io;

PointCloud<PointXYZ>::Ptr cloud_;
PointCloud<PointXYZ>::Ptr cloud_t_;
KdTree<PointXYZ>::Ptr tree_;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SegmentDifferences, Base)
{
  SegmentDifferences<PointXYZ> sd;
  sd.setInputCloud (cloud_);
  sd.setDistanceThreshold (0.00005);

  // Set the target as itself
  sd.setTargetCloud (cloud_);

  PointCloud<PointXYZ> output;
  sd.segment (output);

  EXPECT_EQ ((int)output.points.size (), 0);
  
  // Set a different target
  sd.setTargetCloud (cloud_t_);
  sd.segment (output);
  EXPECT_EQ ((int)output.points.size (), 126);
  //savePCDFile ("./test/0-t.pcd", output);

  // Reverse
  sd.setInputCloud (cloud_t_);
  sd.setTargetCloud (cloud_);
  sd.segment (output);
  EXPECT_EQ ((int)output.points.size (), 127);
  //savePCDFile ("./test/t-0.pcd", output);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  // Load a standard PCD file from disk
  PointCloud<PointXYZ> cloud, cloud_t;
  loadPCDFile ("./test/bun0.pcd", cloud);

  // Tranpose the cloud
  cloud_t = cloud;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    cloud_t.points[i].x += 0.01;

  cloud_   = cloud.makeShared ();
  cloud_t_ = cloud_t.makeShared ();

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
