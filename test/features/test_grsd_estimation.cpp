/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2014, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/grsd.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;

search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GRSDEstimation)
{
  // Estimate normals first
  double rad = 0.02;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setRadiusSearch (rad);
  n.compute (*normals);

  EXPECT_NEAR ((*normals)[103].normal_x, 0.694, 0.1);
  EXPECT_NEAR ((*normals)[103].normal_y, -0.562, 0.1);
  EXPECT_NEAR ((*normals)[103].normal_z, -0.448, 0.1);
  
  // GRSDEstimation
  double rsd_radius = 0.03;
  GRSDEstimation<PointXYZ, Normal, GRSDSignature21> grsd;
  grsd.setInputNormals (normals);
  PointCloud<GRSDSignature21>::Ptr grsd_desc (new PointCloud<GRSDSignature21> ());
  grsd.setInputCloud (cloud);
  grsd.setSearchMethod (tree);
  grsd.setRadiusSearch (rsd_radius);
  grsd.compute (*grsd_desc);
  
  EXPECT_EQ (12, (*grsd_desc)[0].histogram[2]);
  EXPECT_EQ (104, (*grsd_desc)[0].histogram[4]);
  EXPECT_EQ (0, (*grsd_desc)[0].histogram[6]);
  EXPECT_EQ (0, (*grsd_desc)[0].histogram[8]);
  EXPECT_EQ (0, (*grsd_desc)[0].histogram[10]);
  EXPECT_EQ (34, (*grsd_desc)[0].histogram[12]);
  EXPECT_EQ (167, (*grsd_desc)[0].histogram[14]);
  EXPECT_EQ (68, (*grsd_desc)[0].histogram[16]);
  EXPECT_EQ (204, (*grsd_desc)[0].histogram[18]);
  EXPECT_EQ (0, (*grsd_desc)[0].histogram[20]);
  
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

  if (loadPCDFile<PointXYZ> (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
