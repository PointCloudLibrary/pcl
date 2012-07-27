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
 *
 *
 */
#include <gtest/gtest.h>

#include <iostream>  // For debug

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

const float PI = 3.14159265f;
const float rho = sqrtf (2.0f) / 2.0f;  // cos(PI/4) == sin(PI/4)

PointCloud<PointXYZ> cloud;
sensor_msgs::PointCloud2 cloud_blob;

void 
init ()
{
  PointXYZ p0 (0, 0, 0);
  PointXYZ p1 (1, 0, 0);
  PointXYZ p2 (0, 1, 0);
  PointXYZ p3 (0, 0, 1);
  cloud.points.push_back (p0);
  cloud.points.push_back (p1);
  cloud.points.push_back (p2);
  cloud.points.push_back (p3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DeMean)
{
  PointCloud<PointXYZ> cloud, cloud_demean;
  fromROSMsg (cloud_blob, cloud);

  Eigen::Vector4f centroid;
  compute3DCentroid (cloud, centroid);
  EXPECT_NEAR (centroid[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroid[1],  0.102653,  1e-4);
  EXPECT_NEAR (centroid[2],  0.027302,  1e-4);
  EXPECT_NEAR (centroid[3],  0,         1e-4);

  // Check standard demean
  demeanPointCloud (cloud, centroid, cloud_demean);
  EXPECT_EQ (cloud_demean.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud_demean.width, cloud.width);
  EXPECT_EQ (cloud_demean.height, cloud.height);
  EXPECT_EQ (cloud_demean.points.size (), cloud.points.size ());

  EXPECT_NEAR (cloud_demean.points[0].x, 0.034503, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].y, 0.010837, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].z, 0.013447, 1e-4);

  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].x, -0.048849, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].y,  0.072507, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].z, -0.071702, 1e-4);

  vector<int> indices (cloud.points.size ());
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i) { indices[i] = i; }

  // Check standard demean w/ indices
  demeanPointCloud (cloud, indices, centroid, cloud_demean);
  EXPECT_EQ (cloud_demean.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud_demean.width, cloud.width);
  EXPECT_EQ (cloud_demean.height, cloud.height);
  EXPECT_EQ (cloud_demean.points.size (), cloud.points.size ());

  EXPECT_NEAR (cloud_demean.points[0].x, 0.034503, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].y, 0.010837, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].z, 0.013447, 1e-4);

  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].x, -0.048849, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].y,  0.072507, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].z, -0.071702, 1e-4);

  // Check eigen demean
  Eigen::MatrixXf mat_demean;
  demeanPointCloud (cloud, centroid, mat_demean);
  EXPECT_EQ (mat_demean.cols (), int (cloud.points.size ()));
  EXPECT_EQ (mat_demean.rows (), 4);

  EXPECT_NEAR (mat_demean (0, 0), 0.034503, 1e-4);
  EXPECT_NEAR (mat_demean (1, 0), 0.010837, 1e-4);
  EXPECT_NEAR (mat_demean (2, 0), 0.013447, 1e-4);

  EXPECT_NEAR (mat_demean (0, cloud_demean.points.size () - 1), -0.048849, 1e-4);
  EXPECT_NEAR (mat_demean (1, cloud_demean.points.size () - 1),  0.072507, 1e-4);
  EXPECT_NEAR (mat_demean (2, cloud_demean.points.size () - 1), -0.071702, 1e-4);

  // Check eigen demean + indices
  demeanPointCloud (cloud, indices, centroid, mat_demean);
  EXPECT_EQ (mat_demean.cols (), int (cloud.points.size ()));
  EXPECT_EQ (mat_demean.rows (), 4);

  EXPECT_NEAR (mat_demean (0, 0), 0.034503, 1e-4);
  EXPECT_NEAR (mat_demean (1, 0), 0.010837, 1e-4);
  EXPECT_NEAR (mat_demean (2, 0), 0.013447, 1e-4);

  EXPECT_NEAR (mat_demean (0, cloud_demean.points.size () - 1), -0.048849, 1e-4);
  EXPECT_NEAR (mat_demean (1, cloud_demean.points.size () - 1),  0.072507, 1e-4);
  EXPECT_NEAR (mat_demean (2, cloud_demean.points.size () - 1), -0.071702, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Transform)
{
  Eigen::Vector3f offset (100, 0, 0);
  float angle = PI/4;
  Eigen::Quaternionf rotation (cos (angle / 2), 0, 0, sin (angle / 2));

  PointCloud<PointXYZ> cloud_out;
  const vector<PointXYZ, Eigen::aligned_allocator<PointXYZ> > &points (cloud_out.points);
  transformPointCloud (cloud, cloud_out, offset, rotation);

  EXPECT_EQ (cloud.points.size (), cloud_out.points.size ());
  EXPECT_EQ (100, points[0].x);
  EXPECT_EQ (0, points[0].y);
  EXPECT_EQ (0, points[0].z);
  EXPECT_NEAR (100+rho, points[1].x,  1e-4);
  EXPECT_NEAR (rho, points[1].y,  1e-4);
  EXPECT_EQ (0, points[1].z);
  EXPECT_NEAR (100-rho, points[2].x,  1e-4);
  EXPECT_NEAR (rho, points[2].y,  1e-4);
  EXPECT_EQ (0, points[2].z);
  EXPECT_EQ (100, points[3].x);
  EXPECT_EQ (0, points[3].y);
  EXPECT_EQ (1, points[3].z);

  PointCloud<PointXYZ> cloud_out2;
  const vector<PointXYZ, Eigen::aligned_allocator<PointXYZ> > &points2 (cloud_out2.points);
  Eigen::Translation3f translation (offset);
  Eigen::Affine3f transform;
  transform = translation * rotation;
  transformPointCloud (cloud, cloud_out2, transform);

  EXPECT_EQ (cloud.points.size (), cloud_out2.points.size ());
  EXPECT_EQ (100, points2[0].x);
  EXPECT_EQ (0, points2[0].y);
  EXPECT_EQ (0, points2[0].z);
  EXPECT_NEAR (100+rho, points2[1].x,  1e-4);
  EXPECT_NEAR (rho, points2[1].y,  1e-4);
  EXPECT_EQ (0, points2[1].z);
  EXPECT_NEAR (100-rho, points2[2].x,  1e-4);
  EXPECT_NEAR (rho, points2[2].y,  1e-4);
  EXPECT_EQ (0, points2[2].z);
  EXPECT_EQ (100, points2[3].x);
  EXPECT_EQ (0, points2[3].y);
  EXPECT_EQ (1, points2[3].z);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, commonTransform)
{
  Eigen::Vector3f xaxis (1,0,0), yaxis (0,1,0), zaxis (0,0,1);
  Eigen::Affine3f trans = pcl::getTransFromUnitVectorsZY (zaxis, yaxis);
  Eigen::Vector3f xaxistrans=trans*xaxis, yaxistrans=trans*yaxis, zaxistrans=trans*zaxis;
  //std::cout << xaxistrans<<"\n"<<yaxistrans<<"\n"<<zaxistrans<<"\n";
  EXPECT_NEAR ((xaxistrans-xaxis).norm(), 0.0f,  1e-6);
  EXPECT_NEAR ((yaxistrans-yaxis).norm(), 0.0f,  1e-6);
  EXPECT_NEAR ((zaxistrans-zaxis).norm(), 0.0f,  1e-6);
  
  trans = pcl::getTransFromUnitVectorsXY (xaxis, yaxis);
  xaxistrans=trans*xaxis, yaxistrans=trans*yaxis, zaxistrans=trans*zaxis;
  //std::cout << xaxistrans<<"\n"<<yaxistrans<<"\n"<<zaxistrans<<"\n";
  EXPECT_NEAR ((xaxistrans-xaxis).norm(), 0.0f,  1e-6);
  EXPECT_NEAR ((yaxistrans-yaxis).norm(), 0.0f,  1e-6);
  EXPECT_NEAR ((zaxistrans-zaxis).norm(), 0.0f,  1e-6);
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
  if (loadPCDFile (argv[1], cloud_blob) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  init();
  return (RUN_ALL_TESTS ());
}
/* ]--- */
