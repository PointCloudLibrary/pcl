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
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
pcl::Indices indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BoundaryEstimation)
{
  Eigen::Vector4f u = Eigen::Vector4f::Zero ();
  Eigen::Vector4f v = Eigen::Vector4f::Zero ();

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (static_cast<int> (indices.size ()));
  // estimate
  n.compute (*normals);

  BoundaryEstimation<PointXYZ, Normal, Boundary> b;
  b.setInputNormals (normals);
  EXPECT_EQ (b.getInputNormals (), normals);

  // getCoordinateSystemOnPlane
  for (const auto &point : normals->points)
  {
    b.getCoordinateSystemOnPlane (point, u, v);
    Vector4fMapConst n4uv = point.getNormalVector4fMap ();
    EXPECT_NEAR (n4uv.dot(u), 0, 1e-4);
    EXPECT_NEAR (n4uv.dot(v), 0, 1e-4);
    EXPECT_NEAR (u.dot(v), 0, 1e-4);
  }

  // isBoundaryPoint (indices)
  bool pt = false;
  pt = b.isBoundaryPoint (cloud, 0, indices, u, v, float (M_PI) / 2.0);
  EXPECT_FALSE (pt);
  pt = b.isBoundaryPoint (cloud, static_cast<int> (indices.size ()) / 3, indices, u, v, float (M_PI) / 2.0);
  EXPECT_FALSE (pt);
  pt = b.isBoundaryPoint (cloud, static_cast<int> (indices.size ()) / 2, indices, u, v, float (M_PI) / 2.0);
  EXPECT_FALSE (pt);
  pt = b.isBoundaryPoint (cloud, static_cast<int> (indices.size ()) - 1, indices, u, v, float (M_PI) / 2.0);
  EXPECT_TRUE (pt);

  // isBoundaryPoint (points)
  pt = b.isBoundaryPoint (cloud, cloud[0], indices, u, v, float (M_PI) / 2.0);
  EXPECT_FALSE (pt);
  pt = b.isBoundaryPoint (cloud, cloud[indices.size () / 3], indices, u, v, float (M_PI) / 2.0);
  EXPECT_FALSE (pt);
  pt = b.isBoundaryPoint (cloud, cloud[indices.size () / 2], indices, u, v, float (M_PI) / 2.0);
  EXPECT_FALSE (pt);
  pt = b.isBoundaryPoint (cloud, cloud[indices.size () - 1], indices, u, v, float (M_PI) / 2.0);
  EXPECT_TRUE (pt);

  // Object
  PointCloud<Boundary>::Ptr bps (new PointCloud<Boundary> ());

  // set parameters
  b.setInputCloud (cloud.makeShared ());
  b.setIndices (indicesptr);
  b.setSearchMethod (tree);
  b.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  b.compute (*bps);
  EXPECT_EQ (bps->size (), indices.size ());

  pt = (*bps)[0].boundary_point;
  EXPECT_FALSE (pt);
  pt = (*bps)[indices.size () / 3].boundary_point;
  EXPECT_FALSE (pt);
  pt = (*bps)[indices.size () / 2].boundary_point;
  EXPECT_FALSE (pt);
  pt = (*bps)[indices.size () - 1].boundary_point;
  EXPECT_TRUE (pt);
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
  for (std::size_t i = 0; i < indices.size (); ++i)
    indices[i] = static_cast<int> (i);

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
