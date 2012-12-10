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

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PrincipalCurvaturesEstimation)
{
  float pcx, pcy, pcz, pc1, pc2;

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);

  PrincipalCurvaturesEstimation<PointXYZ, Normal, PrincipalCurvatures> pc;
  pc.setInputNormals (normals);
  EXPECT_EQ (pc.getInputNormals (), normals);

  // computePointPrincipalCurvatures (indices)
  pc.computePointPrincipalCurvatures (*normals, 0, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (fabs (pcx), 0.98509, 1e-4);
  EXPECT_NEAR (fabs (pcy), 0.10714, 1e-4);
  EXPECT_NEAR (fabs (pcz), 0.13462, 1e-4);
  EXPECT_NEAR (pc1, 0.23997423052787781, 1e-4);
  EXPECT_NEAR (pc2, 0.19400238990783691, 1e-4);

  pc.computePointPrincipalCurvatures (*normals, 2, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.98079, 1e-4);
  EXPECT_NEAR (pcy, -0.04019, 1e-4);
  EXPECT_NEAR (pcz, 0.19086, 1e-4);
  EXPECT_NEAR (pc1, 0.27207490801811218, 1e-4);
  EXPECT_NEAR (pc2, 0.19464978575706482, 1e-4);

  int indices_size = static_cast<int> (indices.size ());
  pc.computePointPrincipalCurvatures (*normals, indices_size - 3, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.86725, 1e-4);
  EXPECT_NEAR (pcy, -0.37599, 1e-4);
  EXPECT_NEAR (pcz, 0.32635, 1e-4);
  EXPECT_NEAR (pc1, 0.25900053977966309, 1e-4);
  EXPECT_NEAR (pc2, 0.17906945943832397, 1e-4);

  pc.computePointPrincipalCurvatures (*normals, indices_size - 1, indices, pcx, pcy, pcz, pc1, pc2);
  EXPECT_NEAR (pcx, 0.86725, 1e-4);
  EXPECT_NEAR (pcy, -0.375851, 1e-3);
  EXPECT_NEAR (pcz, 0.32636, 1e-4);
  EXPECT_NEAR (pc1, 0.2590005099773407,  1e-4);
  EXPECT_NEAR (pc2, 0.17906956374645233, 1e-4);

  // Object
  PointCloud<PrincipalCurvatures>::Ptr pcs (new PointCloud<PrincipalCurvatures> ());

  // set parameters
  pc.setInputCloud (cloud.makeShared ());
  pc.setIndices (indicesptr);
  pc.setSearchMethod (tree);
  pc.setKSearch (indices_size);

  // estimate
  pc.compute (*pcs);
  EXPECT_EQ (pcs->points.size (), indices.size ());

  // Adjust for small numerical inconsitencies (due to nn_indices not being sorted)
  EXPECT_NEAR (fabs (pcs->points[0].principal_curvature[0]), 0.98509, 1e-4);
  EXPECT_NEAR (fabs (pcs->points[0].principal_curvature[1]), 0.10713, 1e-4);
  EXPECT_NEAR (fabs (pcs->points[0].principal_curvature[2]), 0.13462, 1e-4);
  EXPECT_NEAR (fabs (pcs->points[0].pc1), 0.23997458815574646, 1e-4);
  EXPECT_NEAR (fabs (pcs->points[0].pc2), 0.19400238990783691, 1e-4);

  EXPECT_NEAR (pcs->points[2].principal_curvature[0], 0.98079, 1e-4);
  EXPECT_NEAR (pcs->points[2].principal_curvature[1], -0.04019, 1e-4);
  EXPECT_NEAR (pcs->points[2].principal_curvature[2], 0.19086, 1e-4);
  EXPECT_NEAR (pcs->points[2].pc1, 0.27207502722740173, 1e-4);
  EXPECT_NEAR (pcs->points[2].pc2, 0.1946497857570648,  1e-4);

  EXPECT_NEAR (pcs->points[indices.size () - 3].principal_curvature[0], 0.86725, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 3].principal_curvature[1], -0.37599, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 3].principal_curvature[2], 0.32636, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 3].pc1, 0.2590007483959198,  1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 3].pc2, 0.17906941473484039, 1e-4);

  EXPECT_NEAR (pcs->points[indices.size () - 1].principal_curvature[0], 0.86725, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 1].principal_curvature[1], -0.375851, 1e-3);
  EXPECT_NEAR (pcs->points[indices.size () - 1].principal_curvature[2], 0.32636, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 1].pc1, 0.25900065898895264, 1e-4);
  EXPECT_NEAR (pcs->points[indices.size () - 1].pc2, 0.17906941473484039, 1e-4);
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

  indices.resize (cloud.points.size ());
  for (size_t i = 0; i < indices.size (); ++i)
    indices[i] = static_cast<int> (i);

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
