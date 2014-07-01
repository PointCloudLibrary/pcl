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
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BaseFeature)
{
  // compute3DCentroid (indices)
  Eigen::Vector4f centroid3;
  compute3DCentroid (cloud, indices, centroid3);
  EXPECT_NEAR (centroid3[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroid3[1], 0.102653, 1e-4);
  EXPECT_NEAR (centroid3[2], 0.027302, 1e-4);
  EXPECT_NEAR (centroid3[3], 1, 1e-4);

  // compute3Dcentroid
  compute3DCentroid (cloud, centroid3);
  EXPECT_NEAR (centroid3[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroid3[1], 0.102653, 1e-4);
  EXPECT_NEAR (centroid3[2], 0.027302, 1e-4);
  EXPECT_NEAR (centroid3[3], 1, 1e-4);

  // computeNDCentroid (indices)
  Eigen::VectorXf centroidn;
  computeNDCentroid (cloud, indices, centroidn);
  EXPECT_NEAR (centroidn[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroidn[1], 0.102653, 1e-4);
  EXPECT_NEAR (centroidn[2], 0.027302, 1e-4);

  // computeNDCentroid
  computeNDCentroid (cloud, centroidn);
  EXPECT_NEAR (centroidn[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroidn[1], 0.102653, 1e-4);
  EXPECT_NEAR (centroidn[2], 0.027302, 1e-4);

  // computeCovarianceMatrix (indices)
  Eigen::Matrix3f covariance_matrix;
  computeCovarianceMatrix (cloud, indices, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0), 0.710046, 1e-4);
  EXPECT_NEAR (covariance_matrix (0, 1), -0.234843, 1e-4);
  EXPECT_NEAR (covariance_matrix (0, 2), 0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 0), -0.234843, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 1), 0.68695, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 2), -0.220504, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 0), 0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 1), -0.220504, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 2), 0.195448, 1e-4);

  // computeCovarianceMatrix
  computeCovarianceMatrix (cloud, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0), 0.710046, 1e-4);
  EXPECT_NEAR (covariance_matrix (0, 1), -0.234843, 1e-4);
  EXPECT_NEAR (covariance_matrix (0, 2), 0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 0), -0.234843, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 1), 0.68695, 1e-4);
  EXPECT_NEAR (covariance_matrix (1, 2), -0.220504, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 0), 0.0704933, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 1), -0.220504, 1e-4);
  EXPECT_NEAR (covariance_matrix (2, 2), 0.195448, 1e-4);

  // computeCovarianceMatrixNormalized (indices)
  computeCovarianceMatrixNormalized (cloud, indices, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0), 1.7930e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 1), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 2), 1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 0), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 1), 1.7347e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 2), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 0), 1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 1), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 2), 4.9356e-04, 1e-5);

  // computeCovarianceMatrixNormalized
  computeCovarianceMatrixNormalized (cloud, centroid3, covariance_matrix);
  EXPECT_NEAR (covariance_matrix (0, 0), 1.7930e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 1), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (0, 2), 1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 0), -5.9304e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 1), 1.7347e-03, 1e-5);
  EXPECT_NEAR (covariance_matrix (1, 2), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 0), 1.7801e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 1), -5.5683e-04, 1e-5);
  EXPECT_NEAR (covariance_matrix (2, 2), 4.9356e-04, 1e-5);

  // solvePlaneParameters (Vector)
  Eigen::Vector4f plane_parameters;
  float curvature;
  solvePlaneParameters (covariance_matrix, centroid3, plane_parameters, curvature);
  EXPECT_NEAR (fabs (plane_parameters[0]), 0.035592, 1e-4);
  EXPECT_NEAR (fabs (plane_parameters[1]), 0.369596, 1e-4);
  EXPECT_NEAR (fabs (plane_parameters[2]), 0.928511, 1e-4);
  EXPECT_NEAR (fabs (plane_parameters[3]), 0.0622552, 1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);

  // solvePlaneParameters
  float nx, ny, nz;
  solvePlaneParameters (covariance_matrix, nx, ny, nz, curvature);
  EXPECT_NEAR (fabs (nx), 0.035592, 1e-4);
  EXPECT_NEAR (fabs (ny), 0.369596, 1e-4);
  EXPECT_NEAR (fabs (nz), 0.928511, 1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);
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
