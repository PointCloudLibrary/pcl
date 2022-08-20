/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2016-, Open Perception, Inc.
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
#include <pcl/pcl_tests.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/flare.h>

using KdTreePtr = pcl::search::KdTree<pcl::PointXYZ>::Ptr;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

PointCloudPtr cloud;
KdTreePtr tree;

//sampled surface for the computation of tangent X axis
PointCloudPtr sampled_cloud;
KdTreePtr sampled_tree;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FLARELocalReferenceFrameEstimation)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::ReferenceFrame> bunny_LRF;

  const float mesh_res = 0.005f;

  // Compute normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  ne.setRadiusSearch (2.0f*mesh_res);
  ne.setViewPoint (1, 1, 10);
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  ne.compute (*normals);

  // Compute FLARE LRF
  pcl::FLARELocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> lrf_estimator;

  lrf_estimator.setRadiusSearch (5 * mesh_res);
  lrf_estimator.setTangentRadius (20 * mesh_res);

  lrf_estimator.setInputCloud (cloud);
  lrf_estimator.setSearchSurface (cloud);
  lrf_estimator.setInputNormals (normals);
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setSearchMethodForSampledSurface (sampled_tree);
  lrf_estimator.setSearchSampledSurface (sampled_cloud);

  lrf_estimator.compute (bunny_LRF);

  // TESTS
  EXPECT_TRUE (bunny_LRF.is_dense);

  // Expected Results
  float score_15 = -0.0059431493f;
  Eigen::Vector3f point_15_x (-0.46138301f, 0.75752199f, -0.46182927f);
  Eigen::Vector3f point_15_y (-0.78785944f, -0.11049186f, 0.60586232f);
  Eigen::Vector3f point_15_z (0.40792558f, 0.64339107f, 0.64779979f);
  float score_45 = 0.018918669f;
  Eigen::Vector3f point_45_x (0.63724411f, -0.74846953f, -0.18361199f);
  Eigen::Vector3f point_45_y (0.76468521f, 0.58447874f, 0.27136898f);
  Eigen::Vector3f point_45_z (-0.095794097f, -0.31333363f, 0.94479918f);
  float score_163 = -0.050190225f;
  Eigen::Vector3f point_163_x (-0.67064381f, 0.45722002f, 0.58411193f);
  Eigen::Vector3f point_163_y (-0.58332449f, -0.81150508f, -0.034525186f);
  Eigen::Vector3f point_163_z (0.45822418f, -0.36388087f, 0.81093854f);
  float score_253 = -0.025943652f;
  Eigen::Vector3f point_253_x (0.88240892f, -0.26585102f, 0.38817233f);
  Eigen::Vector3f point_253_y (0.19853911f, 0.95840079f, 0.20506138f);
  Eigen::Vector3f point_253_z (-0.42654046f, -0.10388060f, 0.89848322f);


  //Test Results
  for (int d = 0; d < 3; ++d)
  {
    EXPECT_NEAR (point_15_x[d], bunny_LRF.at (15).x_axis[d], 1E-3);
    EXPECT_NEAR (point_15_y[d], bunny_LRF.at (15).y_axis[d], 1E-3);
    EXPECT_NEAR (point_15_z[d], bunny_LRF.at (15).z_axis[d], 1E-3);

    EXPECT_NEAR (point_45_x[d], bunny_LRF.at (45).x_axis[d], 1E-3);
    EXPECT_NEAR (point_45_y[d], bunny_LRF.at (45).y_axis[d], 1E-3);
    EXPECT_NEAR (point_45_z[d], bunny_LRF.at (45).z_axis[d], 1E-3);

    EXPECT_NEAR (point_163_x[d], bunny_LRF.at (163).x_axis[d], 1E-3);
    EXPECT_NEAR (point_163_y[d], bunny_LRF.at (163).y_axis[d], 1E-3);
    EXPECT_NEAR (point_163_z[d], bunny_LRF.at (163).z_axis[d], 1E-3);

    EXPECT_NEAR (point_253_x[d], bunny_LRF.at (253).x_axis[d], 1E-3);
    EXPECT_NEAR (point_253_y[d], bunny_LRF.at (253).y_axis[d], 1E-3);
    EXPECT_NEAR (point_253_z[d], bunny_LRF.at (253).z_axis[d], 1E-3);
  }
  EXPECT_NEAR (score_15, lrf_estimator.getSignedDistancesFromHighestPoints ()[15], 1E-4);
  EXPECT_NEAR (score_45, lrf_estimator.getSignedDistancesFromHighestPoints ()[45], 1E-4);
  EXPECT_NEAR (score_163, lrf_estimator.getSignedDistancesFromHighestPoints ()[163], 1E-4);
  EXPECT_NEAR (score_253, lrf_estimator.getSignedDistancesFromHighestPoints ()[253], 1E-4);
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

  cloud.reset (new pcl::PointCloud<pcl::PointXYZ> ());

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  tree->setInputCloud (cloud);

  //create and set sampled point cloud for computation of X axis
  const float sampling_perc = 0.2f;
  const float sampling_incr = 1.0f / sampling_perc;

  sampled_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::Indices sampled_indices;
  for (float sa = 0.0f; sa < (float)cloud->size (); sa += sampling_incr)
    sampled_indices.push_back (static_cast<int> (sa));
  copyPointCloud (*cloud, sampled_indices, *sampled_cloud);

  sampled_tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  sampled_tree->setInputCloud (sampled_cloud);

  //start tests
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
