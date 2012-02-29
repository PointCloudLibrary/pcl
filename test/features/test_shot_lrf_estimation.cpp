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
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_lrf.h>

using namespace pcl;
using namespace pcl::test;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SHOTLocalReferenceFrameEstimation)
{
  PointCloud<ReferenceFrame> bunny_LRF;

  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));

  // Compute SHOT LRF
  SHOTLocalReferenceFrameEstimation<PointXYZ, ReferenceFrame> lrf_estimator;
  
  float radius = 0.01f;

  lrf_estimator.setRadiusSearch (radius);

  lrf_estimator.setInputCloud (cloud.makeShared ());
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setIndices (indicesptr);

  lrf_estimator.compute (bunny_LRF);

  // TESTS
  EXPECT_EQ (indices.size (), bunny_LRF.size ());

  EXPECT_FALSE (bunny_LRF.is_dense);
  EXPECT_EQ (numeric_limits<float>::max (), bunny_LRF.at (24).confidence);
  EXPECT_TRUE (pcl_isnan (bunny_LRF.at (24).x_axis.normal_x));

  // Expected Results
  float point_15_conf = 0;
  Eigen::Vector3f point_15_x (-0.849213f, 0.528016f, 0.00593846f);
  Eigen::Vector3f point_15_y (0.274564f, 0.451135f, -0.849171f);
  Eigen::Vector3f point_15_z (-0.451055f, -0.719497f, -0.528084f);

  float point_45_conf = 0;
  Eigen::Vector3f point_45_x (0.950556f, 0.0673042f, 0.303171);
  Eigen::Vector3f point_45_y (0.156242f, -0.947328f, -0.279569);
  Eigen::Vector3f point_45_z (0.268386f, 0.313114f, -0.911004);

  float point_163_conf = 0;
  Eigen::Vector3f point_163_x (0.816369f, 0.309943f, -0.487317);
  Eigen::Vector3f point_163_y (0.235273f, -0.949082f, -0.209498);
  Eigen::Vector3f point_163_z (-0.527436f, 0.0563754f, -0.847722);

  float point_253_conf = 0;
  Eigen::Vector3f point_253_x (0.28858f, 0.92087f, 0.262145);
  Eigen::Vector3f point_253_y (0.859237f, -0.369874f, 0.353418);
  Eigen::Vector3f point_253_z (0.422413f, 0.123256f, -0.897984);

  //Test Results
  EXPECT_NEAR (point_15_conf,bunny_LRF.at (15).confidence, 1E-3);
  EXPECT_NEAR_VECTORS (point_15_x, bunny_LRF.at (15).x_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_15_y, bunny_LRF.at (15).y_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_15_z, bunny_LRF.at (15).z_axis.getNormalVector3fMap (), 1E-3);

  EXPECT_NEAR (point_45_conf, bunny_LRF.at (45).confidence, 1E-3);
  EXPECT_NEAR_VECTORS (point_45_x, bunny_LRF.at (45).x_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_45_y, bunny_LRF.at (45).y_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_45_z, bunny_LRF.at (45).z_axis.getNormalVector3fMap (), 1E-3);

  EXPECT_NEAR (point_163_conf, bunny_LRF.at (163).confidence, 1E-3);
  EXPECT_NEAR_VECTORS (point_163_x, bunny_LRF.at (163).x_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_163_y, bunny_LRF.at (163).y_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_163_z, bunny_LRF.at (163).z_axis.getNormalVector3fMap (), 1E-3);

  EXPECT_NEAR (point_253_conf, bunny_LRF.at (253).confidence, 1E-3);
  EXPECT_NEAR_VECTORS (point_253_x, bunny_LRF.at (253).x_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_253_y, bunny_LRF.at (253).y_axis.getNormalVector3fMap (), 1E-3);
  EXPECT_NEAR_VECTORS (point_253_z, bunny_LRF.at (253).z_axis.getNormalVector3fMap (), 1E-3);  
}

#ifndef PCL_ONLY_CORE_POINT_TYPES
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  TEST (PCL, SHOTLocalReferenceFrameEstimationEigen)
  {
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
    PointCloud<Eigen::MatrixXf> bunny_LRF;

    boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));

    // Compute SHOT LRF
    SHOTLocalReferenceFrameEstimation<PointXYZ, ReferenceFrame> lrf_estimator;
  
    float radius = 0.01f;

    lrf_estimator.setRadiusSearch (radius);

    lrf_estimator.setInputCloud (cloud.makeShared ());
    lrf_estimator.setSearchMethod (tree);
    lrf_estimator.setIndices (indicesptr);

    lrf_estimator.computeEigen (bunny_LRF);

    // TESTS
    EXPECT_EQ (indices.size (), bunny_LRF.size ());

    EXPECT_FALSE (bunny_LRF.is_dense);
    EXPECT_EQ (numeric_limits<float>::max (), bunny_LRF.points (24, 0));
    EXPECT_TRUE (pcl_isnan (bunny_LRF.points (24, 1)));

    // Expected Results
    float point_15_conf = 0;
    Eigen::Vector3f point_15_x (-0.849213f, 0.528016f, 0.00593846f);
    Eigen::Vector3f point_15_y (0.274564f, 0.451135f, -0.849171f);
    Eigen::Vector3f point_15_z (-0.451055f, -0.719497f, -0.528084f);

    float point_45_conf = 0;
    Eigen::Vector3f point_45_x (0.950556f, 0.0673042f, 0.303171);
    Eigen::Vector3f point_45_y (0.156242f, -0.947328f, -0.279569);
    Eigen::Vector3f point_45_z (0.268386f, 0.313114f, -0.911004);

    float point_163_conf = 0;
    Eigen::Vector3f point_163_x (0.816369f, 0.309943f, -0.487317);
    Eigen::Vector3f point_163_y (0.235273f, -0.949082f, -0.209498);
    Eigen::Vector3f point_163_z (-0.527436f, 0.0563754f, -0.847722);

    float point_253_conf = 0;
    Eigen::Vector3f point_253_x (0.28858f, 0.92087f, 0.262145);
    Eigen::Vector3f point_253_y (0.859237f, -0.369874f, 0.353418);
    Eigen::Vector3f point_253_z (0.422413f, 0.123256f, -0.897984);

    ////Test Results
    EXPECT_NEAR (point_15_conf, bunny_LRF.points (15,0), 1E-3);
    EXPECT_NEAR_VECTORS (point_15_x, bunny_LRF.points.block<1,3> (15, 1), 1E-3);
    EXPECT_NEAR_VECTORS (point_15_y, bunny_LRF.points.block<1,3> (15, 4), 1E-3);
    EXPECT_NEAR_VECTORS (point_15_z, bunny_LRF.points.block<1,3> (15, 7), 1E-3);

    EXPECT_NEAR (point_45_conf, bunny_LRF.points (45, 0), 1E-3);
    EXPECT_NEAR_VECTORS (point_45_x, bunny_LRF.points.block<1,3> (45, 1), 1E-3);
    EXPECT_NEAR_VECTORS (point_45_y, bunny_LRF.points.block<1,3> (45, 4), 1E-3);
    EXPECT_NEAR_VECTORS (point_45_z, bunny_LRF.points.block<1,3> (45, 7), 1E-3);

    EXPECT_NEAR (point_163_conf, bunny_LRF.points (163, 0), 1E-3);
    EXPECT_NEAR_VECTORS (point_163_x, bunny_LRF.points.block<1,3> (163, 1), 1E-3);
    EXPECT_NEAR_VECTORS (point_163_y, bunny_LRF.points.block<1,3> (163, 4), 1E-3);
    EXPECT_NEAR_VECTORS (point_163_z, bunny_LRF.points.block<1,3> (163, 7), 1E-3);

    EXPECT_NEAR (point_253_conf, bunny_LRF.points (253, 0), 1E-3);
    EXPECT_NEAR_VECTORS (point_253_x, bunny_LRF.points.block<1,3> (253, 1), 1E-3);
    EXPECT_NEAR_VECTORS (point_253_y, bunny_LRF.points.block<1,3> (253, 4), 1E-3);
    EXPECT_NEAR_VECTORS (point_253_z, bunny_LRF.points.block<1,3> (253, 7), 1E-3);  
  }
#endif

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
  {
    indices[i] = i;
  }

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
