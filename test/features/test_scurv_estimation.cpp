/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2018, University of Innsbruck (Antonio J Rodríguez-Sánchez, Tomas Turecek, Alex Melniciuc)
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
 * $Id: test_scurv_estimation.cpp 4747 2012-02-26 20:51:51Z svn $
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/scurv.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;
typedef PointCloud<PointXYZ>::Ptr CloudPtr;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
vector<int> indices;

float leaf_size_ = 0.005f;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SCurVEstimation)
{

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  n.setInputCloud (cloud);
  //n.setSearchMethod (tree);
  n.setRadiusSearch (leaf_size_ * 4); //2cm to estimate normals
  n.compute (*normals);

  SCurVEstimation<PointXYZ, Normal, SCurVSignature210> scurv;
  scurv.setInputCloud (cloud);
  scurv.setInputNormals (normals);

  // Object
  PointCloud<SCurVSignature210>::Ptr scurvs (new PointCloud<SCurVSignature210> ());

  // estimate
  scurv.compute (*scurvs);
  EXPECT_EQ (static_cast<int>(scurvs->points.size ()), 1);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test file given. Please download `milk.pcd` pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `milk.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
