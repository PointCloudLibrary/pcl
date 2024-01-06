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
 * $Id: test_pfh_estimation.cpp 4747 2012-02-26 20:51:51Z svn $
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/cvfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace pcl::io;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;
using CloudPtr = PointCloud<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
pcl::Indices indices;
KdTreePtr tree;

CloudPtr cloud_milk;
KdTreePtr tree_milk;
float leaf_size_ = 0.005f;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CVFHEstimation)
{
  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setKSearch (10); // Use 10 nearest neighbors to estimate the normals
  // estimate
  n.compute (*normals);
  CVFHEstimation<PointXYZ, Normal, VFHSignature308> cvfh;
  cvfh.setInputNormals (normals);

  // Object
  PointCloud<VFHSignature308>::Ptr vfhs (new PointCloud<VFHSignature308> ());

  // set parameters
  cvfh.setInputCloud (cloud.makeShared ());
  cvfh.setIndices (indicesptr);
  cvfh.setSearchMethod (tree);

  // estimate
  cvfh.compute (*vfhs);
  EXPECT_EQ (static_cast<int>(vfhs->size ()), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CVFHEstimationMilk)
{

  // Estimate normals first
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  n.setInputCloud (cloud_milk);
  n.setSearchMethod (tree);
  n.setRadiusSearch (leaf_size_ * 4); //2cm to estimate normals
  n.compute (*normals);

  CVFHEstimation<PointXYZ, Normal, VFHSignature308> cvfh;
  cvfh.setInputCloud (cloud_milk);
  cvfh.setInputNormals (normals);
  cvfh.setSearchMethod (tree_milk);
  cvfh.setClusterTolerance (leaf_size_ * 3);
  cvfh.setEPSAngleThreshold (0.13f);
  cvfh.setCurvatureThreshold (0.025f);
  cvfh.setNormalizeBins (false);
  cvfh.setRadiusNormals (leaf_size_ * 4);

  // Object
  PointCloud<VFHSignature308>::Ptr vfhs (new PointCloud<VFHSignature308> ());

  // estimate
  cvfh.compute (*vfhs);
  EXPECT_EQ (static_cast<int>(vfhs->size ()), 2);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and `milk.pcd` pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  CloudPtr milk_loaded(new PointCloud<PointXYZ>());
  if (loadPCDFile<PointXYZ> (argv[2], *milk_loaded) < 0)
  {
    std::cerr << "Failed to read test file. Please download `milk.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.size ());
  for (std::size_t i = 0; i < indices.size (); ++i)
  {
    indices[i] = static_cast<int>(i);
  }

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  cloud_milk.reset(new PointCloud<PointXYZ>());
  CloudPtr grid;
  pcl::VoxelGrid < pcl::PointXYZ > grid_;
  grid_.setInputCloud (milk_loaded);
  grid_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
  grid_.filter (*cloud_milk);

  tree_milk.reset (new search::KdTree<PointXYZ> (false));
  tree_milk->setInputCloud (cloud_milk);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
