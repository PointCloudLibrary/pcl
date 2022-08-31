
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: $
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/ransac_based/model_library.h>
#include <pcl/features/normal_3d.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::recognition;

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudTN = pcl::PointCloud<pcl::Normal>;
using PointCloudTPtr = pcl::PointCloud<PointT>::Ptr;
using PointCloudTNPtr = pcl::PointCloud<pcl::Normal>::Ptr;

PointCloud<PointXYZ>::Ptr cloud_;
PointCloudTPtr model_cloud(new pcl::PointCloud<PointT>);
PointCloudTNPtr model_cloud_normals (new pcl::PointCloud<pcl::Normal>);

//////////////////////////////////////////////////////////////////////////////////////////////

int
estimateNormals(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
    // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (cloud);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Use all neighbors in a sphere of radius 1m
      // experiments with tensors dataset show that the points are as far apart as 1m from each other
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals);

      // cloud_normals->size () should have the same size as the input cloud->size ()*
      return cloud_normals->size();
}


//////////////////////////////////////////////////////////////////////////////////////////////

TEST (ORROctreeTest, OctreeSphereIntersection)
{
  float voxel_size = 0.02f;
  float pair_width = 0.05f;
  float frac_of_points_for_registration = 0.3f;
  std::string object_name = "test_object";

  ModelLibrary::Model new_model (*model_cloud, *model_cloud_normals, voxel_size, object_name, frac_of_points_for_registration);

  const ORROctree& octree = new_model.getOctree ();
  const std::vector<ORROctree::Node*> &full_leaves = octree.getFullLeaves ();
  list<ORROctree::Node*> inter_leaves;

  // Run through all full leaves
  for ( const auto& leaf1 : full_leaves )
  {
    const ORROctree::Node::Data* node_data1 = leaf1->getData ();
    // Get all full leaves at the right distance to the current leaf
    inter_leaves.clear ();
    octree.getFullLeavesIntersectedBySphere (node_data1->getPoint (), pair_width, inter_leaves);
    // Ensure that inter_leaves does not contain leaf1
    for ( const auto& leaf2 : inter_leaves )
    {
      EXPECT_NE(leaf1, leaf2);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load a standard PCD file from disk
  if (pcl::io::loadPCDFile (argv[1], *model_cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (!estimateNormals(model_cloud, model_cloud_normals) == model_cloud->size())
  {
    std::cerr << "Failed to estimate normals" << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
