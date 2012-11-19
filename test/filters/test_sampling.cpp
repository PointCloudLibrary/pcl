/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: test_filters.cpp 7683 2012-10-23 02:49:03Z rusu $
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

using namespace pcl;

PointCloud<PointXYZ>::Ptr cloud_walls (new PointCloud<PointXYZ> ()),
                          cloud_turtle (new PointCloud<PointXYZ> ());
PointCloud<PointNormal>::Ptr cloud_walls_normals (new PointCloud<PointNormal> ()),
                             cloud_turtle_normals (new PointCloud<PointNormal> ());


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CovarianceSampling, Filters)
{
  CovarianceSampling<PointNormal, PointNormal> covariance_sampling;
  covariance_sampling.setInputCloud (cloud_walls_normals);
  covariance_sampling.setNormals (cloud_walls_normals);
  covariance_sampling.setNumberOfSamples (static_cast<unsigned int> (cloud_walls_normals->size ()) / 4);
  double cond_num_walls = covariance_sampling.computeConditionNumber ();
  EXPECT_NEAR (cond_num_walls, 19.3518, 1e-4);

  IndicesPtr walls_indices (new std::vector<int> ());
  covariance_sampling.filter (*walls_indices);

  covariance_sampling.setIndices (walls_indices);
  double cond_num_walls_sampled = covariance_sampling.computeConditionNumber ();
  EXPECT_NEAR (cond_num_walls_sampled, 4.0298, 1e-4);

  EXPECT_EQ ((*walls_indices)[0], 316);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () / 4], 3208);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () / 2], 2546);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () * 3 / 4], 1886);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () - 1], 321);

  covariance_sampling.setInputCloud (cloud_turtle_normals);
  covariance_sampling.setNormals (cloud_turtle_normals);
  covariance_sampling.setIndices (IndicesPtr ());
  covariance_sampling.setNumberOfSamples (static_cast<unsigned int> (cloud_turtle_normals->size ()) / 8);
  double cond_num_turtle = covariance_sampling.computeConditionNumber ();
  EXPECT_NEAR (cond_num_turtle, 20661.7663, 1e-4);

  IndicesPtr turtle_indices (new std::vector<int> ());
  covariance_sampling.filter (*turtle_indices);
  covariance_sampling.setIndices (turtle_indices);
  double cond_num_turtle_sampled = covariance_sampling.computeConditionNumber ();
  EXPECT_NEAR (cond_num_turtle_sampled, 5795.5057, 1e-4);

  EXPECT_EQ ((*turtle_indices)[0], 80344);
  EXPECT_EQ ((*turtle_indices)[turtle_indices->size () / 4], 145982);
  EXPECT_EQ ((*turtle_indices)[turtle_indices->size () / 2], 104557);
  EXPECT_EQ ((*turtle_indices)[turtle_indices->size () * 3 / 4], 41512);
  EXPECT_EQ ((*turtle_indices)[turtle_indices->size () - 1], 136885);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (NormalSpaceSampling, Filters)
{
  NormalSpaceSampling<PointNormal, PointNormal> normal_space_sampling;
  normal_space_sampling.setInputCloud (cloud_walls_normals);
  normal_space_sampling.setNormals (cloud_walls_normals);
  normal_space_sampling.setBins (16, 16, 16);
  normal_space_sampling.setSeed (0);
  normal_space_sampling.setSample (static_cast<unsigned int> (cloud_walls_normals->size ()) / 4);

  IndicesPtr walls_indices (new std::vector<int> ());
  normal_space_sampling.filter (*walls_indices);

  CovarianceSampling<PointNormal, PointNormal> covariance_sampling;
  covariance_sampling.setInputCloud (cloud_walls_normals);
  covariance_sampling.setNormals (cloud_walls_normals);
  covariance_sampling.setIndices (walls_indices);
  covariance_sampling.setNumberOfSamples (0);
  double cond_num_walls_sampled = covariance_sampling.computeConditionNumber ();


  EXPECT_NEAR (cond_num_walls_sampled, 7.8289, 1e-4);

  EXPECT_EQ ((*walls_indices)[0], 2314);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () / 4], 1876);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () / 2], 2230);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () * 3 / 4], 1538);
  EXPECT_EQ ((*walls_indices)[walls_indices->size () - 1], 805);
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load two standard PCD files from disk
  if (argc < 3)
  {
    std::cerr << "No test files given. Please download `sac_plane_test.pcd` and 'cturtle.pcd' and pass them path to the test." << std::endl;
    return (-1);
  }

  // Load in the point clouds
  io::loadPCDFile (argv[1], *cloud_walls);
  io::loadPCDFile (argv[2], *cloud_turtle);



  // Compute the normals for each cloud, and then clean them up of any NaN values
  NormalEstimation<PointXYZ,PointNormal> ne;
  ne.setInputCloud (cloud_walls);
  ne.setRadiusSearch (0.02);
  ne.compute (*cloud_walls_normals);
  copyPointCloud (*cloud_walls, *cloud_walls_normals);

  std::vector<int> aux_indices;
  removeNaNFromPointCloud (*cloud_walls_normals, *cloud_walls_normals, aux_indices);
  removeNaNNormalsFromPointCloud (*cloud_walls_normals, *cloud_walls_normals, aux_indices);

  ne = NormalEstimation<PointXYZ, PointNormal> ();
  ne.setInputCloud (cloud_turtle);
  ne.setKSearch (5);
  ne.compute (*cloud_turtle_normals);
  copyPointCloud (*cloud_turtle, *cloud_turtle_normals);
  removeNaNFromPointCloud (*cloud_turtle_normals, *cloud_turtle_normals, aux_indices);
  removeNaNNormalsFromPointCloud (*cloud_turtle_normals, *cloud_turtle_normals, aux_indices);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
