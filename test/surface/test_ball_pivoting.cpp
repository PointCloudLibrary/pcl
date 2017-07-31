/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id: test_surface.cpp 6579 2012-07-27 18:57:32Z rusu $
 *
 */

#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/surface/ball_pivoting.h>
#include <pcl/common/common.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

// custom cloud and tree
PointCloud<PointNormal>::Ptr cloud_custom (new PointCloud<PointNormal>);
search::KdTree<PointNormal>::Ptr tree_custom;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BallPivotingTest)
{
  pcl::PolygonMesh mesh_fixed_radius;
  pcl::PolygonMesh mesh_found_radius;

  BallPivoting<PointNormal> pivoter;
  pivoter.setInputCloud (cloud_custom);
  pivoter.setSearchMethod (tree_custom);

  // reconstruction with custom radius
  pivoter.setSearchRadius (0.02);
  pivoter.reconstruct (mesh_fixed_radius);

  // reconstruction with estimated radius
  pivoter.setSearchRadiusAutomatically (100, 5, 0.95f); 
  pivoter.reconstruct (mesh_found_radius);

  // test reconstruction with custom radius
  const std::vector<Vertices> &polygons_fixed_radius = mesh_fixed_radius.polygons;
  EXPECT_EQ (polygons_fixed_radius[100].vertices[0], 101);
  EXPECT_EQ (polygons_fixed_radius[100].vertices[1], 100);
  EXPECT_EQ (polygons_fixed_radius[100].vertices[2], 102);
  EXPECT_EQ (polygons_fixed_radius[200].vertices[0], 201);
  EXPECT_EQ (polygons_fixed_radius[200].vertices[1], 200);
  EXPECT_EQ (polygons_fixed_radius[200].vertices[2], 202);

  // test reconstruction with estimated radius
  const std::vector<Vertices> &polygons_found_radius = mesh_found_radius.polygons;
  EXPECT_EQ (polygons_found_radius[100].vertices[0], 101);
  EXPECT_EQ (polygons_found_radius[100].vertices[1], 100);
  EXPECT_EQ (polygons_found_radius[100].vertices[2], 102);
  EXPECT_EQ (polygons_found_radius[200].vertices[0], 201);
  EXPECT_EQ (polygons_found_radius[200].vertices[1], 200);
  EXPECT_EQ (polygons_found_radius[200].vertices[2], 202);
}

/**
 * return a point cloud similar to triangle strip in computer graphics, 
 * it generally extends to +X direction, with normal in +Z and points on z=0.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr
compose_strip_cloud (const size_t size_cloud)
{
  const Eigen::Vector3f normal (0.0f, 0.0f, 1.0f);
  const Eigen::Vector3f odd_shift = 
    Eigen::Vector3f (0.5f, std::sqrt (3.0f) / 2.0f, 0.0f) / 100.0f;
  const Eigen::Vector3f even_shift = Eigen::Vector3f (1.0f, 0.0f, 0.0f) / 100.0f;
  Eigen::Vector3f base = Eigen::Vector3f::Zero ();

  pcl::PointCloud<pcl::PointNormal>::Ptr re (
      new pcl::PointCloud<pcl::PointNormal> ());
  re->reserve (size_cloud);

  for (size_t id_point = 0; id_point < size_cloud; ++id_point)
  {
    Eigen::Vector3f position = base;
    pcl::PointNormal point;
    if (id_point % 2 == 1)
    {
      position += odd_shift; // shift in this iteration
      base += even_shift; // shift in following iterations
    }
    point.getVector3fMap () = position;
    point.getNormalVector3fMap () = normal;
    re->push_back (point);
  }

  return re;
}

/* ---[ */
int
main (int argc, char** argv)
{
  // use composed point cloud (strip) for unit test
  cloud_custom= compose_strip_cloud (1000);
  tree_custom.reset (new search::KdTree<PointNormal> ());
  tree_custom->setInputCloud (cloud_custom);

  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
