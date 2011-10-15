/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

#include <iostream>

using namespace pcl;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;
PointCloud<PointXYZ> cloud;
KdTreePtr tree;

NormalEstimation<PointXYZ, Normal> n;
IntegralImageNormalEstimation<PointXYZ, Normal> ne;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalEstimation)
{
  tree.reset (new search::KdTree<PointXYZ> (false));
  n.setSearchMethod (tree);
  n.setKSearch (10);

  n.setInputCloud (cloud.makeShared ());

  PointCloud<Normal> output;
  n.compute (output);

  EXPECT_EQ (output.points.size (), cloud.points.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    EXPECT_NEAR (fabs (output.points[i].normal_x),   0, 1e-2);
    EXPECT_NEAR (fabs (output.points[i].normal_y),   0, 1e-2);
    EXPECT_NEAR (fabs (output.points[i].normal_z), 1.0, 1e-2);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IINormalEstimation)
{
  Normal normal;
  ne.setInputCloud (cloud.makeShared ());
  ne.setRectSize (2, 2);
  ne.computePointNormal (160, 120, normal);
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);

  EXPECT_NEAR (fabs (normal.normal_x),   0, 1e-2);
  EXPECT_NEAR (fabs (normal.normal_y),   0, 1e-2);
  EXPECT_NEAR (fabs (normal.normal_z), 1.0, 1e-2);

  PointCloud<Normal> output;
  ne.compute (output);

  EXPECT_EQ (output.points.size (), cloud.points.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (size_t v = 0; v < cloud.height; ++v)
  {
    for (size_t u = 0; u < cloud.width; ++u)
    {
      if (!pcl_isfinite(output (u, v).normal_x) && 
          !pcl_isfinite(output (u, v).normal_y) && 
          !pcl_isfinite(output (u, v).normal_z)) 
        continue;

      EXPECT_NEAR (fabs (output (u, v).normal_x),   0, 1e-2);
      EXPECT_NEAR (fabs (output (u, v).normal_y),   0, 1e-2);
      EXPECT_NEAR (fabs (output (u, v).normal_z), 1.0, 1e-2);
    }
  }
  EXPECT_NEAR (fabs (output (160, 120).normal_x),   0, 1e-2);
  EXPECT_NEAR (fabs (output (160, 120).normal_y),   0, 1e-2);
  EXPECT_NEAR (fabs (output (160, 120).normal_z), 1.0, 1e-2);


  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.compute (output);

  EXPECT_EQ (output.points.size (), cloud.points.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (size_t v = 0; v < cloud.height; ++v)
  {
    for (size_t u = 0; u < cloud.width; ++u)
    {
      if (!pcl_isfinite(output (u, v).normal_x) && 
          !pcl_isfinite(output (u, v).normal_y) && 
          !pcl_isfinite(output (u, v).normal_z)) 
        continue;

      EXPECT_NEAR (fabs (output (u, v).normal_x),   0, 1e-2);
      EXPECT_NEAR (fabs (output (u, v).normal_y),   0, 1e-2);
      EXPECT_NEAR (fabs (output (u, v).normal_z), 1.0, 1e-2);
    }
  }
}


/* ---[ */
int
main (int argc, char** argv)
{
  cloud.points.resize (320 * 240);
  cloud.width = 320;
  cloud.height = 240;
  cloud.is_dense = true;
  for (size_t v = 0; v < cloud.height; ++v)
  {
    for (size_t u = 0; u < cloud.width; ++u)
    {
      cloud (u, v).x = u;
      cloud (u, v).y = v;
      cloud (u, v).z = 10;
    }
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
  
  return 1;
}
/* ]--- */

