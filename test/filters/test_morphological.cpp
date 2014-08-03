/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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
#include <pcl/filters/morphological_filter.h>
#include <pcl/point_types.h>

using namespace pcl;

PointCloud<PointXYZ> cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Morphological, Dilate)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  float resolution = 5.0f;

  applyMorphologicalOperator<PointXYZ> (cloud_in.makeShared (), resolution, MORPH_DILATE, cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out[1].z, 1.0f);
  EXPECT_EQ (cloud_in.size(), cloud_out.size());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Morphological, Erode)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  float resolution = 5.0f;

  applyMorphologicalOperator<PointXYZ> (cloud_in.makeShared (), resolution, MORPH_ERODE, cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.0f);
  EXPECT_EQ (cloud_out[1].z, 0.0f);
  EXPECT_EQ (cloud_in.size(), cloud_out.size());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Morphological, Open)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  float resolution = 5.0f;

  applyMorphologicalOperator<PointXYZ> (cloud_in.makeShared (), resolution, MORPH_OPEN, cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.0f);
  EXPECT_EQ (cloud_out[1].z, 0.0f);
  EXPECT_EQ (cloud_in.size (), cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Morphological, Close)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  float resolution = 5.0f;

  applyMorphologicalOperator<PointXYZ> (cloud_in.makeShared (), resolution, MORPH_CLOSE, cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out[1].z, 1.0f);
  EXPECT_EQ (cloud_in.size (), cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Morphological, Unsupported)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 2;
  cloud_in.is_dense = true;
  cloud_in.resize (2);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0;
  cloud_in[1].x = 1; cloud_in[1].y = 1; cloud_in[1].z = 1;

  float resolution = 5.0f;

  applyMorphologicalOperator<PointXYZ> (cloud_in.makeShared (), resolution, 99, cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.0f);
  EXPECT_EQ (cloud_out[1].z, 1.0f);
  EXPECT_EQ (cloud_in.size (), cloud_out.size ());
}


/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

