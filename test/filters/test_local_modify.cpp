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
#include <pcl/filters/local_modify.h>
#include <pcl/point_types.h>
#include <stdio.h>

using namespace pcl;

PointCloud<PointXYZ> cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_BOX);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (0.25f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_BOX);
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1.0f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (2.0f, cloud_out[0].z);
  EXPECT_EQ (2.0f, cloud_out[1].z);
  EXPECT_EQ (2.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MEAN);
  lm.setLocalityType (lm.LT_BOX);
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 4.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MEDIAN);
  lm.setLocalityType (lm.LT_BOX);
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.75f, cloud_out[0].z);
  EXPECT_EQ (0.75f, cloud_out[1].z);
  EXPECT_EQ (0.75f, cloud_out[2].z);
  EXPECT_EQ (0.75f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (0.25f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1.0f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (2.0f, cloud_out[0].z);
  EXPECT_EQ (2.0f, cloud_out[1].z);
  EXPECT_EQ (2.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MEAN);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 4.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MEDIAN);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.75f, cloud_out[0].z);
  EXPECT_EQ (0.75f, cloud_out[1].z);
  EXPECT_EQ (0.75f, cloud_out[2].z);
  EXPECT_EQ (0.75f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_KNN);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (0.5f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (1.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_KNN);
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1.0f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MEAN);
  lm.setLocalityType (lm.LT_KNN);
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);

  mean = (cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z) / 2.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);

  mean = (cloud_in[1].z + cloud_in[2].z) / 2.0;

  EXPECT_EQ (mean, cloud_out[2].z);

  mean = (cloud_in[2].z + cloud_in[3].z) / 2.0;

  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MEDIAN);
  lm.setLocalityType (lm.LT_KNN);
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (1.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.375f, cloud_out[0].z);
  EXPECT_EQ (0.375f, cloud_out[1].z);
  EXPECT_EQ (0.75f, cloud_out[2].z);
  EXPECT_EQ (1.5f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MIN);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.25f);
  EXPECT_EQ (cloud_out[1].z, 0.25f);
  EXPECT_EQ (cloud_out[2].z, 0.25f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.25f);
  EXPECT_EQ (cloud_out[1].z, 0.25f);
  EXPECT_EQ (cloud_out[2].z, 0.25f);
  EXPECT_EQ (cloud_out[3].z, 0.25f);
  EXPECT_EQ (cloud_out.size (), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MAX);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out[1].z, 1.0f);
  EXPECT_EQ (cloud_out[2].z, 1.0f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 2.0f);
  EXPECT_EQ (cloud_out[1].z, 2.0f);
  EXPECT_EQ (cloud_out[2].z, 2.0f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MEAN);
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (cloud_out[0].z, mean);
  EXPECT_EQ (cloud_out[1].z, mean);
  EXPECT_EQ (cloud_out[2].z, mean);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 4.0;

  EXPECT_EQ (cloud_out[0].z, mean);
  EXPECT_EQ (cloud_out[1].z, mean);
  EXPECT_EQ (cloud_out[2].z, mean);
  EXPECT_EQ (cloud_out[3].z, mean);
  EXPECT_EQ (cloud_out.size (), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MEDIAN);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.5f);
  EXPECT_EQ (cloud_out[1].z, 0.5f);
  EXPECT_EQ (cloud_out[2].z, 0.5f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.75f);
  EXPECT_EQ (cloud_out[1].z, 0.75f);
  EXPECT_EQ (cloud_out[2].z, 0.75f);
  EXPECT_EQ (cloud_out[3].z, 0.75f);
  EXPECT_EQ (cloud_out.size (), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box_OMP, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_BOX);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (0.25f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box_OMP, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_BOX);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1.0f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (2.0f, cloud_out[0].z);
  EXPECT_EQ (2.0f, cloud_out[1].z);
  EXPECT_EQ (2.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box_OMP, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MEAN);
  lm.setLocalityType (lm.LT_BOX);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 4.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Box_OMP, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setStatType (lm.ST_MEDIAN);
  lm.setLocalityType (lm.LT_BOX);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setResolution (10.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.75f, cloud_out[0].z);
  EXPECT_EQ (0.75f, cloud_out[1].z);
  EXPECT_EQ (0.75f, cloud_out[2].z);
  EXPECT_EQ (0.75f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius_OMP, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (0.25f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius_OMP, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1.0f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (2.0f, cloud_out[0].z);
  EXPECT_EQ (2.0f, cloud_out[1].z);
  EXPECT_EQ (2.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius_OMP, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MEAN);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 4.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);
  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Radius_OMP, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.setStatType (lm.ST_MEDIAN);
  lm.setLocalityType (lm.LT_RADIUS);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setRadius (8.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.75f, cloud_out[0].z);
  EXPECT_EQ (0.75f, cloud_out[1].z);
  EXPECT_EQ (0.75f, cloud_out[2].z);
  EXPECT_EQ (0.75f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN_OMP, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MIN);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.25f, cloud_out[2].z);
  EXPECT_EQ (0.5f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.25f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (1.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN_OMP, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MAX);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (1.0f, cloud_out[0].z);
  EXPECT_EQ (1.0f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (1.0f, cloud_out[2].z);
  EXPECT_EQ (2.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN_OMP, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MEAN);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);
  EXPECT_EQ (mean, cloud_out[2].z);

  mean = (cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 3.0;

  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z) / 2.0;

  EXPECT_EQ (mean, cloud_out[0].z);
  EXPECT_EQ (mean, cloud_out[1].z);

  mean = (cloud_in[1].z + cloud_in[2].z) / 2.0;

  EXPECT_EQ (mean, cloud_out[2].z);

  mean = (cloud_in[2].z + cloud_in[3].z) / 2.0;

  EXPECT_EQ (mean, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (KNN_OMP, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setNumNeighbors (2);
  lm.setStatType (lm.ST_MEDIAN);
  lm.setLocalityType (lm.LT_KNN);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (0.5f, cloud_out[0].z);
  EXPECT_EQ (0.5f, cloud_out[1].z);
  EXPECT_EQ (0.5f, cloud_out[2].z);
  EXPECT_EQ (1.0f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());

  lm.setNumNeighbors (1);
  lm.filter (cloud_out);

  EXPECT_EQ (0.375f, cloud_out[0].z);
  EXPECT_EQ (0.375f, cloud_out[1].z);
  EXPECT_EQ (0.75f, cloud_out[2].z);
  EXPECT_EQ (1.5f, cloud_out[3].z);
  EXPECT_EQ (4, cloud_out.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid_OMP, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MIN);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.25f);
  EXPECT_EQ (cloud_out[1].z, 0.25f);
  EXPECT_EQ (cloud_out[2].z, 0.25f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.25f);
  EXPECT_EQ (cloud_out[1].z, 0.25f);
  EXPECT_EQ (cloud_out[2].z, 0.25f);
  EXPECT_EQ (cloud_out[3].z, 0.25f);
  EXPECT_EQ (cloud_out.size (), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid_OMP, Maximum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MAX);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 1.0f);
  EXPECT_EQ (cloud_out[1].z, 1.0f);
  EXPECT_EQ (cloud_out[2].z, 1.0f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 2.0f);
  EXPECT_EQ (cloud_out[1].z, 2.0f);
  EXPECT_EQ (cloud_out[2].z, 2.0f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid_OMP, Mean)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MEAN);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  float mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z) / 3.0;

  EXPECT_EQ (cloud_out[0].z, mean);
  EXPECT_EQ (cloud_out[1].z, mean);
  EXPECT_EQ (cloud_out[2].z, mean);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  mean = (cloud_in[0].z + cloud_in[1].z + cloud_in[2].z + cloud_in[3].z) / 4.0;

  EXPECT_EQ (cloud_out[0].z, mean);
  EXPECT_EQ (cloud_out[1].z, mean);
  EXPECT_EQ (cloud_out[2].z, mean);
  EXPECT_EQ (cloud_out[3].z, mean);
  EXPECT_EQ (cloud_out.size (), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid_OMP, Median)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (4);

  cloud_in[0].x = 0;    cloud_in[0].y = 0;    cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.25; cloud_in[1].y = 0.25; cloud_in[1].z = 0.5;
  cloud_in[2].x = 0.5;  cloud_in[2].y = 0.5;  cloud_in[2].z = 1;
  cloud_in[3].x = 5;    cloud_in[3].y = 5;    cloud_in[3].z = 2;

  LocalModify<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setResolution (1.0f);
  lm.setLocalityType (lm.LT_GRID);
  lm.setStatType (lm.ST_MEDIAN);
  lm.setNumberOfThreads (omp_get_num_procs ());
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.5f);
  EXPECT_EQ (cloud_out[1].z, 0.5f);
  EXPECT_EQ (cloud_out[2].z, 0.5f);
  EXPECT_EQ (cloud_out[3].z, 2.0f);
  EXPECT_EQ (cloud_out.size (), 4);

  lm.setResolution (6.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.75f);
  EXPECT_EQ (cloud_out[1].z, 0.75f);
  EXPECT_EQ (cloud_out[2].z, 0.75f);
  EXPECT_EQ (cloud_out[3].z, 0.75f);
  EXPECT_EQ (cloud_out.size (), 4);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

