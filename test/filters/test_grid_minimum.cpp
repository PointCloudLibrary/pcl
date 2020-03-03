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

#include <pcl/test/gtest.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/point_types.h>

using namespace pcl;

PointCloud<PointXYZ> cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Grid, Minimum)
{
  PointCloud<PointXYZ> cloud_in, cloud_out;

  cloud_in.height = 1;
  cloud_in.width = 3;
  cloud_in.is_dense = true;
  cloud_in.resize (3);

  cloud_in[0].x = 0; cloud_in[0].y = 0; cloud_in[0].z = 0.25;
  cloud_in[1].x = 0.5; cloud_in[1].y = 0.5; cloud_in[1].z = 1;
  cloud_in[2].x = 1.5; cloud_in[2].y = 1.5; cloud_in[2].z = 0.0;

  GridMinimum<PointXYZ> gm (1.0f);
  gm.setInputCloud (cloud_in.makeShared ());
  gm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.25f);
  EXPECT_EQ (cloud_out[1].z, 0.0f);
  EXPECT_EQ (cloud_out.size (), 2);
  EXPECT_EQ (gm.getResolution (), 1.0f);

  gm.setResolution (2.0f);
  gm.filter (cloud_out);

  EXPECT_EQ (cloud_out[0].z, 0.0f);
  EXPECT_EQ (cloud_out.size (), 1);
  EXPECT_EQ (gm.getResolution (), 2.0f);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

