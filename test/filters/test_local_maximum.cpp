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
#include <pcl/filters/local_maximum.h>
#include <pcl/point_types.h>

using namespace pcl;

PointCloud<PointXYZ> cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (Filters, LocalMaximum)
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

  LocalMaximum<PointXYZ> lm;
  lm.setInputCloud (cloud_in.makeShared ());
  lm.setRadius (1.0f);
  lm.filter (cloud_out);

  EXPECT_EQ (0.25f, cloud_out[0].z);
  EXPECT_EQ (0.50f, cloud_out[1].z);
  EXPECT_EQ (2.00f, cloud_out[2].z);
  EXPECT_EQ (3, cloud_out.size ());
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

