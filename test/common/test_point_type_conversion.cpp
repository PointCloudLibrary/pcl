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
 * $Id:  $
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

using namespace pcl;
using namespace pcl::test;

TEST (PointTypeConversions, PointXYZRGBtoPointXYZHSV)
{
  pcl::PointXYZRGB rgb;
  rgb.r = 0;  rgb.g = 0; rgb.b = 0;

  pcl::PointXYZHSV hsv;
  pcl::PointXYZRGBtoXYZHSV (rgb, hsv);

  EXPECT_EQ (hsv.h, 0);
  EXPECT_EQ (hsv.s, 0);
  EXPECT_EQ (hsv.v, 0);

  rgb.r = 100;  rgb.g = 100; rgb.b = 100;

  pcl::PointXYZRGBtoXYZHSV (rgb, hsv);

  EXPECT_EQ (hsv.h, 0);
  EXPECT_EQ (hsv.s, 0);
  EXPECT_NEAR (hsv.v, 0.392157, 1e-4);

  rgb.r = 255;  rgb.g = 255; rgb.b = 255;

  pcl::PointXYZRGBtoXYZHSV (rgb, hsv);

  EXPECT_EQ (hsv.h, 0);
  EXPECT_EQ (hsv.s, 0);
  EXPECT_EQ (hsv.v, 1);

  rgb.r = 240;  rgb.g = 90; rgb.b = 250;

  pcl::PointXYZRGBtoXYZHSV (rgb, hsv);

  EXPECT_NEAR (hsv.h, 296.25, 1e-2);
  EXPECT_NEAR (hsv.s, 0.64, 1e-2);
  EXPECT_NEAR (hsv.v, 0.980392, 1e-2);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
