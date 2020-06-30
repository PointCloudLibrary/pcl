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
 * $Id$
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/common/intensity.h>

using namespace pcl;
using namespace pcl::test;

TEST (PointOperators, PointXYZRGBtoIntensity)
{
  using namespace pcl::common;
  IntensityFieldAccessor <PointXYZRGB> convert;
  PointXYZRGB p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.r = 0; p1.g = 127; p1.b = 127;
  float p2 = 0.1f * convert (p1);
  EXPECT_NEAR (p2, 0.1 * static_cast<float> (299*p1.r + 587*p1.g + 114*p1.b)/1000.0f, 1e-4);
}

TEST (PointOperators, PointXYZRGBtoPointXYZI)
{
  using namespace pcl::common;
  IntensityFieldAccessor <PointXYZRGB> rgb_intensity;
  IntensityFieldAccessor <PointXYZI> intensity;
  PointXYZRGB p0; p0.x = 0.1f; p0.y = 0.2f;  p0.z = 0.3f; p0.r = 0; p0.g = 127; p0.b = 127;
  PointXYZRGB p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.r = 0; p1.g = 127; p1.b = 127;
  float value = rgb_intensity (p0 + p1);
  PointXYZI p2;
  intensity.set (p2, value);

  // Disabled. Doesn't make any sense
  //EXPECT_EQ (p2.intensity, static_cast<float> (299*p0.r + 587*p0.g + 114*p0.b)/1000.0f + static_cast<float> (299*p1.r + 587*p1.g + 114*p1.b)/1000.0f);
  intensity.set (p2, rgb_intensity (p1) * 0.1f);
  EXPECT_NEAR (p2.intensity, static_cast<float> (299*p1.r + 587*p1.g + 114*p1.b) / 1000.0f * 0.1, 1e-4);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
