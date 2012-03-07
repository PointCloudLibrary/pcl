/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/common/point_operators.h>

using namespace pcl;
using namespace pcl::test;

TEST (PointOperators, PointXYZI)
{
  using namespace pcl::common;
  PointXYZI p0; p0.x = 0.1f; p0.y = 0.2f;  p0.z = 0.3f; p0.intensity = 123;
  PointXYZI p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.intensity = 133;
  PointXYZI p2 = p0 + p1;

  EXPECT_EQ (p2.x, p0.x + p1.x);
  EXPECT_EQ (p2.y, p0.y + p1.y);
  EXPECT_EQ (p2.z, p0.z + p1.z);
  EXPECT_EQ (p2.intensity, p0.intensity + p1.intensity);
  p2 = 0.1f * p1;
  EXPECT_NEAR (p2.x, 0.1 * p1.x, 1e-4);
  EXPECT_NEAR (p2.y, 0.1 * p1.y, 1e-4);
  EXPECT_NEAR (p2.z, 0.1 * p1.z, 1e-4);
  EXPECT_NEAR (p2.intensity, 0.1 * p1.intensity, 1e-4);
  PointXYZI p3 = p1 * 0.1f;
  EXPECT_EQ_VECTORS (p2.getVector3fMap (), p3.getVector3fMap ());
  EXPECT_EQ (p2.intensity, p3.intensity);
}

TEST (PointOperators, PointXYZItoIntensity)
{
  using namespace pcl::common;
  PointInToPointOut <PointXYZI, float> convert;
  PointXYZI p0; p0.x = 0.1f; p0.y = 0.2f;  p0.z = 0.3f; p0.intensity = 123;
  PointXYZI p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.intensity = 133;
  float p2 = convert (p0 + p1);

  EXPECT_EQ (p2, p0.intensity + p1.intensity);
  p2 = 0.1f * convert (p1);
  EXPECT_NEAR (p2, 0.1 * p1.intensity, 1e-4);
}

TEST (PointOperators, PointXYZRGBtoIntensity)
{
  using namespace pcl::common;
  PointInToPointOut <PointXYZRGB, float> convert;
  PointXYZRGB p0; p0.x = 0.1f; p0.y = 0.2f;  p0.z = 0.3f; p0.r = 0; p0.g = 127; p0.b = 127;
  PointXYZRGB p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.r = 0; p1.g = 127; p1.b = 127;
  float p2 = convert (p0 + p1);

  EXPECT_EQ (p2, static_cast<float> (299*p0.r + 587*p0.g + 114*p0.b)/1000.0f + static_cast<float> (299*p1.r + 587*p1.g + 114*p1.b)/1000.0f);
  p2 = 0.1f * convert (p1);
  EXPECT_NEAR (p2, 0.1 * static_cast<float> (299*p1.r + 587*p1.g + 114*p1.b)/1000.0f, 1e-4);
}

TEST (PointOperators, PointXYZRGBtoPointXYZI)
{
  using namespace pcl::common;
  PointInToPointOut <PointXYZRGB, PointXYZI> convert;
  PointXYZRGB p0; p0.x = 0.1f; p0.y = 0.2f;  p0.z = 0.3f; p0.r = 0; p0.g = 127; p0.b = 127;
  PointXYZRGB p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.r = 0; p1.g = 127; p1.b = 127;
  PointXYZI p2 = convert (p0 + p1);

  EXPECT_EQ (p2.intensity, static_cast<float> (299*p0.r + 587*p0.g + 114*p0.b)/1000.0f + static_cast<float> (299*p1.r + 587*p1.g + 114*p1.b)/1000.0f);
  p2 = convert (p1) * 0.1f;
  EXPECT_NEAR (p2.intensity, static_cast<float> (299*p1.r + 587*p1.g + 114*p1.b) / 1000.0f * 0.1, 1e-4);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
