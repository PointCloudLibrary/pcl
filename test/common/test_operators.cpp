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
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/common/point_operators.h>

using namespace pcl;
using namespace pcl::test;

//////////////////////////////////////////////////////////////////////////////
TEST (PointOperators, PointXYZ)
{
  using namespace pcl::common;
  PointXYZ p0; p0.x = 0.1f;  p0.y = 0.2f;  p0.z = 0.3f;
  PointXYZ p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f;
  PointXYZ p2 = p0;
  p2 += p1;
  EXPECT_EQ (p2.x, p0.x + p1.x);
  EXPECT_EQ (p2.y, p0.y + p1.y);
  EXPECT_EQ (p2.z, p0.z + p1.z);

  p2 = p0 + p1;

  EXPECT_EQ (p2.x, p0.x + p1.x);
  EXPECT_EQ (p2.y, p0.y + p1.y);
  EXPECT_EQ (p2.z, p0.z + p1.z);

  PointXYZ p3 = p0 - p1;

  EXPECT_EQ (p3.x, p0.x - p1.x);
  EXPECT_EQ (p3.y, p0.y - p1.y);
  EXPECT_EQ (p3.z, p0.z - p1.z);

  p3 = p0;
  p3 -= p1;

  EXPECT_EQ (p3.x, p0.x - p1.x);
  EXPECT_EQ (p3.y, p0.y - p1.y);
  EXPECT_EQ (p3.z, p0.z - p1.z);

  float scalar = 4;
  p2 *= scalar;

  EXPECT_EQ (p2.x, scalar * p0.x + scalar * p1.x);
  EXPECT_EQ (p2.y, scalar * p0.y + scalar * p1.y);
  EXPECT_EQ (p2.z, scalar * p0.z + scalar * p1.z);

  p2 /= 2;

  EXPECT_EQ (p2.x, scalar / 2.0f * p0.x + scalar / 2.0f * p1.x);
  EXPECT_EQ (p2.y, scalar / 2.0f * p0.y + scalar / 2.0f * p1.y);
  EXPECT_EQ (p2.z, scalar / 2.0f * p0.z + scalar / 2.0f * p1.z);
}

//////////////////////////////////////////////////////////////////////////////
TEST (PointOperators, PointXYZI)
{
  using namespace pcl::common;
  PointXYZI p0; p0.x = 0.1f; p0.y = 0.2f;  p0.z = 0.3f; p0.intensity = 123;
  PointXYZI p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.intensity = 133;
  PointXYZI p2 = p0 + p1;
  PointXYZI p3 = p0 - p1;

  EXPECT_EQ (p2.x, p0.x + p1.x);
  EXPECT_EQ (p2.y, p0.y + p1.y);
  EXPECT_EQ (p2.z, p0.z + p1.z);
  EXPECT_EQ (p2.intensity, p0.intensity + p1.intensity);

  EXPECT_EQ (p3.x, p0.x - p1.x);
  EXPECT_EQ (p3.y, p0.y - p1.y);
  EXPECT_EQ (p3.z, p0.z - p1.z);
  EXPECT_EQ (p3.intensity, p0.intensity - p1.intensity);

  p2 = 0.1f * p1;
  EXPECT_NEAR (p2.x, 0.1 * p1.x, 1e-4);
  EXPECT_NEAR (p2.y, 0.1 * p1.y, 1e-4);
  EXPECT_NEAR (p2.z, 0.1 * p1.z, 1e-4);
  EXPECT_NEAR (p2.intensity, 0.1 * p1.intensity, 1e-4);
  PointXYZI p4 = p1 * 0.1f;
  EXPECT_EQ_VECTORS (p2.getVector3fMap (), p4.getVector3fMap ());
  EXPECT_EQ (p2.intensity, p4.intensity);
}

//////////////////////////////////////////////////////////////////////////////
TEST (PointOperators, PointXYZRGB)
{
  using namespace pcl::common;
  PointXYZRGB p0; p0.x = 0.1f; p0.y = 0.2f;  p0.z = 0.3f; p0.r = 123; p0.g = 125; p0.b = 127;
  PointXYZRGB p1; p1.x = 0.05f; p1.y = 0.05f; p1.z = 0.05f; p1.r = 123; p1.g = 125; p1.b = 127;
  PointXYZRGB p2 = p0 + p1;
  PointXYZRGB p3 = p0 - p1;

  EXPECT_EQ (p2.x, p0.x + p1.x);
  EXPECT_EQ (p2.y, p0.y + p1.y);
  EXPECT_EQ (p2.z, p0.z + p1.z);
  // Disabled. Doesn't make any sense
  //EXPECT_EQ (p2.r, p0.r + p1.r);
  //EXPECT_EQ (p2.g, p0.g + p1.g);
  //EXPECT_EQ (p2.b, p0.b + p1.b);

  EXPECT_EQ (p3.x, p0.x - p1.x);
  EXPECT_EQ (p3.y, p0.y - p1.y);
  EXPECT_EQ (p3.z, p0.z - p1.z);
  // Disabled. Doesn't make any sense
  //EXPECT_EQ (p3.r, p0.r - p1.r);
  //EXPECT_EQ (p3.g, p0.g - p1.g);
  //EXPECT_EQ (p3.b, p0.b - p1.b);


  p2 = 0.1f * p1;
  EXPECT_NEAR (p2.x, 0.1 * p1.x, 1e-4);
  EXPECT_NEAR (p2.y, 0.1 * p1.y, 1e-4);
  EXPECT_NEAR (p2.z, 0.1 * p1.z, 1e-4);
  // Disabled. Doesn't make any sense
  //EXPECT_EQ (p2.r, static_cast<pcl::uint8_t> (0.1 * p1.r));
  //EXPECT_EQ (p2.g, static_cast<pcl::uint8_t> (0.1 * p1.g));
  //EXPECT_EQ (p2.b, static_cast<pcl::uint8_t> (0.1 * p1.b));
  PointXYZRGB p4 = p1 * 0.1f;
  EXPECT_EQ_VECTORS (p2.getVector3fMap (), p4.getVector3fMap ());
  // Disabled. Doesn't make any sense
  //EXPECT_EQ (p2.r, p4.r);
  //EXPECT_EQ (p2.g, p4.g);
  //EXPECT_EQ (p2.b, p4.b);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
