/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#include <pcl/test/gtest.h>

#include <pcl/common/copy_point.h>
#include <pcl/point_types.h>

TEST (CopyPointTest, SameTypeWithoutColor)
{
  {
    pcl::PointXYZ p1 (1, 2, 3), p2 (4, 5, 6);
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
  }
  {
    pcl::Normal p1 (1, 2, 3), p2 (4, 5, 6);
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.normal_x, p2.normal_x);
    EXPECT_FLOAT_EQ (p1.normal_y, p2.normal_y);
    EXPECT_FLOAT_EQ (p1.normal_z, p2.normal_z);
  }
}

TEST (CopyPointTest, SameTypeWithColor)
{
  {
    pcl::PointXYZRGBA p1, p2;
    p1.getVector3fMap () << 1, 2, 3; p1.rgba = 0xFF0000FF;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
    EXPECT_EQ (p1.rgba, p2.rgba);
  }
}

TEST (CopyPointTest, DifferentTypesWithoutColor)
{
  {
    pcl::PointXYZ p1 (1, 2, 3);
    pcl::PointXYZL p2;
    p2.getVector3fMap () << 4, 5, 5; p2.label = 1;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
    EXPECT_EQ (1, p2.label);
  }
  {
    pcl::PointXY p1; p1.x = 1; p1.y = 2;
    pcl::PointWithRange p2; p2.getVector3fMap () << 4, 5, 6; p2.range = 8;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (6, p2.z);
    EXPECT_FLOAT_EQ (8, p2.range);
  }
}

TEST (CopyPointTest, DifferentTypesOneWithColorAnotherWithout)
{
  // Source without color
  {
    pcl::PointXYZ p1 (1, 2, 3);
    pcl::PointXYZRGB p2;
    p2.getVector3fMap () << 4, 5, 5; p2.rgba = 0xFFFF00;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
    EXPECT_EQ (0xFFFF00, p2.rgba);
  }
  // Target without color
  {
    pcl::PointXYZRGBNormal p1; p1.getVector3fMap () << 1, 2, 3; p1.rgba = 0xFF0000;
    pcl::PointWithRange p2; p2.getVector3fMap () << 4, 5, 6; p2.range = 8;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
    EXPECT_FLOAT_EQ (8, p2.range);
  }
}

TEST (CopyPointTest, DifferentTypesWithDifferentColor)
{
  {
    pcl::RGB p1;
    pcl::PointXYZRGB p2; p2.getVector3fMap () << 4, 5, 6; p2.r = 7; p2.g = 8; p2.b = 9, p2.a = 10;
    pcl::copyPoint (p1, p2);
    EXPECT_EQ (p1.rgba, p2.rgba);
    EXPECT_FLOAT_EQ (4, p2.x);
    EXPECT_FLOAT_EQ (5, p2.y);
    EXPECT_FLOAT_EQ (6, p2.z);
  }
  {
    pcl::PointXYZRGBNormal p1; p1.getVector3fMap () << 1, 2, 3; p1.r = 7; p1.g = 8; p1.b = 9;
    pcl::PointXYZRGBL p2; p2.getVector3fMap () << 4, 5, 6; p2.r = 3; p2.g = 2; p2.b = 1; p2.label = 8;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
    EXPECT_EQ (p1.rgba, p2.rgba);
    EXPECT_EQ (8, p2.label);
  }
  {
    pcl::PointXYZRGBA p1; p1.getVector3fMap () << 1, 2, 3; p1.rgba = 0xFF00FF;
    pcl::PointXYZRGB p2; p2.getVector3fMap () << 4, 5, 6; p2.rgba = 0x00FF00;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
    EXPECT_EQ (p1.rgba, p2.rgba);
  }
}

TEST (CopyPointTest, DifferentTypesWithSameColor)
{
  {
    pcl::RGB p1;
    pcl::PointXYZRGBA p2; p2.getVector3fMap () << 4, 5, 6; p2.r = 7; p2.g = 8; p2.b = 9, p2.a = 10;
    pcl::copyPoint (p1, p2);
    EXPECT_EQ (p1.rgba, p2.rgba);
    EXPECT_FLOAT_EQ (4, p2.x);
    EXPECT_FLOAT_EQ (5, p2.y);
    EXPECT_FLOAT_EQ (6, p2.z);
  }
  {
    pcl::PointXYZRGBNormal p1; p1.getVector3fMap () << 1, 2, 3; p1.r = 7; p1.g = 8; p1.b = 9;
    pcl::PointXYZRGB p2; p2.getVector3fMap () << 4, 5, 6; p2.r = 3; p2.g = 2; p2.b = 1;
    pcl::copyPoint (p1, p2);
    EXPECT_FLOAT_EQ (p1.x, p2.x);
    EXPECT_FLOAT_EQ (p1.y, p2.y);
    EXPECT_FLOAT_EQ (p1.z, p2.z);
    EXPECT_EQ (p1.rgba, p2.rgba);
  }
}

namespace custom {
  struct MyPoint {
    float x, y, z, w;
    double intensity;
  };
  struct MyPoint2 {
    double x, y, z, w;
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT (custom::MyPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_STRUCT (custom::MyPoint2,
    (double, x, x)
    (double, y, y)
    (double, z, z)
)

TEST(CopyPointTest, SameFieldNameDifferentDatatype)
{
  // in p1 the intensity field is double, while in p2 it is float
  custom::MyPoint p1;
  pcl::PointXYZI p2;
  p1.x = 1.0;
  p1.y = 2.0;
  p1.z = 3.0;
  p1.intensity = 5.0;
  pcl::copyPoint(p1, p2);
  EXPECT_EQ(p1.x, p2.x);
  EXPECT_EQ(p1.y, p2.y);
  EXPECT_EQ(p1.z, p2.z);
  EXPECT_EQ(p1.intensity, p2.intensity);

  // in p3 xyz are float, while in p4 they are double
  pcl::PointXYZ p3;
  custom::MyPoint2 p4;
  p3.x = 10.0;
  p3.y = 11.0;
  p3.z = 12.0;
  pcl::copyPoint(p3, p4);
  EXPECT_EQ(p3.x, p4.x);
  EXPECT_EQ(p3.y, p4.y);
  EXPECT_EQ(p3.z, p4.z);
}

int
main (int argc, char **argv)
{
  try
  {
    ::testing::InitGoogleTest (&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS ();
  }
  catch (std::exception& e)
  {
    std::cerr << "Unhandled exception: " << e.what () << "\n";
  }
  return 1;
}

