/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Fei Yan
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
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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
 */

/**
  * \file test_bearing_angle_image.cpp
  * \created on: July 07, 2013
  * \author: Qinghua Li (qinghua__li@163.com)
  */

#include <gtest/gtest.h>
#include <iostream>
#include <pcl/range_image/bearing_angle_image.h>

pcl::BearingAngleImage bearing_angle_image;
pcl::PointCloud<pcl::PointXYZ> point_cloud (3, 2);

/////////////////////////////////////////////////////////////////////
TEST (BearingAngleImageTest, GetAngle)
{
  pcl::PointXYZ point1 (1.0, 2.0, 3.0);
  pcl::PointXYZ point2 (2.0, 1.0, 1.0);

  double angle = bearing_angle_image.getAngle (point1, point2);
  EXPECT_NEAR (40.203, angle, 1e-3);
}

/////////////////////////////////////////////////////////////////////
TEST (BearingAngleImageTest, GenerateBAImage)
{
  point_cloud.points[0] = pcl::PointXYZ (3.0, 1.5, -2.0);
  point_cloud.points[1] = pcl::PointXYZ (1.0, 3.0, 2.0);
  point_cloud.points[2] = pcl::PointXYZ (2.0, 3.0, 2.0);

  point_cloud.points[3] = pcl::PointXYZ (2.0, 3.0, 1.0);
  point_cloud.points[4] = pcl::PointXYZ (4.0, 2.0, 2.0);
  point_cloud.points[5] = pcl::PointXYZ (-1.5, 3.0, 1.0);

  bearing_angle_image.generateBAImage (point_cloud);

  uint8_t grays[6];
  for (int i = 0; i < 3 * 2; ++i)
  {
    grays[i] = (bearing_angle_image.points[i].rgba >> 8) & 0xff;
  }

  EXPECT_EQ (0, grays[0]);
  EXPECT_EQ (0, grays[1]);
  EXPECT_EQ (0, grays[2]);
  EXPECT_EQ (112, grays[3]);
  EXPECT_EQ (80, grays[4]);
  EXPECT_EQ (0, grays[5]);
}


/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
