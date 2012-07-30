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
#include <pcl/common/random.h>
#include <pcl/common/generate.h>

TEST (UniformCloudGenerator, PointXYZ)
{
  using namespace pcl::common;
  CloudGenerator<pcl::PointXYZ,  UniformGenerator<float> > generator;
  UniformGenerator<float>::Parameters x_params;
  generator.setParametersForX (x_params);
  UniformGenerator<float>::Parameters y_params (-1.f, 1.f);
  generator.setParametersForY (y_params);
  UniformGenerator<float>::Parameters z_params (-2.5, 1.5f);
  generator.setParametersForZ (z_params);

  pcl::PointCloud<pcl::PointXYZ> output;
  int result = generator.fill (480, 640, output);
  EXPECT_EQ (result, 0);
  EXPECT_EQ (output.height, 640);
  EXPECT_EQ (output.width, 480);
  EXPECT_EQ (output.size (), 480*640);
  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator points_it = output.begin ();
      points_it != output.end ();
      ++points_it)
  {
    EXPECT_GE (points_it->x, 0);
    EXPECT_LT (points_it->x, 1);
    EXPECT_GE (points_it->y, -1);
    EXPECT_LT (points_it->y, 1);
    EXPECT_GE (points_it->z, -2.5);
    EXPECT_LT (points_it->z, 1.5);
  }
}

TEST (UniformCloudGenerator, PointXY)
{
  using namespace pcl::common;
  CloudGenerator<pcl::PointXY,  UniformGenerator<float> > generator;
  UniformGenerator<float>::Parameters x_params;
  generator.setParametersForX (x_params);
  UniformGenerator<float>::Parameters y_params (-1.f, 1.f);
  generator.setParametersForY (y_params);

  pcl::PointCloud<pcl::PointXY> output;
  int result = generator.fill (480, 640, output);
  EXPECT_EQ (result, 0);
  EXPECT_EQ (output.height, 640);
  EXPECT_EQ (output.width, 480);
  EXPECT_EQ (output.size (), 480*640);
  for(pcl::PointCloud<pcl::PointXY>::const_iterator points_it = output.begin ();
      points_it != output.end ();
      ++points_it)
  {
    EXPECT_GE (points_it->x, 0);
    EXPECT_LT (points_it->x, 1);
    EXPECT_GE (points_it->y, -1);
    EXPECT_LT (points_it->y, 1);
  }
}

TEST (UniformCloudGenerator, Cube)
{
  using namespace pcl::common;
  CloudGenerator<pcl::PointXYZ,  UniformGenerator<float> > generator;
  UniformGenerator<float>::Parameters params (-3, 3, 1);
  generator.setParameters (params);

  pcl::PointCloud<pcl::PointXYZ> output;
  int result = generator.fill (480, 640, output);
  EXPECT_EQ (result, 0);
  EXPECT_EQ (output.height, 640);
  EXPECT_EQ (output.width, 480);
  EXPECT_EQ (output.size (), 480*640);
  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator points_it = output.begin ();
      points_it != output.end ();
      ++points_it)
  {
    EXPECT_GE (points_it->x, -3);
    EXPECT_LT (points_it->x, 3);
    EXPECT_GE (points_it->y, -3);
    EXPECT_LT (points_it->y, 3);
    EXPECT_GE (points_it->z, -3);
    EXPECT_LT (points_it->z, 3);
  }
}

TEST (UniformCloudGenerator, Square)
{
  using namespace pcl::common;
  CloudGenerator<pcl::PointXY,  UniformGenerator<float> > generator;
  UniformGenerator<float>::Parameters params (-3, 3, 1);
  generator.setParameters (params);

  pcl::PointCloud<pcl::PointXY> output;
  int result = generator.fill (480, 640, output);
  EXPECT_EQ (result, 0);
  EXPECT_EQ (output.height, 640);
  EXPECT_EQ (output.width, 480);
  EXPECT_EQ (output.size (), 480*640);
  for(pcl::PointCloud<pcl::PointXY>::const_iterator points_it = output.begin ();
      points_it != output.end ();
      ++points_it)
  {
    EXPECT_GE (points_it->x, -3);
    EXPECT_LT (points_it->x, 3);
    EXPECT_GE (points_it->y, -3);
    EXPECT_LT (points_it->y, 3);
  }
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
