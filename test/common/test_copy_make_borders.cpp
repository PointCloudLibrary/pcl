/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Perception, Inc.
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
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_config.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

using namespace pcl;
using namespace pcl::test;

pcl::PointCloud<pcl::PointXYZ> cloud (64,48);
int top = 2, bottom = 2, left = 2, right = 2;
const pcl::PointXYZ constant (0, 0, 0);

TEST (CopyPointCloud, constant)
{
  pcl::PointCloud<pcl::PointXYZ> dst (cloud.width + left + right, cloud.height + top + bottom);
  pcl::copyPointCloud (cloud, dst, top, bottom, left, right, pcl::BORDER_CONSTANT, constant);

  for (int j = 0; j < top; ++j)
    for (std::uint32_t i = 0; i < dst.width; ++i)
      EXPECT_XYZ_EQ (dst (i,j), constant);

  for (unsigned int j = top; j < cloud.height+top; ++j)
  {
    for (std::uint32_t i = 0; i < dst.width; ++i)
    {
      if (static_cast<int> (i) < left)
        EXPECT_XYZ_EQ (dst (i,j), constant);
      else
      {
        if (i >= (cloud.width + left))
          EXPECT_XYZ_EQ (dst (i,j), constant);
        else
          EXPECT_XYZ_EQ (dst (i,j), cloud (i - left, j -top));
      }
    }
  }

  for (std::uint32_t j = cloud.height+top; j < dst.height; ++j)
    for (std::uint32_t i = 0; i < dst.width; ++i)
      EXPECT_XYZ_EQ (dst (i,j), constant);
}

TEST (CopyPointCloud, replicate)
{
  pcl::PointCloud<pcl::PointXYZ> dst (cloud.width + left + right, cloud.height + top + bottom);
  pcl::copyPointCloud (cloud, dst, top, bottom, left, right, pcl::BORDER_REPLICATE, constant);

  for (int j = 0; j < top; ++j)
  {
    for (int i = 0; i < left; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (0,0));
    for (unsigned int i = left; i < cloud.width+left; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (i-left,0));
    for (std::uint32_t i = cloud.width+left; i < dst.width; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (cloud.width-1,0));
  }

  for (unsigned int j = top; j < cloud.height+top; ++j)
  {
    for (std::uint32_t i = 0; i < dst.width; ++i)
    {
      if (static_cast<int> (i) < left)
        EXPECT_XYZ_EQ (dst (i,j), cloud (0,j-top));
      else
      {
        if (i >= (cloud.width + left))
          EXPECT_XYZ_EQ (dst (i,j), cloud (cloud.width-1,j-top));
        else
          EXPECT_XYZ_EQ (dst (i,j), cloud (i - left, j -top));
      }
    }
  }

  for (std::uint32_t j = cloud.height+top; j < dst.height; ++j)
  {
    for (int i = 0; i < left; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (0,cloud.height-1));
    for (unsigned int i = left; i < cloud.width+left; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (i-left,cloud.height-1));
    for (std::uint32_t i = cloud.width+left; i < dst.width; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (cloud.width-1,cloud.height-1));
  }
}

TEST (CopyPointCloud, reflect)
{
  pcl::PointCloud<pcl::PointXYZ> dst (cloud.width + left + right, cloud.height + top + bottom);
  pcl::copyPointCloud (cloud, dst, top, bottom, left, right, pcl::BORDER_REFLECT, constant);

  for (int j = 0, k = top-1; j < top; ++j,--k)
  {
    for (int i = 0, l = left-1; i < left; ++i, --l)
      EXPECT_XYZ_EQ (dst (i,j), cloud (l, k));

    for (unsigned int i = left; i < cloud.width+left; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (i-left,k));

    for (int i = cloud.width+left, l = cloud.width-left; i < left; ++i, --l)
      EXPECT_XYZ_EQ (dst (i,j), cloud (l, k));
  }

  for (unsigned int j = top; j < cloud.height+top; ++j)
  {
    for (int i = 0, l = left-1; i < left; ++i, --l)
      EXPECT_XYZ_EQ (dst (i,j), cloud (l, j-top));

    for (unsigned int i = left; i < cloud.width + left; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (i-left,j-top));

    for (int i = cloud.width+left, l = cloud.width-left; i < left; ++i, --l)
      EXPECT_XYZ_EQ (dst (i,j), cloud (l, j-top));
  }

  for (int j = cloud.height+top, k = cloud.height-1; j < top; ++j,--k)
  {
    for (int i = 0, l = left-1; i < left; ++i, --l)
      EXPECT_XYZ_EQ (dst (i,j), cloud (l, k));

    for (unsigned int i = left; i < cloud.width+left; ++i)
      EXPECT_XYZ_EQ (dst (i,j), cloud (i-left,k));

    for (int i = cloud.width+left, l = cloud.width-left; i < left; ++i, --l)
      EXPECT_XYZ_EQ (dst (i,j), cloud (l, k));
  }
}

int
main (int argc, char** argv)
{
  for (std::size_t i = 0, j = 10; i < cloud.size (); ++j, ++i)
    cloud[i] = pcl::PointXYZ (j, j*10, j*100);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
