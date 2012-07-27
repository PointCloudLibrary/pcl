/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_tests.h>

using namespace pcl;
using namespace pcl::test;

PointCloud<PointXYZ> cloud;
const size_t size = 10 * 480;

TEST (PointCloud, size)
{
  EXPECT_EQ(cloud.points.size (), cloud.size ());
}

TEST (PointCloud, sq_brackets_wrapper)
{
  for (uint32_t i = 0; i < size; ++i)
    EXPECT_EQ_VECTORS (cloud.points[i].getVector3fMap (),
                       cloud[i].getVector3fMap ());
}

TEST (PointCloud, at)
{
  for (uint32_t i = 0; i < size; ++i)
    EXPECT_EQ_VECTORS (cloud.points.at (i).getVector3fMap (),
                       cloud.at (i).getVector3fMap ());
}

TEST (PointCloud, front)
{
  EXPECT_EQ_VECTORS (cloud.points.front ().getVector3fMap (),
                     cloud.front ().getVector3fMap ());
}

TEST (PointCloud, back)
{
  EXPECT_EQ_VECTORS (cloud.points.back ().getVector3fMap (),
                     cloud.back ().getVector3fMap ());
}

TEST (PointCloud, constructor_with_allocation)
{
  PointCloud<PointXYZ> cloud2 (5, 80);
  EXPECT_EQ (cloud2.width, 5);
  EXPECT_EQ (cloud2.height, 80);
  EXPECT_EQ (cloud2.size (), 5*80);
}

TEST (PointCloud, constructor_with_allocation_valued)
{
  PointXYZ nan_point (0.1f, 0.2f, 0.3f);  
  PointCloud<PointXYZ> cloud2 (5, 80, nan_point);
  EXPECT_EQ (cloud2.width, 5);
  EXPECT_EQ (cloud2.height, 80);
  EXPECT_EQ (cloud2.size (), 5*80);
  for (PointCloud<PointXYZ>::const_iterator pit = cloud2.begin ();
       pit != cloud2.end ();
       ++pit)
  {
    EXPECT_NEAR (pit->x, 0.1, 1e-3);
    EXPECT_NEAR (pit->y, 0.2, 1e-3);
    EXPECT_NEAR (pit->z, 0.3, 1e-3);
  }
  
}

TEST (PointCloud, iterators)
{
  EXPECT_EQ_VECTORS (cloud.begin ()->getVector3fMap (), 
                     cloud.points.begin ()->getVector3fMap ());
  EXPECT_EQ_VECTORS (cloud.end ()->getVector3fMap (), 
                     cloud.points.end ()->getVector3fMap ());
  PointCloud<PointXYZ>::const_iterator pit = cloud.begin ();
  PointCloud<PointXYZ>::VectorType::const_iterator pit2 = cloud.points.begin ();
  for (; pit < cloud.end (); ++pit2, ++pit)
    EXPECT_EQ_VECTORS (pit->getVector3fMap (), pit2->getVector3fMap ());
}

TEST (PointCloud, insert_range)
{
  PointCloud<PointXYZ> cloud2 (10, 1);
  for (uint32_t i = 0; i < 10; ++i)
    cloud2[i] = PointXYZ (5.0f * static_cast<float>(i) + 0, 5.0f * static_cast<float> (i) + 1, 5.0f * static_cast<float> (i) + 2);

  uint32_t old_size = static_cast<uint32_t> (cloud.size ());
  cloud.insert (cloud.begin (), cloud2.begin (), cloud2.end ());
  EXPECT_EQ (cloud.width, cloud.size ());
  EXPECT_EQ (cloud.height, 1);
  EXPECT_EQ (cloud.width, old_size + cloud2.size ());
  PointCloud<PointXYZ>::const_iterator pit = cloud.begin ();
  PointCloud<PointXYZ>::const_iterator pit2 = cloud2.begin ();
  for (; pit2 < cloud2.end (); ++pit2, ++pit)
    EXPECT_EQ_VECTORS (pit->getVector3fMap (), pit2->getVector3fMap ());
}

int
main (int argc, char** argv)
{
  cloud.width = 10;
  cloud.height = 480;
  for (uint32_t i = 0; i < size; ++i)
    cloud.points.push_back (PointXYZ (3.0f * static_cast<float>(i) + 0, 3.0f * static_cast<float> (i) + 1, 3.0f * static_cast<float> (i) + 2));

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
