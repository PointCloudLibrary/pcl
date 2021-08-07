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
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> class XYZPointTypesTest : public ::testing::Test { };
using XYZPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM(PCL_XYZ_POINT_TYPES)>;
TYPED_TEST_SUITE(XYZPointTypesTest, XYZPointTypes);

TYPED_TEST(XYZPointTypesTest, Distance)
{
  TypeParam p1, p2;
  p1.x = 3; p1.y = 4; p1.z = 5;
  p2.y = 1; p2.x = 1; p2.z = 1.5;
  double distance = geometry::distance (p1, p2);
  EXPECT_NEAR (distance, 5.024938, 1e-4);
}

TYPED_TEST(XYZPointTypesTest, SquaredDistance)
{
  TypeParam p1, p2;
  p1.x = 3; p1.y = 4; p1.z = 5;
  p2.y = 1; p2.x = 1; p2.z = 1.5;
  double distance = geometry::squaredDistance (p1, p2);
  EXPECT_NEAR (distance, 25.25, 1e-4);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
