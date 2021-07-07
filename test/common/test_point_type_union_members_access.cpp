/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>

#include <gmock/gmock.h>

using namespace pcl;
using namespace pcl::test;

template <typename PointT>
bool isDataMemberDefault(PointT &&p)
{
  return p.data[0] == 0.0f && p.data[1] == 0.0f && p.data[2] == 0.0f && p.data[3] == 1.0f;
}

template <typename T> class PointTypeConstexprConstructorTest : public testing::Test { };

TEST (PointTypeUnionMembersEq, PointXYZRGBAUnionMembersEquivalenceTest)
{
  for (const auto &pt : { pcl::PointXYZRGBA{}, pcl::PointXYZRGBA{2.0f, 3.0f, 4.0f, 2u, 3u, 4u, 5u} })
  {
    EXPECT_FLOAT_EQ(pt.data[0], pt.x);
    EXPECT_FLOAT_EQ(pt.data[1], pt.y);
    EXPECT_FLOAT_EQ(pt.data[2], pt.z);
    EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
    //const auto rgb = (static_cast<int>(pt.r) << 16) | (static_cast<int>(pt.g) << 8) | (static_cast<int>(pt.b));
    //EXPECT_EQ(pt.rgb, rgb);
  }
}

/*TEST (PointTypeConstruction, PointXYZRGBDefaultConstruction)
{
  constexpr const pcl::PointXYZRGB pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.r == 0u, "");
  static_assert(pt.g == 0u, "");
  static_assert(pt.b == 0u, "");
}

TEST (PointTypeConstruction, PointXYZRGBSixScalarsConstruction)
{
  constexpr const pcl::PointXYZRGB pt(2.0f, 3.0f, 4.0f, 2u, 3u, 4u);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.r == 2u, "");
  static_assert(pt.g == 3u, "");
  static_assert(pt.b == 4u, "");
}

TEST (PointTypeConstruction, PointXYZRGBLDefaultConstruction)
{
  constexpr const pcl::PointXYZRGBL pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.r == 0u, "");
  static_assert(pt.g == 0u, "");
  static_assert(pt.b == 0u, "");
  static_assert(pt.label == 0u, "");
}

TEST (PointTypeConstruction, PointXYZRGBLSevenScalarsConstruction)
{
  constexpr const pcl::PointXYZRGBL pt(2.0f, 3.0f, 4.0f, 2u, 3u, 4u, 5u);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.r == 2u, "");
  static_assert(pt.g == 3u, "");
  static_assert(pt.b == 4u, "");
  static_assert(pt.label == 5u, "");
}

TEST (PointTypeConstruction, PointXYZLABDefaultConstruction)
{
  constexpr const pcl::PointXYZLAB pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_lab[0] == 0.0f, "");
  static_assert(pt.data_lab[1] == 0.0f, "");
  static_assert(pt.data_lab[2] == 0.0f, "");
  static_assert(pt.data_lab[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZLABSixScalarsConstruction)
{
  constexpr const pcl::PointXYZLAB pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_lab[0] == 5.0f, "");
  static_assert(pt.data_lab[1] == 6.0f, "");
  static_assert(pt.data_lab[2] == 7.0f, "");
  static_assert(pt.data_lab[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZHSVDefaultConstruction)
{
  constexpr const pcl::PointXYZHSV pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 0.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZHSVSixScalarsConstruction)
{
  constexpr const pcl::PointXYZHSV pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 5.0f, "");
  static_assert(pt.data_c[1] == 6.0f, "");
  static_assert(pt.data_c[2] == 7.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}*/


/*TEST (PointTypeConstruction, PointXYZRGBNormalDefaultConstruction)
{
  constexpr const pcl::PointXYZRGBNormal pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 0.0f, "");
  static_assert(pt.data_n[1] == 0.0f, "");
  static_assert(pt.data_n[2] == 0.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
  static_assert(pt.data_c[0] == 0.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZRGBNormalTenScalarsConstruction)
{
  constexpr const pcl::PointXYZRGBNormal pt(2.0f, 3.0f, 4.0f, 5u, 6u, 7u, 8.0f, 9.0f, 10.0f, 11.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 8.0f, "");
  static_assert(pt.data_n[1] == 9.0f, "");
  static_assert(pt.data_n[2] == 10.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
  static_assert(pt.data_c[0] == 7.0f, "");
  static_assert(pt.data_c[1] == 6.0f, "");
  static_assert(pt.data_c[2] == 5.0f, "");
  static_assert(pt.data_c[3] == 11.0f, "");
}

TEST (PointTypeConstruction, PointXYZINormalDefaultConstruction)
{
  constexpr const pcl::PointXYZINormal pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 0.0f, "");
  static_assert(pt.data_n[1] == 0.0f, "");
  static_assert(pt.data_n[2] == 0.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
  static_assert(pt.data_c[0] == 0.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZINormalTenScalarsConstruction)
{
  constexpr const pcl::PointXYZINormal pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 6.0f, "");
  static_assert(pt.data_n[1] == 7.0f, "");
  static_assert(pt.data_n[2] == 8.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
  static_assert(pt.data_c[0] == 5.0f, "");
  static_assert(pt.data_c[1] == 9.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}*/

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
