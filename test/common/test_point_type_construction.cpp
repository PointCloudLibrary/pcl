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

using namespace pcl;
using namespace pcl::test;

/*template <typename PointT>
bool isDataMemberDefault(PointT &&p)
{
  return p.data[0] == 0.0f && p.data[1] == 0.0f && p.data[2] == 0.0f && p.data[3] == 1.0f;
}*/

template <typename T> class PointTypeConstexprConstructorTest : public testing::Test { };

TEST (PointTypeConstruction, PointXYZDefaultConstruction)
{
  constexpr const pcl::PointXYZ pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
}

TEST (PointTypeConstruction, PointXYZThreeScalarsConstruction)
{
  constexpr const pcl::PointXYZ pt(2.0f, 3.0f, 4.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
}

TEST (PointTypeConstruction, PointXYZIDefaultConstruction)
{
  constexpr const pcl::PointXYZI pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.intensity == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZIFourScalarsConstruction)
{
  constexpr const pcl::PointXYZI pt(2.0f, 3.0f, 4.0f, 5.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.intensity == 5.0f, "");
}

TEST (PointTypeConstruction, PointXYZLDefaultConstruction)
{
  constexpr const pcl::PointXYZL pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.label == 0.0f, "");
}

TEST (PointTypeConstruction, PointXYZLFourScalarsConstruction)
{
  constexpr const pcl::PointXYZL pt(2.0f, 3.0f, 4.0f, 5.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.label == 5.0f, "");
}

TEST (PointTypeConstruction, IntensityDefaultConstruction)
{
  constexpr const pcl::Intensity pt;
  static_assert(pt.intensity == 0.0f, "");
}

TEST (PointTypeConstruction, IntensityOneScalarConstruction)
{
  constexpr const pcl::Intensity pt(1.0f);
  static_assert(pt.intensity == 1.0f, "");
}

TEST (PointTypeConstruction, Intensity8uDefaultConstruction)
{
  constexpr const pcl::Intensity8u pt;
  static_assert(pt.intensity == std::uint8_t{}, "");
}

TEST (PointTypeConstruction, Intensity8uOneScalarConstruction)
{
  constexpr const pcl::Intensity8u pt(1u);
  static_assert(pt.intensity == std::uint8_t{1u}, "");
}

TEST (PointTypeConstruction, Intensity32uDefaultConstruction)
{
  constexpr const pcl::Intensity8u pt;
  static_assert(pt.intensity == std::uint32_t{}, "");
}

TEST (PointTypeConstruction, Intensity32uOneScalarConstruction)
{
  constexpr const pcl::Intensity32u pt(1u);
  static_assert(pt.intensity == std::uint32_t{1u}, "");
}

using AllPointTypes = ::testing::Types<pcl::PointXYZ, pcl::RGB, pcl::Intensity, pcl::PointXYZI>;
TYPED_TEST_SUITE (PointTypeConstexprConstructorTest, AllPointTypes);

TYPED_TEST (PointTypeConstexprConstructorTest, ConstexprDefaultConstructionTests)
{
  static_assert(std::is_default_constructible<TypeParam>::value, "");
}

TEST (PointTypeConstruction, LabelDefaultConstruction)
{
  constexpr const pcl::Label pt;
  static_assert(pt.label == 0.0f, "");
}

TEST (PointTypeConstruction, LabelOneScalarConstruction)
{
  constexpr const pcl::Label pt(1.0f);
  static_assert(pt.label == 1.0f, "");
}

TEST (PointTypeConstruction, PointXYZRGBADefaultConstruction)
{
  constexpr const pcl::PointXYZRGBA pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.r == 0u, "");
  static_assert(pt.g == 0u, "");
  static_assert(pt.b == 0u, "");
  static_assert(pt.a == 255u, "");
}

TEST (PointTypeConstruction, PointXYZRGBASevenScalarsConstruction)
{
  constexpr const pcl::PointXYZRGBA pt(2.0f, 3.0f, 4.0f, 2u, 3u, 4u, 5u);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.r == 2u, "");
  static_assert(pt.g == 3u, "");
  static_assert(pt.b == 4u, "");
  static_assert(pt.a == 5u, "");
}

TEST (PointTypeConstruction, PointXYZRGBDefaultConstruction)
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
}

TEST (PointTypeConstruction, PointXYDefaultConstruction)
{
  constexpr const pcl::PointXY pt;
  static_assert(pt.x == 0.0f, "");
  static_assert(pt.y == 0.0f, ""); 
}

TEST (PointTypeConstruction, PointXYTwoScalarsConstruction)
{
  constexpr const pcl::PointXY pt(1.0f, 2.0f);
  static_assert(pt.x == 1.0f, "");
  static_assert(pt.y == 2.0f, "");
}

TEST (PointTypeConstruction, PointUVDefaultConstruction)
{
  constexpr const pcl::PointUV pt;
  static_assert(pt.u == 0.0f, "");
  static_assert(pt.v == 0.0f, ""); 
}

TEST (PointTypeConstruction, PointUVTwoScalarsConstruction)
{
  constexpr const pcl::PointUV pt(1.0f, 2.0f);
  static_assert(pt.u == 1.0f, "");
  static_assert(pt.v == 2.0f, "");
}

TEST (PointTypeConstruction, NormalDefaultConstruction)
{
  constexpr const pcl::Normal pt;
  static_assert(pt.data_n[0] == 0.0f, "");
  static_assert(pt.data_n[1] == 0.0f, "");
  static_assert(pt.data_n[2] == 0.0f, "");
  static_assert(pt.data_n[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 0.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, NormalFourScalarsConstruction)
{
  constexpr const pcl::Normal pt(2.0f, 3.0f, 4.0f, 5.0f);
  static_assert(pt.data_n[0] == 2.0f, "");
  static_assert(pt.data_n[1] == 3.0f, "");
  static_assert(pt.data_n[2] == 4.0f, "");
  static_assert(pt.data_n[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 5.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, AxisDefaultConstruction)
{
  constexpr const pcl::Axis pt;
  static_assert(pt.data_n[0] == 0.0f, "");
  static_assert(pt.data_n[1] == 0.0f, "");
  static_assert(pt.data_n[2] == 0.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
}

TEST (PointTypeConstruction, AxisThreeScalarsConstruction)
{
  constexpr const pcl::Axis pt(2.0f, 3.0f, 4.0f);
  static_assert(pt.data_n[0] == 2.0f, "");
  static_assert(pt.data_n[1] == 3.0f, "");
  static_assert(pt.data_n[2] == 4.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointNormalDefaultConstruction)
{
  constexpr const pcl::PointNormal pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 0.0f, "");
  static_assert(pt.data_n[1] == 0.0f, "");
  static_assert(pt.data_n[2] == 0.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointNormalSixScalarsConstruction)
{
  constexpr const pcl::PointNormal pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 5.0f, "");
  static_assert(pt.data_n[1] == 6.0f, "");
  static_assert(pt.data_n[2] == 7.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
}


TEST (PointTypeConstruction, PointXYZRGBNormalDefaultConstruction)
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
  static_assert(pt.r == 0u, "");
  static_assert(pt.g == 0u, "");
  static_assert(pt.b == 0u, "");
  static_assert(pt.a == 255u, "");
  static_assert(pt.curvature == 0.0f, "");
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
  static_assert(pt.r == 5u, "");
  static_assert(pt.g == 6u, "");
  static_assert(pt.b == 7u, "");
  static_assert(pt.curvature == 11.0f, "");
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
}

TEST (PointTypeConstruction, PointXYZLNormalDefaultConstruction)
{
  constexpr const pcl::PointXYZLNormal pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 0.0f, "");
  static_assert(pt.data_n[1] == 0.0f, "");
  static_assert(pt.data_n[2] == 0.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
  static_assert(pt.data_c[0] == 0u, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");

}

TEST (PointTypeConstruction, PointXYZLNormalTenScalarsConstruction)
{
  constexpr const pcl::PointXYZLNormal pt(2.0f, 3.0f, 4.0f, 5u, 6.0f, 7.0f, 8.0f, 9.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 6.0f, "");
  static_assert(pt.data_n[1] == 7.0f, "");
  static_assert(pt.data_n[2] == 8.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
  static_assert(pt.data_c[0] == 5u, "");
  static_assert(pt.data_c[1] == 9.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointWithRangeDefaultConstruction)
{
  constexpr const pcl::PointWithRange pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 0.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointWithRangeFourScalarsConstruction)
{
  constexpr const pcl::PointWithRange pt(2.0f, 3.0f, 4.0f, 5.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 5.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointWithViewpointDefaultConstruction)
{
  constexpr const pcl::PointWithViewpoint pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 0.0f, "");
  static_assert(pt.data_c[1] == 0.0f, "");
  static_assert(pt.data_c[2] == 0.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, PointWithViewpointSixScalarsConstruction)
{
  constexpr const pcl::PointWithViewpoint pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  static_assert(pt.data[0] == 2.0f, "");
  static_assert(pt.data[1] == 3.0f, "");
  static_assert(pt.data[2] == 4.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_c[0] == 5.0f, "");
  static_assert(pt.data_c[1] == 6.0f, "");
  static_assert(pt.data_c[2] == 7.0f, "");
  static_assert(pt.data_c[3] == 0.0f, "");
}

TEST (PointTypeConstruction, MomentInvariantsDefaultConstruction)
{
  constexpr const pcl::MomentInvariants pt;
  static_assert(pt.j1 == 0.0f, "");
  static_assert(pt.j2 == 0.0f, ""); 
  static_assert(pt.j3 == 0.0f, ""); 
}

TEST (PointTypeConstruction, MomentInvariantsThreeScalarsConstruction)
{
  constexpr const pcl::MomentInvariants pt(1.0f, 2.0f, 3.0f);
  static_assert(pt.j1 == 1.0f, "");
  static_assert(pt.j2 == 2.0f, "");
  static_assert(pt.j3 == 3.0f, "");
}

TEST (PointTypeConstruction, PrincipalRadiiRSDDefaultConstruction)
{
  constexpr const pcl::PrincipalRadiiRSD pt;
  static_assert(pt.r_min == 0.0f, "");
  static_assert(pt.r_max == 0.0f, ""); 
}

TEST (PointTypeConstruction, PrincipalRadiiRSDScalarsConstruction)
{
  constexpr const pcl::PrincipalRadiiRSD pt(1.0f, 2.0f);
  static_assert(pt.r_min == 1.0f, "");
  static_assert(pt.r_max == 2.0f, "");
}

TEST (PointTypeConstruction, BoundaryDefaultConstruction)
{
  constexpr const pcl::Boundary pt;
  static_assert(pt.boundary_point == std::uint8_t{}, "");
}

TEST (PointTypeConstruction, BoundaryOneScalarConstruction)
{
  constexpr const pcl::Boundary pt(1u);
  static_assert(pt.boundary_point == std::uint8_t{1u}, "");
}

TEST (PointTypeConstruction, PrincipalCurvaturesDefaultConstruction)
{
  constexpr const pcl::PrincipalCurvatures pt;
  static_assert(pt.principal_curvature_x == 0.0f, "");
  static_assert(pt.principal_curvature_y == 0.0f, "");
  static_assert(pt.principal_curvature_z == 0.0f, "");
  static_assert(pt.pc1 == 0.0f, "");
  static_assert(pt.pc2 == 0.0f, "");
}

TEST (PointTypeConstruction, PrincipalCurvaturesFiveScalarsConstruction)
{
  constexpr const pcl::PrincipalCurvatures pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  static_assert(pt.principal_curvature_x == 2.0f, "");
  static_assert(pt.principal_curvature_y == 3.0f, "");
  static_assert(pt.principal_curvature_z == 4.0f, "");
  static_assert(pt.pc1 == 5.0f, "");
  static_assert(pt.pc2 == 6.0f, "");
}

TEST (PointTypeConstruction, ReferenceFrameDefaultConstruction)
{
  constexpr const pcl::ReferenceFrame pt;
  static_assert(pt.rf[0] == 0.0f, "");
  static_assert(pt.rf[1] == 0.0f, "");
  static_assert(pt.rf[2] == 0.0f, "");
  static_assert(pt.rf[3] == 0.0f, "");
  static_assert(pt.rf[4] == 0.0f, "");
  static_assert(pt.rf[5] == 0.0f, "");
  static_assert(pt.rf[6] == 0.0f, "");
  static_assert(pt.rf[7] == 0.0f, "");
  static_assert(pt.rf[8] == 0.0f, "");
}

TEST (PointTypeConstruction, ReferenceFrameArrayOfScalarsConstruction)
{
  constexpr const float values[9]{ 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f };
  constexpr const pcl::ReferenceFrame pt(values);
  static_assert(pt.rf[0] == values[0], "");
  static_assert(pt.rf[1] == values[1], "");
  static_assert(pt.rf[2] == values[2], "");
  static_assert(pt.rf[3] == values[3], "");
  static_assert(pt.rf[4] == values[4], "");
  static_assert(pt.rf[5] == values[5], "");
  static_assert(pt.rf[6] == values[6], "");
  static_assert(pt.rf[7] == values[7], "");
  static_assert(pt.rf[8] == values[8], "");
}

// TODO
/*TEST (PointTypeConstruction, FPFHSignature33DefaultConstruction)
{
  constexpr const pcl::FPFHSignature33 pt;
  for (auto i{0u}; i < 33u; ++i)
  {
    static_assert(pt.histogram[i] == 0.0f, "");
  }
}*/

// TODO compile-time for-looped check
TEST (PointTypeConstruction, VFHSignature308DefaultConstruction)
{
  //constexpr const pcl::VFHSignature308 pt;
}

// TODO compile-time for-looped check
TEST (PointTypeConstruction, GRSDSignature21DefaultConstruction)
{
  //constexpr const pcl::GRSDSignature21 pt;
}

// TODO compile-time for-looped check
TEST (PointTypeConstruction, BRISKSignature512DefaultConstruction)
{
  //constexpr const pcl::BRISKSignature512 pt;
}

// TODO compile-time for-looped check
TEST (PointTypeConstruction, ESFSignature640DefaultConstruction)
{
  //constexpr const pcl::ESFSignature640 pt;
}

// TODO compile-time for-looped check
TEST (PointTypeConstruction, GASDSignature512DefaultConstruction)
{
  //constexpr const pcl::GASDSignature512 pt;
}

// TODO compile-time for-looped check
TEST (PointTypeConstruction, GASDSignature984DefaultConstruction)
{
  //constexpr const pcl::GASDSignature984 pt;
}

// TODO compile-time for-looped check
TEST (PointTypeConstruction, GASDSignature7992DefaultConstruction)
{
  //constexpr const pcl::GASDSignature7992 pt;
}

// TODO compile-time for-looped check
TEST (PointTypeConstruction, GFPFHSignature16DefaultConstruction)
{
  //constexpr const pcl::GFPFHSignature16 pt;
}

TEST (PointTypeConstruction, Narf36DefaultConstruction)
{
  constexpr const pcl::Narf36 pt;
  static_assert(pt.x == 0.0f, "");
  static_assert(pt.y == 0.0f, "");
  static_assert(pt.z == 0.0f, "");
  static_assert(pt.roll == 0.0f, "");
  static_assert(pt.pitch == 0.0f, "");
  static_assert(pt.yaw == 0.0f, "");
}

TEST (PointTypeConstruction, Narf36ThreeScalarsConstruction)
{
  constexpr const pcl::Narf36 pt{1.0f, 2.0f, 3.0f};
  static_assert(pt.x == 1.0f, "");
  static_assert(pt.y == 2.0f, "");
  static_assert(pt.z == 3.0f, "");
  static_assert(pt.roll == 0.0f, "");
  static_assert(pt.pitch == 0.0f, "");
  static_assert(pt.yaw == 0.0f, "");
}

TEST (PointTypeConstruction, Narf36SixScalarsConstruction)
{
  constexpr const pcl::Narf36 pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  static_assert(pt.x == 1.0f, "");
  static_assert(pt.y == 2.0f, "");
  static_assert(pt.z == 3.0f, "");
  static_assert(pt.roll == 4.0f, "");
  static_assert(pt.pitch == 5.0f, "");
  static_assert(pt.yaw == 6.0f, "");
}

TEST (PointTypeConstruction, BorderDescriptionDefaultConstruction)
{
  constexpr const pcl::BorderDescription pt;
  static_assert(pt.x == 0, "");
  static_assert(pt.y == 0, "");
}


TEST (PointTypeConstruction, BorderDescriptionTwoScalarsConstruction)
{
  constexpr const pcl::BorderDescription pt{1, 2};
  static_assert(pt.x == 1, "");
  static_assert(pt.y == 2, "");
}

TEST (PointTypeConstruction, IntensityGradientDefaultConstruction)
{
  constexpr const pcl::IntensityGradient pt;
  static_assert(pt.gradient_x == 0.0f, "");
  static_assert(pt.gradient_y == 0.0f, "");
  static_assert(pt.gradient_z == 0.0f, "");
}

TEST (PointTypeConstruction, IntensityGradientThreeScalarsConstruction)
{
  constexpr const pcl::IntensityGradient pt{1.0f, 2.0f, 3.0f};
  static_assert(pt.gradient_x == 1.0f, "");
  static_assert(pt.gradient_y == 2.0f, "");
  static_assert(pt.gradient_z == 3.0f, "");
}

TEST (PointTypeConstruction, PointWithScaleDefaultConstruction)
{
  constexpr const pcl::PointWithScale pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.scale == 1.0f, "");
  static_assert(pt.angle == -1.0f, "");
  static_assert(pt.response == 0.0f, "");
  static_assert(pt.octave == 0, "");
}

TEST (PointTypeConstruction, PointWithScaleSevenScalarsConstruction)
{
  constexpr const pcl::PointWithScale pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7};
  static_assert(pt.data[0] == 1.0f, "");
  static_assert(pt.data[1] == 2.0f, "");
  static_assert(pt.data[2] == 3.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.scale == 4.0f, "");
  static_assert(pt.angle == 5.0f, "");
  static_assert(pt.response == 6.0f, "");
  static_assert(pt.octave == 7, "");
}

TEST (PointTypeConstruction, PointSurfelDefaultConstruction)
{
  constexpr const pcl::PointSurfel pt;
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

TEST (PointTypeConstruction, PointSurfelTenScalarsConstruction)
{
  constexpr const pcl::PointSurfel pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7u, 8.0f, 9.0f, 10.0f};
  static_assert(pt.data[0] == 1.0f, "");
  static_assert(pt.data[1] == 2.0f, "");
  static_assert(pt.data[2] == 3.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.data_n[0] == 4.0f, "");
  static_assert(pt.data_n[1] == 5.0f, "");
  static_assert(pt.data_n[2] == 6.0f, "");
  static_assert(pt.data_n[3] == 0.0f, "");
  static_assert(pt.data_c[0] == 7u, "");
  static_assert(pt.data_c[1] == 8.0f, "");
  static_assert(pt.data_c[2] == 9.0f, "");
  static_assert(pt.data_c[3] == 10.0f, "");
}

TEST (PointTypeConstruction, PointDEMDefaultConstruction)
{
  constexpr const pcl::PointDEM pt;
  static_assert(pt.data[0] == 0.0f, "");
  static_assert(pt.data[1] == 0.0f, "");
  static_assert(pt.data[2] == 0.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.intensity == 0.0f, "");
  static_assert(pt.intensity_variance == 0.0f, "");
  static_assert(pt.height_variance == 0.0f, "");
}

TEST (PointTypeConstruction, PointDEMSixScalarsConstruction)
{
  constexpr const pcl::PointDEM pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  static_assert(pt.data[0] == 1.0f, "");
  static_assert(pt.data[1] == 2.0f, "");
  static_assert(pt.data[2] == 3.0f, "");
  static_assert(pt.data[3] == 1.0f, "");
  static_assert(pt.intensity == 4.0f, "");
  static_assert(pt.intensity_variance == 5.0f, "");
  static_assert(pt.height_variance == 6.0f, "");
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
