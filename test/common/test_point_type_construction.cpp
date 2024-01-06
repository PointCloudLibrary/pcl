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

TEST (PointTypeConstruction, PointXYZDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZ pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
}

TEST (PointTypeConstruction, PointXYZThreeScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZ pt(2.0f, 3.0f, 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
}

TEST (PointTypeConstruction, PointXYZIDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZI pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 0.0f);
}

TEST (PointTypeConstruction, PointXYZIFourScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZI pt(2.0f, 3.0f, 4.0f, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 5.0f);
}

TEST (PointTypeConstruction, PointXYZLDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZL pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.label, 0u);
}

TEST (PointTypeConstruction, PointXYZLFourScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZL pt(2.0f, 3.0f, 4.0f, 5u);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.label, 5u);
}

TEST (PointTypeConstruction, IntensityDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Intensity pt;
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 0.0f);
}

TEST (PointTypeConstruction, IntensityOneScalarConstruction)
{
  PCL_CONSTEXPR const pcl::Intensity pt(1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 1.0f);
}

TEST (PointTypeConstruction, Intensity8uDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Intensity8u pt;
  PCL_EXPECT_INT_EQ(pt.intensity, std::uint8_t{});
}

TEST (PointTypeConstruction, Intensity8uOneScalarConstruction)
{
  PCL_CONSTEXPR const pcl::Intensity8u pt(1u);
  PCL_EXPECT_INT_EQ(pt.intensity, 1u);
}

TEST (PointTypeConstruction, Intensity32uDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Intensity32u pt;
  PCL_EXPECT_INT_EQ(pt.intensity, std::uint32_t{});
}

TEST (PointTypeConstruction, Intensity32uOneScalarConstruction)
{
  PCL_CONSTEXPR const pcl::Intensity32u pt(1u);
  PCL_EXPECT_INT_EQ(pt.intensity, std::uint32_t{1u});
}

TEST (PointTypeConstruction, LabelDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Label pt;
  PCL_EXPECT_INT_EQ(pt.label, 0u);
}

TEST (PointTypeConstruction, LabelOneScalarConstruction)
{
  PCL_CONSTEXPR const pcl::Label pt(1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.label, 1u);
}

TEST (PointTypeConstruction, PointXYZRGBADefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGBA pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.r, 0u);
  PCL_EXPECT_INT_EQ(pt.g, 0u);
  PCL_EXPECT_INT_EQ(pt.b, 0u);
  PCL_EXPECT_INT_EQ(pt.a, 255u);
}

TEST (PointTypeConstruction, PointXYZRGBASevenScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGBA pt(2.0f, 3.0f, 4.0f, 2u, 3u, 4u, 5u);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.r, 2u);
  PCL_EXPECT_INT_EQ(pt.g, 3u);
  PCL_EXPECT_INT_EQ(pt.b, 4u);
  PCL_EXPECT_INT_EQ(pt.a, 5u);
}

TEST (PointTypeConstruction, PointXYZRGBDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGB pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.r, 0u);
  PCL_EXPECT_INT_EQ(pt.g, 0u);
  PCL_EXPECT_INT_EQ(pt.b, 0u);
}

TEST (PointTypeConstruction, PointXYZRGBSixScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGB pt(2.0f, 3.0f, 4.0f, 2u, 3u, 4u);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.r, 2u);
  PCL_EXPECT_INT_EQ(pt.g, 3u);
  PCL_EXPECT_INT_EQ(pt.b, 4u);
}

TEST (PointTypeConstruction, PointXYZRGBLDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGBL pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.r, 0u);
  PCL_EXPECT_INT_EQ(pt.g, 0u);
  PCL_EXPECT_INT_EQ(pt.b, 0u);
  PCL_EXPECT_INT_EQ(pt.label, 0u);
}

TEST (PointTypeConstruction, PointXYZRGBLSevenScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGBL pt(2.0f, 3.0f, 4.0f, 2u, 3u, 4u, 5u);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_INT_EQ(pt.r, 2u);
  PCL_EXPECT_INT_EQ(pt.g, 3u);
  PCL_EXPECT_INT_EQ(pt.b, 4u);
  PCL_EXPECT_INT_EQ(pt.label, 5u);
}

TEST (PointTypeConstruction, PointXYZLABDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZLAB pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.L, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.a, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.b, 0.0f);
}

TEST (PointTypeConstruction, PointXYZLABSixScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZLAB pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.L, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.a, 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.b, 7.0f);
}

TEST (PointTypeConstruction, PointXYZHSVDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZHSV pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.h, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.s, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.v, 0.0f);
}

TEST (PointTypeConstruction, PointXYZHSVSixScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZHSV pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.h, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.s, 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.v, 7.0f);
}

TEST (PointTypeConstruction, PointXYDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXY pt;
  PCL_EXPECT_FLOAT_EQ(pt.x, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.y, 0.0f); 
}

TEST (PointTypeConstruction, PointXYTwoScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXY pt(1.0f, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.x, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.y, 2.0f);
}

TEST (PointTypeConstruction, PointUVDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointUV pt;
  PCL_EXPECT_FLOAT_EQ(pt.u, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.v, 0.0f); 
}

TEST (PointTypeConstruction, PointUVTwoScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointUV pt(1.0f, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.u, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.v, 2.0f);
}

TEST (PointTypeConstruction, NormalDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Normal pt;
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 0.0f);
}

TEST (PointTypeConstruction, NormalFourScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::Normal pt(2.0f, 3.0f, 4.0f, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 5.0f);
}

TEST (PointTypeConstruction, AxisDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Axis pt;
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
}

TEST (PointTypeConstruction, AxisThreeScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::Axis pt(2.0f, 3.0f, 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
}

TEST (PointTypeConstruction, PointNormalDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointNormal pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
}

TEST (PointTypeConstruction, PointNormalSixScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointNormal pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 7.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
}


TEST (PointTypeConstruction, PointXYZRGBNormalDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGBNormal pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_INT_EQ(pt.r, 0u);
  PCL_EXPECT_INT_EQ(pt.g, 0u);
  PCL_EXPECT_INT_EQ(pt.b, 0u);
  PCL_EXPECT_INT_EQ(pt.a, 255u);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 0.0f);
}

TEST (PointTypeConstruction, PointXYZRGBNormalTenScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZRGBNormal pt(2.0f, 3.0f, 4.0f, 5u, 6u, 7u, 8.0f, 9.0f, 10.0f, 11.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 8.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 9.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 10.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_INT_EQ(pt.r, 5u);
  PCL_EXPECT_INT_EQ(pt.g, 6u);
  PCL_EXPECT_INT_EQ(pt.b, 7u);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 11.0f);
}

TEST (PointTypeConstruction, PointXYZINormalDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZINormal pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 0.0f);
}

TEST (PointTypeConstruction, PointXYZINormalTenScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZINormal pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 7.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 8.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 9.0f);
}

TEST (PointTypeConstruction, PointXYZLNormalDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZLNormal pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_INT_EQ(pt.label, 0u);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 0.0f);

}

TEST (PointTypeConstruction, PointXYZLNormalTenScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointXYZLNormal pt(2.0f, 3.0f, 4.0f, 5u, 6.0f, 7.0f, 8.0f, 9.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 7.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 8.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_INT_EQ(pt.label, 5u);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 9.0f);
}

TEST (PointTypeConstruction, PointWithRangeDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointWithRange pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.range, 0.0f);
}

TEST (PointTypeConstruction, PointWithRangeFourScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointWithRange pt(2.0f, 3.0f, 4.0f, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.range, 5.0f);
}

TEST (PointTypeConstruction, PointWithViewpointDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointWithViewpoint pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.vp_x, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.vp_y, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.vp_z, 0.0f);
}

TEST (PointTypeConstruction, PointWithViewpointSixScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointWithViewpoint pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.vp_x, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.vp_y, 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.vp_z, 7.0f);
}

TEST (PointTypeConstruction, MomentInvariantsDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::MomentInvariants pt;
  PCL_EXPECT_FLOAT_EQ(pt.j1, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.j2, 0.0f); 
  PCL_EXPECT_FLOAT_EQ(pt.j3, 0.0f); 
}

TEST (PointTypeConstruction, MomentInvariantsThreeScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::MomentInvariants pt(1.0f, 2.0f, 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.j1, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.j2, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.j3, 3.0f);
}

TEST (PointTypeConstruction, PrincipalRadiiRSDDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PrincipalRadiiRSD pt;
  PCL_EXPECT_FLOAT_EQ(pt.r_min, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.r_max, 0.0f); 
}

TEST (PointTypeConstruction, PrincipalRadiiRSDScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PrincipalRadiiRSD pt(1.0f, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.r_min, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.r_max, 2.0f);
}

TEST (PointTypeConstruction, BoundaryDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Boundary pt;
  PCL_EXPECT_INT_EQ(pt.boundary_point, std::uint8_t{});
}

TEST (PointTypeConstruction, BoundaryOneScalarConstruction)
{
  PCL_CONSTEXPR const pcl::Boundary pt(1u);
  PCL_EXPECT_INT_EQ(pt.boundary_point, std::uint8_t{1u});
}

TEST (PointTypeConstruction, PrincipalCurvaturesDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PrincipalCurvatures pt;
  PCL_EXPECT_FLOAT_EQ(pt.principal_curvature_x, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.principal_curvature_y, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.principal_curvature_z, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.pc1, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.pc2, 0.0f);
}

TEST (PointTypeConstruction, PrincipalCurvaturesFiveScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PrincipalCurvatures pt(2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.principal_curvature_x, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.principal_curvature_y, 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.principal_curvature_z, 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.pc1, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.pc2, 6.0f);
}

TEST (PointTypeConstruction, ReferenceFrameDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::ReferenceFrame pt;
  PCL_EXPECT_FLOAT_EQ(pt.rf[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[3], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[4], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[5], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[6], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[7], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.rf[8], 0.0f);
}

TEST (PointTypeConstruction, ReferenceFrameArrayOfScalarsConstruction)
{
  PCL_CONSTEXPR const float values[9]{ 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f };
  PCL_CONSTEXPR const pcl::ReferenceFrame pt(values);
  PCL_EXPECT_FLOAT_EQ(pt.rf[0], values[0]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[1], values[1]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[2], values[2]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[3], values[3]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[4], values[4]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[5], values[5]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[6], values[6]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[7], values[7]);
  PCL_EXPECT_FLOAT_EQ(pt.rf[8], values[8]);
}


namespace pcl
{

// to be replaced with std:: when C++20 is available
// implementations taken from cppreference.com

template<class InputIt, class UnaryPredicate>
constexpr InputIt find_if_not(InputIt first, InputIt last, UnaryPredicate q)
{
    for (; first != last; ++first) {
        if (!q(*first)) {
            return first;
        }
    }
    return last;
}

template <class InputIt, class UnaryPredicate>
constexpr bool all_of(InputIt first, InputIt last, UnaryPredicate p)
{
    return pcl::find_if_not(first, last, p) == last;
}

// may be replaced with lambda when C++17 is available
constexpr bool is_equal_to_zero(float value)
{
  return value == 0.0f;
}

}

template <typename T> class PointTypesWithRawArrayMemberTest : public ::testing::Test { };
using PointTypesWithRawArrayMember
  = ::testing::Types<
    pcl::FPFHSignature33, 
    pcl::VFHSignature308, 
    pcl::GRSDSignature21, 
    pcl::ESFSignature640, 
    pcl::GASDSignature512, 
    pcl::GASDSignature984, 
    pcl::GASDSignature7992, 
    pcl::GFPFHSignature16>;
TYPED_TEST_SUITE (PointTypesWithRawArrayMemberTest, PointTypesWithRawArrayMember);

TYPED_TEST (PointTypesWithRawArrayMemberTest, ConstexprDefaultConstructionTests)
{
  PCL_CONSTEXPR const TypeParam pt;
  PCL_EXPECT_TRUE(pcl::all_of(std::cbegin(pt.histogram), std::cend(pt.histogram), &pcl::is_equal_to_zero));
}

TEST (PointTypeConstuction, BRISKSignature512DefaultConstruction)
{
  PCL_CONSTEXPR const BRISKSignature512 pt;
  PCL_EXPECT_TRUE(pcl::all_of(std::cbegin(pt.descriptor), std::cend(pt.descriptor), &pcl::is_equal_to_zero));
}

TEST (PointTypeConstruction, Narf36DefaultConstruction)
{
  PCL_CONSTEXPR const pcl::Narf36 pt;
  PCL_EXPECT_FLOAT_EQ(pt.x, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.y, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.z, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.roll, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.pitch, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.yaw, 0.0f);
}

TEST (PointTypeConstruction, Narf36ThreeScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::Narf36 pt{1.0f, 2.0f, 3.0f};
  PCL_EXPECT_FLOAT_EQ(pt.x, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.y, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.z, 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.roll, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.pitch, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.yaw, 0.0f);
}

TEST (PointTypeConstruction, Narf36SixScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::Narf36 pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  PCL_EXPECT_FLOAT_EQ(pt.x, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.y, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.z, 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.roll, 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.pitch, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.yaw, 6.0f);
}

TEST (PointTypeConstruction, BorderDescriptionDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::BorderDescription pt;
  PCL_EXPECT_INT_EQ(pt.x, 0);
  PCL_EXPECT_INT_EQ(pt.y, 0);
}


TEST (PointTypeConstruction, BorderDescriptionTwoScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::BorderDescription pt{1, 2};
  PCL_EXPECT_INT_EQ(pt.x, 1);
  PCL_EXPECT_INT_EQ(pt.y, 2);
}

TEST (PointTypeConstruction, IntensityGradientDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::IntensityGradient pt;
  PCL_EXPECT_FLOAT_EQ(pt.gradient_x, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.gradient_y, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.gradient_z, 0.0f);
}

TEST (PointTypeConstruction, IntensityGradientThreeScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::IntensityGradient pt{1.0f, 2.0f, 3.0f};
  PCL_EXPECT_FLOAT_EQ(pt.gradient_x, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.gradient_y, 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.gradient_z, 3.0f);
}

TEST (PointTypeConstruction, PointWithScaleDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointWithScale pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.scale, 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.angle, -1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.response, 0.0f);
  PCL_EXPECT_INT_EQ(pt.octave, 0);
}

TEST (PointTypeConstruction, PointWithScaleSevenScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointWithScale pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7};
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.scale, 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.angle, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.response, 6.0f);
  PCL_EXPECT_INT_EQ(pt.octave, 7);
}

TEST (PointTypeConstruction, PointSurfelDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointSurfel pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_INT_EQ(pt.r, 0u);
  PCL_EXPECT_INT_EQ(pt.g, 0u);
  PCL_EXPECT_INT_EQ(pt.b, 0u);
  PCL_EXPECT_INT_EQ(pt.a, 0u);
  PCL_EXPECT_FLOAT_EQ(pt.radius, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.confidence, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 0.0f);
}

TEST (PointTypeConstruction, PointSurfelTenScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointSurfel pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7u, 8u, 9u, 10u, 11.0f, 12.0f, 13.0f};
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[0], 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[1], 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[2], 6.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data_n[3], 0.0f);
  PCL_EXPECT_INT_EQ(pt.r, 7u);
  PCL_EXPECT_INT_EQ(pt.g, 8u);
  PCL_EXPECT_INT_EQ(pt.b, 9u);
  PCL_EXPECT_INT_EQ(pt.a, 10u);
  PCL_EXPECT_FLOAT_EQ(pt.radius, 11.0f);
  PCL_EXPECT_FLOAT_EQ(pt.confidence, 12.0f);
  PCL_EXPECT_FLOAT_EQ(pt.curvature, 13.0f);
}

TEST (PointTypeConstruction, PointDEMDefaultConstruction)
{
  PCL_CONSTEXPR const pcl::PointDEM pt;
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity_variance, 0.0f);
  PCL_EXPECT_FLOAT_EQ(pt.height_variance, 0.0f);
}

TEST (PointTypeConstruction, PointDEMSixScalarsConstruction)
{
  PCL_CONSTEXPR const pcl::PointDEM pt{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  PCL_EXPECT_FLOAT_EQ(pt.data[0], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[1], 2.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[2], 3.0f);
  PCL_EXPECT_FLOAT_EQ(pt.data[3], 1.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity, 4.0f);
  PCL_EXPECT_FLOAT_EQ(pt.intensity_variance, 5.0f);
  PCL_EXPECT_FLOAT_EQ(pt.height_variance, 6.0f);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
