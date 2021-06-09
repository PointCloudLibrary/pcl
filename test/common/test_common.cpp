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

#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/point_tests.h> // for isFinite

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointXYZRGB)
{
  PointXYZRGB p;

  std::uint8_t r = 127, g = 64, b = 254;
  p.r = r;
  p.g = g;
  p.b = b;

  std::uint8_t rr = (p.rgba >> 16) & 0x0000ff;
  std::uint8_t gg = (p.rgba >> 8)  & 0x0000ff;
  std::uint8_t bb = (p.rgba)       & 0x0000ff;

  EXPECT_EQ (r, rr);
  EXPECT_EQ (g, gg);
  EXPECT_EQ (b, bb);
  EXPECT_EQ (rr, 127);
  EXPECT_EQ (gg, 64);
  EXPECT_EQ (bb, 254);

  p.r = 0; p.g = 127; p.b = 0;
  std::uint32_t rgb = p.rgba;
  rr = (rgb >> 16) & 0x0000ff;
  gg = (rgb >> 8)  & 0x0000ff;
  bb = (rgb)       & 0x0000ff;

  EXPECT_EQ (rr, 0);
  EXPECT_EQ (gg, 127);
  EXPECT_EQ (bb, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointXYZRGBNormal)
{
  PointXYZRGBNormal p;

  std::uint8_t r = 127, g = 64, b = 254;
  std::uint32_t rgb = (static_cast<std::uint32_t> (r) << 16 |
                  static_cast<std::uint32_t> (g) << 8 |
                  static_cast<std::uint32_t> (b));
  p.rgba = rgb;

  std::uint8_t rr = (p.rgba >> 16) & 0x0000ff;
  std::uint8_t gg = (p.rgba >> 8)  & 0x0000ff;
  std::uint8_t bb = (p.rgba)       & 0x0000ff;

  EXPECT_EQ (r, rr);
  EXPECT_EQ (g, gg);
  EXPECT_EQ (b, bb);
  EXPECT_EQ (rr, 127);
  EXPECT_EQ (gg, 64);
  EXPECT_EQ (bb, 254);

  p.r = 0; p.g = 127; p.b = 0;
  rgb = p.rgba;
  rr = (rgb >> 16) & 0x0000ff;
  gg = (rgb >> 8)  & 0x0000ff;
  bb = (rgb)       & 0x0000ff;

  EXPECT_EQ (rr, 0);
  EXPECT_EQ (gg, 127);
  EXPECT_EQ (bb, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, isFinite)
{
  PointXYZ p;
  p.x = std::numeric_limits<float>::quiet_NaN ();
  EXPECT_FALSE (isFinite (p));
  Normal n;
  n.normal_x = std::numeric_limits<float>::quiet_NaN ();
  EXPECT_FALSE (isFinite (n));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Common)
{
  PointXYZ p1, p2, p3;
  p1.x = 1; p1.y = p1.z = 0;
  p2.y = 1; p2.x = p2.z = 0;
  p3.z = 1; p3.x = p3.y = 0;
  double radius = getCircumcircleRadius (p1, p2, p3);
  EXPECT_NEAR (radius, 0.816497, 1e-4);

  Eigen::Vector4f pt (1,0,0,0), line_pt (0,0,0,0), line_dir (1,1,0,0);
  double point2line_disance = sqrt (sqrPointToLineDistance (pt, line_pt, line_dir));
  EXPECT_NEAR (point2line_disance, sqrt(2.0)/2, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Eigen)
{
  Eigen::Matrix3f mat, vec;
  mat << 0.000536227f, -1.56178e-05f, -9.47391e-05f, -1.56178e-05f, 0.000297322f, -0.000148785f, -9.47391e-05f, -0.000148785f, 9.7827e-05f;
  Eigen::Vector3f val;

  eigen33 (mat, vec, val);

  EXPECT_NEAR (std::abs (vec (0, 0)), 0.168841, 1e-4); EXPECT_NEAR (std::abs (vec (0, 1)), 0.161623, 1e-4); EXPECT_NEAR (std::abs (vec (0, 2)), 0.972302, 1e-4);
  EXPECT_NEAR (std::abs (vec (1, 0)), 0.451632, 1e-4); EXPECT_NEAR (std::abs (vec (1, 1)), 0.889498, 1e-4); EXPECT_NEAR (std::abs (vec (1, 2)), 0.0694328, 1e-4);
  EXPECT_NEAR (std::abs (vec (2, 0)), 0.876082, 1e-4); EXPECT_NEAR (std::abs (vec (2, 1)), 0.4274,   1e-4); EXPECT_NEAR (std::abs (vec (2, 2)), 0.223178, 1e-4);

  EXPECT_NEAR (val (0), 2.86806e-06, 1e-4); EXPECT_NEAR (val (1), 0.00037165, 1e-4); EXPECT_NEAR (val (2), 0.000556858, 1e-4);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig (mat);

  EXPECT_NEAR (eig.eigenvectors () (0, 0), -0.168841, 1e-4); EXPECT_NEAR (eig.eigenvectors () (0, 1),  0.161623, 1e-4); EXPECT_NEAR (eig.eigenvectors () (0, 2),  0.972302, 1e-4);
  EXPECT_NEAR (eig.eigenvectors () (1, 0), -0.451632, 1e-4); EXPECT_NEAR (eig.eigenvectors () (1, 1), -0.889498, 1e-4); EXPECT_NEAR (eig.eigenvectors () (1, 2),  0.0694328, 1e-4);
  EXPECT_NEAR (eig.eigenvectors () (2, 0), -0.876083, 1e-4); EXPECT_NEAR (eig.eigenvectors () (2, 1),  0.4274,   1e-4); EXPECT_NEAR (eig.eigenvectors () (2, 2), -0.223178, 1e-4);

  EXPECT_NEAR (eig.eigenvalues () (0), 2.86806e-06, 1e-4); EXPECT_NEAR (eig.eigenvalues () (1), 0.00037165, 1e-4); EXPECT_NEAR (eig.eigenvalues () (2), 0.000556858, 1e-4);

  Eigen::Vector3f eivals = mat.selfadjointView<Eigen::Lower>().eigenvalues ();

  EXPECT_NEAR (eivals (0), 2.86806e-06, 1e-4); EXPECT_NEAR (eivals (1), 0.00037165, 1e-4); EXPECT_NEAR (eivals (2), 0.000556858, 1e-4);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointTypes)
{
  EXPECT_EQ (sizeof (PointXYZ), 16);
  EXPECT_EQ (__alignof (PointXYZ), 16);
  EXPECT_EQ (sizeof (PointXYZI), 32);
  EXPECT_EQ (__alignof (PointXYZI), 16);
  EXPECT_EQ (sizeof (PointXYZRGB), 32);
  EXPECT_EQ (__alignof (PointXYZRGB), 16);
  EXPECT_EQ (sizeof (PointXYZRGBA), 32);
  EXPECT_EQ (__alignof (PointXYZRGBA), 16);
  EXPECT_EQ (sizeof (Normal), 32);
  EXPECT_EQ (__alignof (Normal), 16);
  EXPECT_EQ (sizeof (PointNormal), 48);
  EXPECT_EQ (__alignof (PointNormal), 16);
  EXPECT_EQ (sizeof (PointXYZRGBNormal), 48);
  EXPECT_EQ (__alignof (PointXYZRGBNormal), 16);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T> class XYZPointTypesTest : public ::testing::Test { };
using XYZPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM(PCL_XYZ_POINT_TYPES)>;
TYPED_TEST_SUITE(XYZPointTypesTest, XYZPointTypes);
TYPED_TEST(XYZPointTypesTest, GetVectorXfMap)
{
  TypeParam pt;
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getVector3fMap () (i));
  for (std::size_t i = 0; i < 4; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getVector4fMap () (i));
}

TYPED_TEST(XYZPointTypesTest, GetArrayXfMap)
{
  TypeParam pt;
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getArray3fMap () (i));
  for (std::size_t i = 0; i < 4; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getArray4fMap () (i));
}

template <typename T> class NormalPointTypesTest : public ::testing::Test { };
using NormalPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM(PCL_NORMAL_POINT_TYPES)>;
TYPED_TEST_SUITE(NormalPointTypesTest, NormalPointTypes);
TYPED_TEST(NormalPointTypesTest, GetNormalVectorXfMap)
{
  TypeParam pt;
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_EQ (&pt.data_n[i], &pt.getNormalVector3fMap () (i));
  for (std::size_t i = 0; i < 4; ++i)
    EXPECT_EQ (&pt.data_n[i], &pt.getNormalVector4fMap () (i));
}

template <typename T> class RGBPointTypesTest : public ::testing::Test { };
using RGBPointTypes = ::testing::Types<BOOST_PP_SEQ_ENUM(PCL_RGB_POINT_TYPES)>;
TYPED_TEST_SUITE(RGBPointTypesTest, RGBPointTypes);
TYPED_TEST(RGBPointTypesTest, GetRGBVectorXi)
{
  TypeParam pt; pt.r = 1; pt.g = 2; pt.b = 3; pt.a = 4;
  EXPECT_EQ (pt.r, pt.getRGBVector3i () (0));
  EXPECT_EQ (pt.g, pt.getRGBVector3i () (1));
  EXPECT_EQ (pt.b, pt.getRGBVector3i () (2));
  EXPECT_EQ (pt.r, pt.getRGBVector4i () (0));
  EXPECT_EQ (pt.g, pt.getRGBVector4i () (1));
  EXPECT_EQ (pt.b, pt.getRGBVector4i () (2));
  EXPECT_EQ (pt.a, pt.getRGBVector4i () (3));
  EXPECT_EQ (pt.r, pt.getRGBAVector4i () (0));
  EXPECT_EQ (pt.g, pt.getRGBAVector4i () (1));
  EXPECT_EQ (pt.b, pt.getRGBAVector4i () (2));
  EXPECT_EQ (pt.a, pt.getRGBAVector4i () (3));
}

TYPED_TEST(RGBPointTypesTest, GetBGRVectorXcMap)
{
  TypeParam pt;
  EXPECT_EQ (&pt.b, &pt.getBGRVector3cMap () (0));
  EXPECT_EQ (&pt.g, &pt.getBGRVector3cMap () (1));
  EXPECT_EQ (&pt.r, &pt.getBGRVector3cMap () (2));
  EXPECT_EQ (&pt.b, &pt.getBGRAVector4cMap () (0));
  EXPECT_EQ (&pt.g, &pt.getBGRAVector4cMap () (1));
  EXPECT_EQ (&pt.r, &pt.getBGRAVector4cMap () (2));
  EXPECT_EQ (&pt.a, &pt.getBGRAVector4cMap () (3));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Intersections)
{
  Eigen::VectorXf zline (6), yline (6);
  zline[0] = 0.543892f; zline[1] = -0.515623f; zline[2] = 1.321f;   zline[3] = 0.0266191f; zline[4] = 0.600215f;  zline[5] = -0.0387667f;
  yline[0] = 0.493479f; yline[1] = 0.169246f;  yline[2] = 1.22677f; yline[3] = 0.5992f;    yline[4] = 0.0505085f; yline[5] = 0.405749f;

  Eigen::Vector4f pt;
  EXPECT_TRUE (pcl::lineWithLineIntersection (zline, yline, pt));
  EXPECT_NEAR (pt[0], 0.574544, 1e-3);
  EXPECT_NEAR (pt[1], 0.175526, 1e-3);
  EXPECT_NEAR (pt[2], 1.27636,  1e-3);
  EXPECT_EQ (pt[3], 0);

  zline << 0.545203f, -0.514419f, 1.31967f, 0.0243372f, 0.597946f, -0.0413579f;
  yline << 0.492706f,  0.164196f, 1.23192f, 0.598704f,  0.0442014f, 0.411328f;
  EXPECT_FALSE (pcl::lineWithLineIntersection (zline, yline, pt));
  //intersection: [ 3.06416e+08    15.2237     3.06416e+08       4.04468e-34 ]
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CopyIfFieldExists)
{
  PointXYZRGBNormal p;

  p.x = 1.0; p.y = 2;  p.z = 3.0;
  p.r = 127; p.g = 64; p.b = 254;
  p.normal_x = 1.0; p.normal_y = 0.0; p.normal_z = 0.0;

  using FieldList = pcl::traits::fieldList<PointXYZRGBNormal>::type;
  bool is_x = false, is_y = false, is_z = false, is_rgb = false,
       is_normal_x = false, is_normal_y = false, is_normal_z = false;

  float x_val, y_val, z_val, normal_x_val, normal_y_val, normal_z_val, rgb_val;
  x_val = y_val = z_val = std::numeric_limits<float>::quiet_NaN ();
  normal_x_val = normal_y_val = normal_z_val = std::numeric_limits<float>::quiet_NaN ();
  rgb_val = std::numeric_limits<float>::quiet_NaN ();

  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "x", is_x, x_val));
  EXPECT_TRUE (is_x);
  EXPECT_EQ (x_val, 1.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "y", is_y, y_val));
  EXPECT_TRUE (is_y);
  EXPECT_EQ (y_val, 2.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "z", is_z, z_val));
  EXPECT_TRUE (is_z);
  EXPECT_EQ (z_val, 3.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "rgb", is_rgb, rgb_val));
  EXPECT_TRUE (is_rgb);
  std::uint32_t rgb;
  std::memcpy (&rgb, &rgb_val, sizeof(rgb_val));
  EXPECT_EQ (rgb, 0xff7f40fe);      // alpha is 255
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "normal_x", is_normal_x, normal_x_val));
  EXPECT_TRUE (is_normal_x);
  EXPECT_EQ (normal_x_val, 1.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "normal_y", is_normal_y, normal_y_val));
  EXPECT_TRUE (is_normal_y);
  EXPECT_EQ (normal_y_val, 0.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "normal_z", is_normal_z, normal_z_val));
  EXPECT_TRUE (is_normal_z);
  EXPECT_EQ (normal_z_val, 0.0);

  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "x", x_val));
  EXPECT_EQ (x_val, 1.0);

  float xx_val = -1.0;
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "xx", xx_val));
  EXPECT_EQ (xx_val, -1.0);
  bool is_xx = true;
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "xx", is_xx, xx_val));
  EXPECT_FALSE (is_xx);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SetIfFieldExists)
{
  PointXYZRGBNormal p;

  p.x = p.y = p.z = 0.0;
  p.r = p.g = p.b = 0;
  p.normal_x = p.normal_y = p.normal_z = 0.0;

  using FieldList = pcl::traits::fieldList<PointXYZRGBNormal>::type;
  pcl::for_each_type<FieldList> (SetIfFieldExists<PointXYZRGBNormal, float> (p, "x", 1.0));
  EXPECT_EQ (p.x, 1.0);
  pcl::for_each_type<FieldList> (SetIfFieldExists<PointXYZRGBNormal, float> (p, "y", 2.0));
  EXPECT_EQ (p.y, 2.0);
  pcl::for_each_type<FieldList> (SetIfFieldExists<PointXYZRGBNormal, float> (p, "z", 3.0));
  EXPECT_EQ (p.z, 3.0);
  pcl::for_each_type<FieldList> (SetIfFieldExists<PointXYZRGBNormal, float> (p, "normal_x", 1.0));
  EXPECT_EQ (p.normal_x, 1.0);
  pcl::for_each_type<FieldList> (SetIfFieldExists<PointXYZRGBNormal, float> (p, "normal_y", 0.0));
  EXPECT_EQ (p.normal_y, 0.0);
  pcl::for_each_type<FieldList> (SetIfFieldExists<PointXYZRGBNormal, float> (p, "normal_z", 0.0));
  EXPECT_EQ (p.normal_z, 0.0);

//  pcl::PointXY p1;
//  pcl::for_each_type<pcl::traits::fieldList<pcl::PointXY>::type> (pcl::SetIfFieldExists<pcl::PointXY, float> (p1, "intensity", 3.0));
//
//  pcl::PFHSignature125 p2;
//  pcl::for_each_type<pcl::traits::fieldList<pcl::PFHSignature125>::type> (pcl::SetIfFieldExists<pcl::PFHSignature125, float*> (p2, "intensity", 3.0));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IsSamePointType)
{
  bool status = isSamePointType<PointXYZ, PointXYZ> ();
  EXPECT_TRUE (status);
  status = isSamePointType<PointXYZ, PointXY> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointXY, PointXYZ> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointNormal, PointNormal> ();
  EXPECT_TRUE (status);
  status = isSamePointType<PointNormal, PointXYZRGBNormal> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointXYZRGB, PointXYZRGB> ();
  EXPECT_TRUE (status);

  // Even though it's the "same" type, rgb != rgba
  status = isSamePointType<PointXYZRGB, PointXYZRGBA> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointXYZRGBA, PointXYZRGB> ();
  EXPECT_FALSE (status);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, HasField)
{
  // has_field
  EXPECT_TRUE ((pcl::traits::has_field<pcl::Normal, pcl::fields::curvature>::value));
  EXPECT_FALSE ((pcl::traits::has_field<pcl::PointXYZ, pcl::fields::curvature>::value));
  // has_all_fields
  EXPECT_TRUE ((pcl::traits::has_all_fields<pcl::PointXYZRGB, boost::mpl::vector<pcl::fields::x, pcl::fields::rgb> >::value));
  EXPECT_FALSE ((pcl::traits::has_all_fields<pcl::PointXYZ, boost::mpl::vector<pcl::fields::x, pcl::fields::rgb> >::value));
  // has_any_field
  EXPECT_TRUE ((pcl::traits::has_any_field<pcl::PointXYZ, boost::mpl::vector<pcl::fields::x, pcl::fields::normal_x> >::value));
  EXPECT_TRUE ((pcl::traits::has_any_field<pcl::Normal, boost::mpl::vector<pcl::fields::x, pcl::fields::normal_x> >::value));
  EXPECT_FALSE ((pcl::traits::has_any_field<pcl::RGB, boost::mpl::vector<pcl::fields::x, pcl::fields::normal_x> >::value));
  // has_xyz
  EXPECT_TRUE ((pcl::traits::has_xyz<pcl::PointXYZ>::value));
  EXPECT_FALSE ((pcl::traits::has_xyz<pcl::Normal>::value));
  // has_normal
  EXPECT_TRUE ((pcl::traits::has_normal<pcl::PointNormal>::value));
  EXPECT_FALSE ((pcl::traits::has_normal<pcl::PointXYZ>::value));
  // has_curvature
  EXPECT_TRUE ((pcl::traits::has_curvature<pcl::PointNormal>::value));
  EXPECT_FALSE ((pcl::traits::has_curvature<pcl::RGB>::value));
  // has_intensity
  EXPECT_TRUE ((pcl::traits::has_intensity<pcl::PointXYZI>::value));
  EXPECT_FALSE ((pcl::traits::has_intensity<pcl::PointXYZ>::value));
  // has_color
  EXPECT_TRUE ((pcl::traits::has_color<pcl::PointXYZRGB>::value));
  EXPECT_TRUE ((pcl::traits::has_color<pcl::PointXYZRGBA>::value));
  EXPECT_FALSE ((pcl::traits::has_color<pcl::PointXYZ>::value));
  // has_label
  EXPECT_TRUE ((pcl::traits::has_label<pcl::PointXYZL>::value));
  EXPECT_FALSE ((pcl::traits::has_label<pcl::Normal>::value));
}

TEST (PCL, GetMinMax3D)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.emplace_back ( 0.0f,      0.0f,  0.0f);
  cloud.emplace_back (10.0f, -10000.0f,  1.0f);
  cloud.emplace_back ( 5.0f,      5.0f,  0.0f);
  cloud.emplace_back (-5.0f,      0.0f, -0.5f);

  pcl::PointXYZ min_pt, max_pt;
  Eigen::Vector4f min_vec, max_vec;

  pcl::getMinMax3D (cloud, min_pt, max_pt);
  EXPECT_EQ (min_pt.x, -5.0f);
  EXPECT_EQ (min_pt.y, -10000.0f);
  EXPECT_EQ (min_pt.z, -0.5f);
  EXPECT_EQ (max_pt.x, 10.0f);
  EXPECT_EQ (max_pt.y, 5.0f);
  EXPECT_EQ (max_pt.z, 1.0f);

  pcl::getMinMax3D (cloud, min_vec, max_vec);
  EXPECT_EQ (min_vec.x (), -5.0f);
  EXPECT_EQ (min_vec.y (), -10000.0f);
  EXPECT_EQ (min_vec.z (), -0.5f);
  EXPECT_EQ (max_vec.x (), 10.0f);
  EXPECT_EQ (max_vec.y (), 5.0f);
  EXPECT_EQ (max_vec.z (), 1.0f);

  pcl::PointIndices pindices;
  pindices.indices.push_back (0);
  pindices.indices.push_back (2);
  pcl::getMinMax3D (cloud, pindices, min_vec, max_vec);
  EXPECT_EQ (min_vec.x (), 0.0f);
  EXPECT_EQ (min_vec.y (), 0.0f);
  EXPECT_EQ (min_vec.z (), 0.0f);
  EXPECT_EQ (max_vec.x (), 5.0f);
  EXPECT_EQ (max_vec.y (), 5.0f);
  EXPECT_EQ (max_vec.z (), 0.0f);

  pcl::Indices indices;
  indices.push_back (1);
  indices.push_back (3);
  pcl::getMinMax3D (cloud, indices, min_vec, max_vec);
  EXPECT_EQ (min_vec.x (), -5.0f);
  EXPECT_EQ (min_vec.y (), -10000.0f);
  EXPECT_EQ (min_vec.z (), -0.5f);
  EXPECT_EQ (max_vec.x (), 10.0f);
  EXPECT_EQ (max_vec.y (), 0.0f);
  EXPECT_EQ (max_vec.z (), 1.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GetMaxDistance)
{
  PointCloud<PointXYZ> cloud;
  Eigen::Vector4f max_pt, max_exp_pt;
  const Eigen::Vector4f pivot_pt (Eigen::Vector4f::Zero ());

  // populate cloud
  cloud.resize (3);
  cloud[0].data[0] = 4.f; cloud[0].data[1] = 3.f;
  cloud[0].data[2] = 0.f; cloud[0].data[3] = 0.f;
  cloud[1].data[0] = 0.f; cloud[1].data[1] = 0.f;
  cloud[1].data[2] = 0.f; cloud[1].data[3] = 1000.f;  //This term should not influence max dist
  cloud[2].data[0] = -1.5f; cloud[2].data[1] = 1.5f;
  cloud[2].data[2] = -.5f; cloud[2].data[3] = 0.f;

  // No indices specified
  max_exp_pt = cloud[0].getVector4fMap ();
  getMaxDistance (cloud, pivot_pt, max_pt);
  test::EXPECT_EQ_VECTORS (max_exp_pt, max_pt);

  // Specifying indices
  Indices idx (2);
  idx[0] = 1; idx[1] = 2;
  max_exp_pt = cloud[2].getVector4fMap ();
  getMaxDistance (cloud, idx, pivot_pt, max_pt);
  test::EXPECT_EQ_VECTORS (max_exp_pt, max_pt);
}

TEST (PCL, computeMedian)
{
  std::vector<float> vector1{4.0f, 2.0f, 1.0f, 5.0f, 3.0f, 6.0f};
  const auto median1 = computeMedian (vector1.begin (), vector1.end ());
  EXPECT_EQ(median1, 3.5f);

  std::vector<double> vector2{1.0, 25.0, 9.0, 4.0, 16.0};
  const auto median2 = computeMedian (vector2.begin (), vector2.end (), [](const double& x){ return std::sqrt(x); });
  EXPECT_EQ(median2, 3.0);

  std::vector<double> vector3{1.0, 2.0, 6.0, 5.0, 4.0, 3.0};
  const auto median3 = computeMedian (vector3.begin (), vector3.end (), [](const double& x){ return x+1.0; });
  EXPECT_EQ(median3, 4.5);

  std::vector<int> vector4{-1, 1, 2, 9, 15, 16};
  const auto median4 = computeMedian (vector4.begin (), vector4.end ());
  EXPECT_EQ(median4, 5);

  std::vector<int> vector5{-1, 1, 2, 9, 15, 16};
  const auto median5 = computeMedian (vector5.begin (), vector5.end (), [](const int& x){ return static_cast<double>(x); });
  EXPECT_EQ(median5, 5.5);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
