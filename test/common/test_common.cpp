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

#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/centroid.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointXYZRGB)
{
  PointXYZRGB p;

  uint8_t r = 127, g = 64, b = 254;
  uint32_t rgb = (static_cast<uint32_t> (r) << 16 | 
                  static_cast<uint32_t> (g) << 8 | 
                  static_cast<uint32_t> (b));
  p.rgb = *reinterpret_cast<float*>(&rgb);

  rgb = *reinterpret_cast<int*>(&p.rgb);
  uint8_t rr = (rgb >> 16) & 0x0000ff;
  uint8_t gg = (rgb >> 8)  & 0x0000ff;
  uint8_t bb = (rgb)       & 0x0000ff;

  EXPECT_EQ (r, rr);
  EXPECT_EQ (g, gg);
  EXPECT_EQ (b, bb);
  EXPECT_EQ (rr, 127);
  EXPECT_EQ (gg, 64);
  EXPECT_EQ (bb, 254);

  p.r = 0; p.g = 127; p.b = 0;
  rgb = *reinterpret_cast<int*>(&p.rgb);
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

  uint8_t r = 127, g = 64, b = 254;
  uint32_t rgb = (static_cast<uint32_t> (r) << 16 | 
                  static_cast<uint32_t> (g) << 8 | 
                  static_cast<uint32_t> (b));
  p.rgb = *reinterpret_cast<float*>(&rgb);

  rgb = *reinterpret_cast<int*>(&p.rgb);
  uint8_t rr = (rgb >> 16) & 0x0000ff;
  uint8_t gg = (rgb >> 8)  & 0x0000ff;
  uint8_t bb = (rgb)       & 0x0000ff;

  EXPECT_EQ (r, rr);
  EXPECT_EQ (g, gg);
  EXPECT_EQ (b, bb);
  EXPECT_EQ (rr, 127);
  EXPECT_EQ (gg, 64);
  EXPECT_EQ (bb, 254);

  p.r = 0; p.g = 127; p.b = 0;
  rgb = *reinterpret_cast<int*>(&p.rgb);
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
  EXPECT_EQ (isFinite (p), false);
  Normal n;
  n.normal_x = std::numeric_limits<float>::quiet_NaN ();
  EXPECT_EQ (isFinite (n), false);
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

  EXPECT_NEAR (fabs (vec (0, 0)), 0.168841, 1e-4); EXPECT_NEAR (fabs (vec (0, 1)), 0.161623, 1e-4); EXPECT_NEAR (fabs (vec (0, 2)), 0.972302, 1e-4);
  EXPECT_NEAR (fabs (vec (1, 0)), 0.451632, 1e-4); EXPECT_NEAR (fabs (vec (1, 1)), 0.889498, 1e-4); EXPECT_NEAR (fabs (vec (1, 2)), 0.0694328, 1e-4);
  EXPECT_NEAR (fabs (vec (2, 0)), 0.876082, 1e-4); EXPECT_NEAR (fabs (vec (2, 1)), 0.4274,   1e-4); EXPECT_NEAR (fabs (vec (2, 2)), 0.223178, 1e-4);

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
TEST (PCL, PointCloud)
{
  PointCloud<PointXYZ> cloud;
  cloud.width = 640;
  cloud.height = 480;

  EXPECT_EQ (cloud.isOrganized (), true);

  cloud.height = 1;
  EXPECT_EQ (cloud.isOrganized (), false);

  cloud.width = 10;
  for (uint32_t i = 0; i < cloud.width*cloud.height; ++i)
  {
    float j = static_cast<float> (i);
    cloud.points.push_back (PointXYZ (3.0f * j + 0.0f, 3.0f * j + 1.0f, 3.0f * j + 2.0f));
  }

  Eigen::MatrixXf mat_xyz1 = cloud.getMatrixXfMap ();
  Eigen::MatrixXf mat_xyz = cloud.getMatrixXfMap (3, 4, 0);

  if (Eigen::MatrixXf::Flags & Eigen::RowMajorBit)
  {
    EXPECT_EQ (mat_xyz1.cols (), 4);
    EXPECT_EQ (mat_xyz1.rows (), cloud.width);
    EXPECT_EQ (mat_xyz1 (0, 0), 0);
    EXPECT_EQ (mat_xyz1 (cloud.width - 1, 2), 3 * cloud.width - 1);   // = 29

    EXPECT_EQ (mat_xyz.cols (), 3);
    EXPECT_EQ (mat_xyz.rows (), cloud.width);
    EXPECT_EQ (mat_xyz (0, 0), 0);
    EXPECT_EQ (mat_xyz (cloud.width - 1, 2), 3 * cloud.width - 1);    // = 29
  }
  else
  {
    EXPECT_EQ (mat_xyz1.cols (), cloud.width);
    EXPECT_EQ (mat_xyz1.rows (), 4);
    EXPECT_EQ (mat_xyz1 (0, 0), 0);
    EXPECT_EQ (mat_xyz1 (2, cloud.width - 1), 3 * cloud.width - 1);   // = 29

    EXPECT_EQ (mat_xyz.cols (), cloud.width);
    EXPECT_EQ (mat_xyz.rows (), 3);
    EXPECT_EQ (mat_xyz (0, 0), 0);
    EXPECT_EQ (mat_xyz (2, cloud.width - 1), 3 * cloud.width - 1);    // = 29
  }
  
#ifdef NDEBUG
  if (Eigen::MatrixXf::Flags & Eigen::RowMajorBit)
  {
    Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, 1);
    EXPECT_EQ (mat_yz.cols (), 2);
    EXPECT_EQ (mat_yz.rows (), cloud.width);
    EXPECT_EQ (mat_yz (0, 0), 1);
    EXPECT_EQ (mat_yz (cloud.width - 1, 1), 3 * cloud.width - 1);
    uint32_t j = 1;
    for (uint32_t i = 1; i < cloud.width*cloud.height; i+=4, j+=3)
    {
      Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, i);
      EXPECT_EQ (mat_yz.cols (), 2);
      EXPECT_EQ (mat_yz.rows (), cloud.width);
      EXPECT_EQ (mat_yz (0, 0), j);
    }
  }
  else
  {
    Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, 1);
    EXPECT_EQ (mat_yz.cols (), cloud.width);
    EXPECT_EQ (mat_yz.rows (), 2);
    EXPECT_EQ (mat_yz (0, 0), 1);
    EXPECT_EQ (mat_yz (1, cloud.width - 1), 3 * cloud.width - 1);
    uint32_t j = 1;
    for (uint32_t i = 1; i < cloud.width*cloud.height; i+=4, j+=3)
    {
      Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap (2, 4, i);
      EXPECT_EQ (mat_yz.cols (), cloud.width);
      EXPECT_EQ (mat_yz.rows (), 2);
      EXPECT_EQ (mat_yz (0, 0), j);
    }
  }
#endif
  cloud.clear ();
  EXPECT_EQ (cloud.width, 0);
  EXPECT_EQ (cloud.height, 0);

  cloud.width = 640;
  cloud.height = 480;

  cloud.insert (cloud.end (), PointXYZ (1, 1, 1));
  EXPECT_EQ (cloud.isOrganized (), false);
  EXPECT_EQ (cloud.width, 1);

  cloud.insert (cloud.end (), 5, PointXYZ (1, 1, 1));
  EXPECT_EQ (cloud.isOrganized (), false);
  EXPECT_EQ (cloud.width, 6);

  cloud.erase (cloud.end () - 1);
  EXPECT_EQ (cloud.isOrganized (), false);
  EXPECT_EQ (cloud.width, 5);

  cloud.erase (cloud.begin (), cloud.end ());
  EXPECT_EQ (cloud.isOrganized (), false);
  EXPECT_EQ (cloud.width, 0);
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
typedef ::testing::Types<BOOST_PP_SEQ_ENUM(PCL_XYZ_POINT_TYPES)> XYZPointTypes;
TYPED_TEST_CASE(XYZPointTypesTest, XYZPointTypes);
TYPED_TEST(XYZPointTypesTest, GetVectorXfMap)
{
  TypeParam pt;
  for (size_t i = 0; i < 3; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getVector3fMap () (i));
  for (size_t i = 0; i < 4; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getVector4fMap () (i));
}

TYPED_TEST(XYZPointTypesTest, GetArrayXfMap)
{
  TypeParam pt;
  for (size_t i = 0; i < 3; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getArray3fMap () (i));
  for (size_t i = 0; i < 4; ++i)
    EXPECT_EQ (&pt.data[i], &pt.getArray4fMap () (i));
}

template <typename T> class NormalPointTypesTest : public ::testing::Test { };
typedef ::testing::Types<BOOST_PP_SEQ_ENUM(PCL_NORMAL_POINT_TYPES)> NormalPointTypes;
TYPED_TEST_CASE(NormalPointTypesTest, NormalPointTypes);
TYPED_TEST(NormalPointTypesTest, GetNormalVectorXfMap)
{
  TypeParam pt;
  for (size_t i = 0; i < 3; ++i)
    EXPECT_EQ (&pt.data_n[i], &pt.getNormalVector3fMap () (i));
  for (size_t i = 0; i < 4; ++i)
    EXPECT_EQ (&pt.data_n[i], &pt.getNormalVector4fMap () (i));
}

template <typename T> class RGBPointTypesTest : public ::testing::Test { };
typedef ::testing::Types<BOOST_PP_SEQ_ENUM(PCL_RGB_POINT_TYPES)> RGBPointTypes;
TYPED_TEST_CASE(RGBPointTypesTest, RGBPointTypes);
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
  EXPECT_EQ ((pcl::lineWithLineIntersection (zline, yline, pt)), true);
  EXPECT_NEAR (pt[0], 0.574544, 1e-3);
  EXPECT_NEAR (pt[1], 0.175526, 1e-3);
  EXPECT_NEAR (pt[2], 1.27636,  1e-3);
  EXPECT_EQ (pt[3], 0);

  zline << 0.545203f, -0.514419f, 1.31967f, 0.0243372f, 0.597946f, -0.0413579f;
  yline << 0.492706f,  0.164196f, 1.23192f, 0.598704f,  0.0442014f, 0.411328f;
  EXPECT_EQ ((pcl::lineWithLineIntersection (zline, yline, pt)), false);
  //intersection: [ 3.06416e+08    15.2237     3.06416e+08       4.04468e-34 ]
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, compute3DCentroidFloat)
{
  pcl::PointIndices pindices;
  std::vector<int> indices;
  PointXYZ point;
  PointCloud<PointXYZ> cloud;
  Eigen::Vector4f centroid;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)
  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;

  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);

  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  indices.push_back (8); // add the NaN
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);

  pindices.indices = indices;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, compute3DCentroidDouble)
{
  pcl::PointIndices pindices;
  std::vector<int> indices;
  PointXYZ point;
  PointCloud<PointXYZ> cloud;
  Eigen::Vector4d centroid;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)
  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;

  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);

  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  indices.push_back (8); // add the NaN
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
  
  pindices.indices = indices;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, compute3DCentroidCloudIterator)
{
  pcl::PointIndices pindices;
  std::vector<int> indices;
  PointXYZ point;
  PointCloud<PointXYZ> cloud;
  Eigen::Vector4f centroid_f;

  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;

  ConstCloudIterator<PointXYZ> it (cloud, indices);
  
  EXPECT_EQ (compute3DCentroid (it, centroid_f), 4);

  EXPECT_EQ (centroid_f[0], 0.0f);
  EXPECT_EQ (centroid_f[1], 1.0f);
  EXPECT_EQ (centroid_f[2], 0.0f);
  
  Eigen::Vector4d centroid_d;
  it.reset ();
  EXPECT_EQ (compute3DCentroid (it, centroid_d), 4);

  EXPECT_EQ (centroid_d[0], 0.0);
  EXPECT_EQ (centroid_d[1], 1.0);
  EXPECT_EQ (centroid_d[2], 0.0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeCovarianceMatrix)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;

  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 8);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 8);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 8);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 4);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 4);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 0;

  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 8);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 8);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 8);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 4);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeCovarianceMatrixNormalized)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;

  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, indices, centroid, covariance_matrix), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 8);

  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, indices, centroid, covariance_matrix), 4);

  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 0;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, indices, centroid, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeDemeanedCovariance)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Matrix3f covariance_matrix;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, covariance_matrix), 0);

  cloud.clear ();
  indices.clear ();

  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeMeanAndCovariance)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f centroid;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix, centroid), 0);

  cloud.clear ();
  indices.clear ();

  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 8);

  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;

  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix, centroid), 4);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 1);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;

  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 8);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;

  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix, centroid), 4);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 1);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CopyIfFieldExists)
{
  PointXYZRGBNormal p;

  p.x = 1.0; p.y = 2;  p.z = 3.0;
  p.r = 127; p.g = 64; p.b = 254;
  p.normal_x = 1.0; p.normal_y = 0.0; p.normal_z = 0.0;

  typedef pcl::traits::fieldList<PointXYZRGBNormal>::type FieldList;
  bool is_x = false, is_y = false, is_z = false, is_rgb = false, 
       is_normal_x = false, is_normal_y = false, is_normal_z = false;

  float x_val, y_val, z_val, normal_x_val, normal_y_val, normal_z_val, rgb_val;
  x_val = y_val = z_val = std::numeric_limits<float>::quiet_NaN ();
  normal_x_val = normal_y_val = normal_z_val = std::numeric_limits<float>::quiet_NaN ();
  rgb_val = std::numeric_limits<float>::quiet_NaN ();

  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "x", is_x, x_val));
  EXPECT_EQ (is_x, true);
  EXPECT_EQ (x_val, 1.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "y", is_y, y_val));
  EXPECT_EQ (is_y, true);
  EXPECT_EQ (y_val, 2.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "z", is_z, z_val));
  EXPECT_EQ (is_z, true);
  EXPECT_EQ (z_val, 3.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "rgb", is_rgb, rgb_val));
  EXPECT_EQ (is_rgb, true);
  int rgb = *reinterpret_cast<int*>(&rgb_val);
  EXPECT_EQ (rgb, 8339710);      // alpha is 0
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "normal_x", is_normal_x, normal_x_val));
  EXPECT_EQ (is_normal_x, true);
  EXPECT_EQ (normal_x_val, 1.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "normal_y", is_normal_y, normal_y_val));
  EXPECT_EQ (is_normal_y, true);
  EXPECT_EQ (normal_y_val, 0.0);
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "normal_z", is_normal_z, normal_z_val));
  EXPECT_EQ (is_normal_z, true);
  EXPECT_EQ (normal_z_val, 0.0);
  
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "x", x_val));
  EXPECT_EQ (x_val, 1.0);

  float xx_val = -1.0;
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "xx", xx_val));
  EXPECT_EQ (xx_val, -1.0);
  bool is_xx = true;
  pcl::for_each_type<FieldList> (CopyIfFieldExists<PointXYZRGBNormal, float> (p, "xx", is_xx, xx_val));
  EXPECT_EQ (is_xx, false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SetIfFieldExists)
{
  PointXYZRGBNormal p;

  p.x = p.y = p.z = 0.0;
  p.r = p.g = p.b = 0;
  p.normal_x = p.normal_y = p.normal_z = 0.0;

  typedef pcl::traits::fieldList<PointXYZRGBNormal>::type FieldList;
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
  // has_color
  EXPECT_TRUE ((pcl::traits::has_color<pcl::PointXYZRGB>::value));
  EXPECT_TRUE ((pcl::traits::has_color<pcl::PointXYZRGBA>::value));
  EXPECT_FALSE ((pcl::traits::has_color<pcl::PointXYZ>::value));
  // has_label
  EXPECT_TRUE ((pcl::traits::has_label<pcl::PointXYZL>::value));
  EXPECT_FALSE ((pcl::traits::has_label<pcl::Normal>::value));
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
