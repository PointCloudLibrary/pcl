/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "pcl/common/centroid.h"

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointXYZRGB)
{
  PointXYZRGB p;

  uint8_t r = 127, g = 64, b = 254;
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
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
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
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
  mat << 0.000536227, -1.56178e-05, -9.47391e-05, -1.56178e-05, 0.000297322, -0.000148785, -9.47391e-05, -0.000148785, 9.7827e-05;
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
    cloud.points.push_back (PointXYZ (3 * i + 0, 3 * i + 1, 3 * i + 2));

  Eigen::MatrixXf mat_xyz1 = cloud.getMatrixXfMap ();
  Eigen::MatrixXf mat_xyz = cloud.getMatrixXfMap(3,4,0);

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
TEST (PCL, Intersections)
{
  Eigen::VectorXf zline (6), yline (6);
  zline[0] = 0.543892; zline[1] = -0.515623; zline[2] = 1.321;   zline[3] = 0.0266191; zline[4] = 0.600215;  zline[5] = -0.0387667;
  yline[0] = 0.493479; yline[1] = 0.169246;  yline[2] = 1.22677; yline[3] = 0.5992;    yline[4] = 0.0505085; yline[5] = 0.405749;

  Eigen::Vector4f pt;
  EXPECT_EQ ((pcl::lineWithLineIntersection (zline, yline, pt)), true);
  EXPECT_NEAR (pt[0], 0.574544, 1e-3);
  EXPECT_NEAR (pt[1], 0.175526, 1e-3);
  EXPECT_NEAR (pt[2], 1.27636,  1e-3);
  EXPECT_EQ (pt[3], 0);

  zline << 0.545203, -0.514419, 1.31967, 0.0243372, 0.597946, -0.0413579;
  yline << 0.492706,  0.164196, 1.23192, 0.598704,  0.0442014, 0.411328;
  EXPECT_EQ ((pcl::lineWithLineIntersection (zline, yline, pt)), false);
  //intersection: [ 3.06416e+08    15.2237     3.06416e+08       4.04468e-34 ]
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, compute3DCentroid)
{
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
  EXPECT_EQ (rgb, 8339710);
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

//* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
