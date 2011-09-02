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
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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
    cloud.points.push_back (PointXYZ (3*i+0,3*i+1,3*i+2));
    
  Eigen::MatrixXf mat_xyz1 = cloud.getMatrixXfMap();
  EXPECT_EQ (mat_xyz1.rows (), 4);
  EXPECT_EQ (mat_xyz1.cols (), cloud.width);
  EXPECT_EQ (mat_xyz1 (0,0), 0);
  EXPECT_EQ (mat_xyz1 (2,cloud.width-1), 3*cloud.width-1);

  Eigen::MatrixXf mat_xyz = cloud.getMatrixXfMap(3,4,0);
  EXPECT_EQ (mat_xyz.rows (), 3);
  EXPECT_EQ (mat_xyz.cols (), cloud.width);
  EXPECT_EQ (mat_xyz (0,0), 0);
  EXPECT_EQ (mat_xyz (2,cloud.width-1), 3*cloud.width-1);

#ifdef NDEBUG
  Eigen::MatrixXf mat_yz = cloud.getMatrixXfMap(2,4,1);
  EXPECT_EQ (mat_yz.rows (), 2);
  EXPECT_EQ (mat_yz.cols (), cloud.width);
  EXPECT_EQ (mat_yz (0,0), 1);
  EXPECT_EQ (mat_yz (1,cloud.width-1), 3*cloud.width-1);
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
 
/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
