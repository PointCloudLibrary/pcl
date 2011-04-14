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
 * $Id: test_common.cpp 36632 2010-05-12 19:17:08Z rusu $
 *
 */

#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>

using pcl::PointXYZ;

TEST (PCL, Common)
{
  PointXYZ p1, p2, p3;
  p1.x = 1; p1.y = p1.z = 0;
  p2.y = 1; p2.x = p2.z = 0;
  p3.z = 1; p3.x = p3.y = 0;
  double radius = getCircumcircleRadius (p1, p2, p3);
  EXPECT_NEAR (radius, 0.816497, 1e-4);

  Eigen::Vector4f pt (1,0,0,0), line_pt (0,0,0,0), line_dir (1,1,0,0);
  double point2line_disance = sqrt (pcl::sqrPointToLineDistance (pt, line_pt, line_dir));
  EXPECT_NEAR (point2line_disance, sqrt(2.0)/2, 1e-4);
}

TEST (PCL, Eigen)
{
  Eigen::Matrix3f mat, vec;
  mat << 0.000536227, -1.56178e-05, -9.47391e-05, -1.56178e-05, 0.000297322, -0.000148785, -9.47391e-05, -0.000148785, 9.7827e-05;
  Eigen::Vector3f val;

  pcl::eigen33 (mat, vec, val);
  
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

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
