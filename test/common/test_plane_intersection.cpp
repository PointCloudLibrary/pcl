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
 * $Id: test_plane_intersection.cpp 5686 2012-05-11 20:59:00Z gioia $
 */

#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <pcl/common/intersections.h>
#include <pcl/pcl_tests.h>


using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, lineWithLineIntersection)
{
  Eigen::VectorXf line_a(6);
  Eigen::VectorXf line_b(6);

  //case 1
  line_a[0] = 0.01f;
  line_a[1] = 0.02f;
  line_a[2] = 0.03f;
  line_a[3] = 0.4f;
  line_a[4] = 0.5f;
  line_a[5] = 0.6f;

  line_b[0] = 0.1f;
  line_b[1] = 0.2f;
  line_b[2] = 0.3f;
  line_b[3] = 0.04f;
  line_b[4] = 0.05f;
  line_b[5] = 0.06f;

  Eigen::Vector4f p1, p2;
  lineToLineSegment (line_a, line_b, p1, p2);

  Eigen::Vector4f point_case_1;
  bool result_case_1 = lineWithLineIntersection(line_a, line_b, point_case_1);

  double sqr_dist_case_1 = (p1 - p2).squaredNorm ();

  double default_sqr_eps = 1e-4;
  EXPECT_GT(sqr_dist_case_1, default_sqr_eps);
  Eigen::Vector4f zero(0.0f, 0.0f, 0.0f, 0.0f);

  EXPECT_EQ(point_case_1[0], zero[0]);
  EXPECT_EQ(point_case_1[1], zero[1]);
  EXPECT_EQ(point_case_1[2], zero[2]);
  EXPECT_EQ(point_case_1[3], zero[3]);

  EXPECT_FALSE(result_case_1);
  
  pcl::ModelCoefficients line_a_mod;
  pcl::ModelCoefficients line_b_mod;

  std::vector<float> values_a_case_1;
  values_a_case_1.push_back(line_a[0]);
  values_a_case_1.push_back(line_a[1]);
  values_a_case_1.push_back(line_a[2]);
  values_a_case_1.push_back(line_a[3]);
  values_a_case_1.push_back(line_a[4]);
  values_a_case_1.push_back(line_a[5]);

  std::vector<float> values_b_case_1;
  values_b_case_1.push_back(line_b[0]);
  values_b_case_1.push_back(line_b[1]);
  values_b_case_1.push_back(line_b[2]);
  values_b_case_1.push_back(line_b[3]);
  values_b_case_1.push_back(line_b[4]);
  values_b_case_1.push_back(line_b[5]);

  line_a_mod.values = values_a_case_1;
  line_b_mod.values = values_b_case_1;

  Eigen::Vector4f point_mod_1;
  EXPECT_FALSE(lineWithLineIntersection(line_a_mod, line_b_mod, point_mod_1));
  EXPECT_EQ(result_case_1, lineWithLineIntersection(line_a_mod, line_b_mod, point_mod_1));

  EXPECT_EQ(point_mod_1[0], zero[0]);
  EXPECT_EQ(point_mod_1[1], zero[1]);
  EXPECT_EQ(point_mod_1[2], zero[2]);
  EXPECT_EQ(point_mod_1[3], zero[3]);

  //case 2
  line_a[0] = 0.00100f;
  line_a[1] = 0.00200f;
  line_a[2] = 0.00300f;
  line_a[3] = 0.00400f;
  line_a[4] = 0.00500f;
  line_a[5] = 0.00600f;

  line_b[0] = 0.00157f;
  line_b[1] = 0.00233f;
  line_b[2] = 0.00378f;
  line_b[3] = 0.00495f;
  line_b[4] = 0.00565f;
  line_b[5] = 0.00666f;

  lineToLineSegment (line_a, line_b, p1, p2);

  Eigen::Vector4f point_case_2;
  double sqr_eps_case_2 = 1e-1;
  bool result_case_2 = lineWithLineIntersection(line_a, line_b, point_case_2, sqr_eps_case_2);

  double sqr_dist_case_2 = (p1 - p2).squaredNorm ();
  EXPECT_LT(sqr_dist_case_2, sqr_eps_case_2);

  EXPECT_EQ(point_case_2[0], p1[0]);
  EXPECT_EQ(point_case_2[1], p1[1]);
  EXPECT_EQ(point_case_2[2], p1[2]);
  EXPECT_EQ(point_case_2[3], p1[3]);

  EXPECT_TRUE(result_case_2);

  pcl::ModelCoefficients line_a_mod_2;
  pcl::ModelCoefficients line_b_mod_2;

  std::vector<float> values_a_case_2;
  values_a_case_2.push_back(0.1000f);
  values_a_case_2.push_back(0.2000f);
  values_a_case_2.push_back(0.3000f);
  values_a_case_2.push_back(0.4000f);
  values_a_case_2.push_back(0.5000f);
  values_a_case_2.push_back(0.6000f);

  std::vector<float> values_b_case_2;
  values_b_case_2.push_back(0.1001f);
  values_b_case_2.push_back(0.2001f);
  values_b_case_2.push_back(0.3001f);
  values_b_case_2.push_back(0.4001f);
  values_b_case_2.push_back(0.5001f);
  values_b_case_2.push_back(0.6001f);

  line_a_mod_2.values = values_a_case_2;
  line_b_mod_2.values = values_b_case_2;

  Eigen::Vector4f point_mod_2;
  Eigen::Vector4f point_mod_case_2;
  double sqr_mod_case_2 = 1e-1;

  Eigen::VectorXf coeff1 = Eigen::VectorXf::Map (&line_a_mod_2.values[0], line_a_mod_2.values.size ());
  Eigen::VectorXf coeff2 = Eigen::VectorXf::Map (&line_b_mod_2.values[0], line_b_mod_2.values.size ());

  Eigen::Vector4f p1_mod, p2_mod;
  lineToLineSegment (coeff1, coeff2, p1_mod, p2_mod);
  double sqr_mod_act_2 = (p1_mod - p2_mod).squaredNorm ();

  EXPECT_LT(sqr_mod_act_2, sqr_mod_case_2);
  EXPECT_EQ(lineWithLineIntersection (coeff1, coeff2, point_mod_case_2, sqr_mod_case_2),
                        lineWithLineIntersection(line_a_mod_2, line_b_mod_2, point_mod_2, sqr_mod_case_2));
  EXPECT_TRUE(lineWithLineIntersection(line_a_mod_2, line_b_mod_2, point_mod_2, sqr_mod_case_2));

  EXPECT_EQ(point_mod_2[0], point_mod_case_2[0]);
  EXPECT_EQ(point_mod_2[1], point_mod_case_2[1]);
  EXPECT_EQ(point_mod_2[2], point_mod_case_2[2]);
  EXPECT_EQ(point_mod_2[3], point_mod_case_2[3]);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, planeWithPlaneIntersection)
{
  //Testing parallel planes
  const int k = 2;
  float x = 1.0f;
  float y = 2.0f;
  float z = 3.0f;
  float w = 4.0f;
  Eigen::Vector4f plane_a;
  plane_a.x() = x;
  plane_a.y() = y;
  plane_a.z() = z;
  plane_a.w() = w;

  EXPECT_EQ(1.0f, plane_a.x());
  EXPECT_EQ(2.0f, plane_a.y());
  EXPECT_EQ(3.0f, plane_a.z());
  EXPECT_EQ(4.0f, plane_a.w());

  Eigen::Vector4f plane_b;
  plane_b.x() = x;
  plane_b.y() = y;
  plane_b.z() = z;
  plane_b.w() = w + k;

  EXPECT_EQ(1.0f, plane_b.x());
  EXPECT_EQ(2.0f, plane_b.y());
  EXPECT_EQ(3.0f, plane_b.z());
  EXPECT_EQ(6.0f, plane_b.w());

  Eigen::VectorXf line;

  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 45));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 90));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 180));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 360));

  plane_b.x() = k * x;
  plane_b.y() = k * y;
  plane_b.z() = k * z;
  plane_b.w() = k * w;

  EXPECT_EQ(2.0f, plane_b.x());
  EXPECT_EQ(4.0f, plane_b.y());
  EXPECT_EQ(6.0f, plane_b.z());
  EXPECT_EQ(8.0f, plane_b.w());

  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 45));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 90));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 180));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 360));

  //overlapping planes
  plane_b.w() = w;
  EXPECT_EQ(4.0f, plane_b.w());

  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 45));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 90));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 180));
  std::cout << std::endl;
  EXPECT_FALSE(planeWithPlaneIntersection(plane_a, plane_b, line, 360));

  //orthogonal planes
  plane_a.x() = 2.0f;
  plane_a.y() = 1.0f;
  plane_a.z() = -5.0f;
  plane_a.w() = 6.0f;
  EXPECT_EQ(2.0f, plane_a.x());
  EXPECT_EQ(1.0f, plane_a.y());
  EXPECT_EQ(-5.0f, plane_a.z());
  EXPECT_EQ(6.0f, plane_a.w());

  plane_b.x() = 2.0f;
  plane_b.y() = 1.0f;
  plane_b.z() = 1.0f;
  plane_b.w() = 7.0f;

  EXPECT_EQ(2.0f, plane_b.x());
  EXPECT_EQ(1.0f, plane_b.y());
  EXPECT_EQ(1.0f, plane_b.z());
  EXPECT_EQ(7.0f, plane_b.w());

  std::cout << std::endl;
  EXPECT_TRUE(planeWithPlaneIntersection(plane_a, plane_b, line, 0));

  //general planes
  plane_a.x() = 1.555f;
  plane_a.y() = 0.894f;
  plane_a.z() = 1.234f;
  plane_a.w() = 3.567f;

  plane_b.x() = 0.743f;
  plane_b.y() = 1.890f;
  plane_b.z() = 6.789f;
  plane_b.w() = 5.432f;

  std::cout << std::endl;
  EXPECT_TRUE(planeWithPlaneIntersection(plane_a, plane_b, line, 0));

}

//* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

