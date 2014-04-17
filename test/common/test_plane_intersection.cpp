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
  Eigen::VectorXf line_a (6);
  Eigen::VectorXf line_b (6);

  //case 1
  line_a[0] = 0.01;
  line_a[1] = 0.02;
  line_a[2] = 0.03;
  line_a[3] = 0.4;
  line_a[4] = 0.5;
  line_a[5] = 0.6;

  line_b[0] = 0.1;
  line_b[1] = 0.2;
  line_b[2] = 0.3;
  line_b[3] = 0.04;
  line_b[4] = 0.05;
  line_b[5] = 0.06;

  Eigen::Vector4f p1, p2;
  lineToLineSegment (line_a, line_b, p1, p2);

  Eigen::Vector4f point_case_1;
  bool result_case_1 = lineWithLineIntersection (line_a, line_b, point_case_1);

  double sqr_dist_case_1 = (p1 - p2).squaredNorm ();

  double default_sqr_eps = 1e-4;
  EXPECT_GT (sqr_dist_case_1, default_sqr_eps);
  Eigen::Vector4f zero (0.0, 0.0, 0.0, 0.0);

  EXPECT_EQ (point_case_1[0], zero[0]);
  EXPECT_EQ (point_case_1[1], zero[1]);
  EXPECT_EQ (point_case_1[2], zero[2]);
  EXPECT_EQ (point_case_1[3], zero[3]);

  EXPECT_FALSE (result_case_1);
  
  pcl::ModelCoefficients line_a_mod;
  pcl::ModelCoefficients line_b_mod;

  std::vector<float> values_a_case_1;
  values_a_case_1.push_back (line_a[0]);
  values_a_case_1.push_back (line_a[1]);
  values_a_case_1.push_back (line_a[2]);
  values_a_case_1.push_back (line_a[3]);
  values_a_case_1.push_back (line_a[4]);
  values_a_case_1.push_back (line_a[5]);

  std::vector<float> values_b_case_1;
  values_b_case_1.push_back (line_b[0]);
  values_b_case_1.push_back (line_b[1]);
  values_b_case_1.push_back (line_b[2]);
  values_b_case_1.push_back (line_b[3]);
  values_b_case_1.push_back (line_b[4]);
  values_b_case_1.push_back (line_b[5]);

  line_a_mod.values = values_a_case_1;
  line_b_mod.values = values_b_case_1;

  Eigen::Vector4f point_mod_1;
  EXPECT_FALSE (lineWithLineIntersection (line_a_mod, line_b_mod, point_mod_1));
  EXPECT_EQ (result_case_1, lineWithLineIntersection (line_a_mod, line_b_mod, point_mod_1));

  EXPECT_EQ (point_mod_1[0], zero[0]);
  EXPECT_EQ (point_mod_1[1], zero[1]);
  EXPECT_EQ (point_mod_1[2], zero[2]);
  EXPECT_EQ (point_mod_1[3], zero[3]);

  //case 2
  line_a[0] = 0.00100;
  line_a[1] = 0.00200;
  line_a[2] = 0.00300;
  line_a[3] = 0.00400;
  line_a[4] = 0.00500;
  line_a[5] = 0.00600;

  line_b[0] = 0.00157;
  line_b[1] = 0.00233;
  line_b[2] = 0.00378;
  line_b[3] = 0.00495;
  line_b[4] = 0.00565;
  line_b[5] = 0.00666;

  lineToLineSegment (line_a, line_b, p1, p2);

  Eigen::Vector4f point_case_2;
  double sqr_eps_case_2 = 1e-1;
  bool result_case_2 = lineWithLineIntersection (line_a, line_b, point_case_2, sqr_eps_case_2);

  double sqr_dist_case_2 = (p1 - p2).squaredNorm ();
  EXPECT_LT (sqr_dist_case_2, sqr_eps_case_2);

  EXPECT_EQ (point_case_2[0], p1[0]);
  EXPECT_EQ (point_case_2[1], p1[1]);
  EXPECT_EQ (point_case_2[2], p1[2]);
  EXPECT_EQ (point_case_2[3], p1[3]);

  EXPECT_TRUE (result_case_2);

  pcl::ModelCoefficients line_a_mod_2;
  pcl::ModelCoefficients line_b_mod_2;

  std::vector<float> values_a_case_2;
  values_a_case_2.push_back (0.1000);
  values_a_case_2.push_back (0.2000);
  values_a_case_2.push_back (0.3000);
  values_a_case_2.push_back (0.4000);
  values_a_case_2.push_back (0.5000);
  values_a_case_2.push_back (0.6000);

  std::vector<float> values_b_case_2;
  values_b_case_2.push_back (0.1001);
  values_b_case_2.push_back (0.2001);
  values_b_case_2.push_back (0.3001);
  values_b_case_2.push_back (0.4001);
  values_b_case_2.push_back (0.5001);
  values_b_case_2.push_back (0.6001);

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

  EXPECT_LT (sqr_mod_act_2, sqr_mod_case_2);
  EXPECT_EQ (lineWithLineIntersection (coeff1, coeff2, point_mod_case_2, sqr_mod_case_2),
                        lineWithLineIntersection (line_a_mod_2, line_b_mod_2, point_mod_2, sqr_mod_case_2));
  EXPECT_TRUE (lineWithLineIntersection (line_a_mod_2, line_b_mod_2, point_mod_2, sqr_mod_case_2));

  EXPECT_EQ (point_mod_2[0], point_mod_case_2[0]);
  EXPECT_EQ (point_mod_2[1], point_mod_case_2[1]);
  EXPECT_EQ (point_mod_2[2], point_mod_case_2[2]);
  EXPECT_EQ (point_mod_2[3], point_mod_case_2[3]);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, planeWithPlaneIntersection)
{
  // Testing parallel planes
  const int k = 2;
  double x = 1.0;
  double y = 2.0;
  double z = 3.0;
  double w = 4.0;
  float xf = 1.0;
  float yf = 2.0;
  float zf = 3.0;
  float wf = 4.0;

  Eigen::Vector4d plane_a, plane_b;
  plane_a << x, y, z, w;
  plane_b << x, y, z, (w+k);

  Eigen::Vector4f plane_af, plane_bf;
  plane_af << xf, yf, zf, wf;
  plane_bf << xf, yf, zf, (wf+k);

  Eigen::VectorXd line;
  Eigen::VectorXf linef;

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 45));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 45));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 90));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 90));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 180));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 180));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 360));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 360));

  plane_b << x, y, z, w;
  plane_b *= k;
  plane_bf << xf, yf, zf, wf;
  plane_bf *= k;

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 45));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 45));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 90));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 90));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 180));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 180));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 360));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 360));

  // Overlapping planes
  plane_b.w() = w;
  plane_bf.w() = wf;

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 45));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 45));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 90));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 90));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 180));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 180));

  EXPECT_FALSE (planeWithPlaneIntersection (plane_a, plane_b, line, 360));
  EXPECT_FALSE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 360));

  // Orthogonal planes
  plane_a << 2.0, 1.0, -5.0, 6.0;
  plane_b << 2.0, 1.0, 1.0, 7.0;
  plane_af << 2.0, 1.0, -5.0, 6.0;
  plane_bf << 2.0, 1.0, 1.0, 7.0;

  EXPECT_TRUE (planeWithPlaneIntersection (plane_a, plane_b, line, 0));
  EXPECT_TRUE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 0));

  // General planes
  plane_a << 1.555, 0.894, 1.234, 3.567;
  plane_b << 0.743, 1.890, 6.789, 5.432;
  plane_af << 1.555, 0.894, 1.234, 3.567;
  plane_bf << 0.743, 1.890, 6.789, 5.432;

  EXPECT_TRUE (planeWithPlaneIntersection (plane_a, plane_b, line, 0));
  EXPECT_TRUE (planeWithPlaneIntersection (plane_af, plane_bf, linef, 0));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, threePlanesIntersection)
{
  // Testing with floats/doubles
  // Testing 2 parallel planes
  Eigen::Vector4d plane_a (1.0, 0.0, 0.0, -0.5);
  Eigen::Vector4d plane_b (1.0, 0.0, 0.0, 0.5);
  Eigen::Vector4d plane_c (0.0, 0.0, 1.0, -0.5);
  Eigen::Vector3d point (1.0, 2.0, 3.0);

  Eigen::Vector4f fplane_a (1.0, 0.0, 0.0, -0.5);
  Eigen::Vector4f fplane_b (1.0, 0.0, 0.0, 0.5);
  Eigen::Vector4f fplane_c (0.0, 0.0, 1.0, -0.5);
  Eigen::Vector3f fpoint (1.0, 2.0, 3.0);

  EXPECT_FALSE (threePlanesIntersection (plane_a, plane_b, plane_c, point, 1e-6));
  EXPECT_FALSE (threePlanesIntersection (fplane_a, fplane_b, fplane_c, fpoint, 1e-6));
  EXPECT_FALSE (threePlanesIntersection (plane_a, plane_b, plane_c, point, 1e-3));
  EXPECT_FALSE (threePlanesIntersection (fplane_a, fplane_b, fplane_c, fpoint, 1e-3));

  EXPECT_DOUBLE_EQ (1.0, point [0]);
  EXPECT_DOUBLE_EQ (2.0, point [1]);
  EXPECT_DOUBLE_EQ (3.0, point [2]);
  EXPECT_FLOAT_EQ (1.0, fpoint [0]);
  EXPECT_FLOAT_EQ (2.0, fpoint [1]);
  EXPECT_FLOAT_EQ (3.0, fpoint [2]);

  // Perfect box
  plane_b << 0.0, 1.0, 0.0, 0.5;
  fplane_b << 0.0, 1.0, 0.0, 0.5;

  EXPECT_TRUE (threePlanesIntersection (plane_a, plane_b, plane_c, point));
  EXPECT_TRUE (threePlanesIntersection (fplane_a, fplane_b, fplane_c, fpoint));

  EXPECT_DOUBLE_EQ (0.5, point [0]);
  EXPECT_DOUBLE_EQ (-0.5, point [1]);
  EXPECT_DOUBLE_EQ (0.5, point [2]);
  EXPECT_FLOAT_EQ (0.5, fpoint [0]);
  EXPECT_FLOAT_EQ (-0.5, fpoint [1]);
  EXPECT_FLOAT_EQ (0.5, fpoint [2]);

  // Random planes
  plane_a << 1.4564, 0.5465, -0.1325, 0.4685;
  plane_b << -1.5619, 5.5461, 5.4569, 2.9414;
  plane_c << 0.9852, 654.55, -0.1546, -45.1516;
  fplane_a << 1.4564, 0.5465, -0.1325, 0.4685;
  fplane_b << -1.5619, 5.5461, 5.4569, 2.9414;
  fplane_c << 0.9852, 654.55, -0.1546, -45.1516;

  EXPECT_TRUE (threePlanesIntersection (plane_a, plane_b, plane_c, point));
  EXPECT_TRUE (threePlanesIntersection (fplane_a, fplane_b, fplane_c, fpoint));

  EXPECT_NEAR (-0.413977, point [0], 1e-4);
  EXPECT_NEAR (0.0694323, point [1], 1e-4);
  EXPECT_NEAR (-0.728082, point [2], 1e-4);
  EXPECT_NEAR (-0.413977, fpoint [0], 1e-4);
  EXPECT_NEAR (0.0694323, fpoint [1], 1e-4);
  EXPECT_NEAR (-0.728082, fpoint [2], 1e-4);
}

//* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */


