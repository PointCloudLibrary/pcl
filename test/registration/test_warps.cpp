/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 */

#include <Eigen/Geometry> // for Quaternionf

#include <pcl/test/gtest.h>

#include <pcl/point_types.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/warp_point_rigid_6d.h>

using namespace pcl;
using namespace pcl::registration;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, WarpPointRigid6DFloat)
{
  WarpPointRigid6D<PointXYZ, PointXYZ, float> warp;
  Eigen::Quaternionf q (0.4455f, 0.9217f, 0.3382f, 0.3656f);
  q.normalize ();
  Eigen::Vector3f t (0.82550f, 0.11697f, 0.44864f);

  Eigen::VectorXf p (6);
  p[0] = t.x (); p[1] = t.y (); p[2] = t.z (); p[3] = q.x (); p[4] = q.y (); p[5] = q.z ();
  warp.setParam (p);

  PointXYZ pin (1, 2, 3), pout;
  warp.warpPoint (pin, pout);
  EXPECT_NEAR (pout.x, 4.15963f, 1e-5);
  EXPECT_NEAR (pout.y, -1.51363f, 1e-5);
  EXPECT_NEAR (pout.z, 0.922648f, 1e-5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, WarpPointRigid6DDouble)
{
  WarpPointRigid6D<PointXYZ, PointXYZ, double> warp;
  Eigen::Quaterniond q (0.4455, 0.9217, 0.3382, 0.3656);
  q.normalize ();
  Eigen::Vector3d t (0.82550, 0.11697, 0.44864);

  Eigen::VectorXd p (6);
  p[0] = t.x (); p[1] = t.y (); p[2] = t.z (); p[3] = q.x (); p[4] = q.y (); p[5] = q.z ();
  warp.setParam (p);

  PointXYZ pin (1, 2, 3), pout;
  warp.warpPoint (pin, pout);
  EXPECT_NEAR (pout.x, 4.15963, 1e-5);
  EXPECT_NEAR (pout.y, -1.51363, 1e-5);
  EXPECT_NEAR (pout.z, 0.922648, 1e-5);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
