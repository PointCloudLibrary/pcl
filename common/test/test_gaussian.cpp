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
 *
 */
/** \author Nizar Sallem */

#include <gtest/gtest.h>
#include "pcl/common/gaussian.h"

TEST(PCL, GaussianKernel)
{
  Eigen::VectorXf kernel(31);
  kernel << 0.000888059, 0.00158611, 0.00272177, 0.00448744, 0.00710844, 0.0108188, 0.0158201, 0.0222264, 0.0300025, 0.0389112, 0.0484864, 0.0580487, 0.0667719, 0.0737944, 0.0783576, 0.0799405, 0.0783576, 0.0737944, 0.0667719, 0.0580487, 0.0484864, 0.0389112, 0.0300025, 0.0222264, 0.0158201, 0.0108188, 0.00710844, 0.00448744, 0.00272177, 0.00158611, 0.000888059;

  Eigen::VectorXf derivative(35);
  derivative << 0.000168673, 0.000307151, 0.000535285, 0.000892304, 0.00142183, 0.00216388, 0.00314209, 0.00434741, 0.00572143, 0.00714516, 0.00843934, 0.00938163, 0.00974186, 0.0093305, 0.00804947, 0.0059307, 0.00314871, 0, -0.00314871, -0.0059307, -0.00804947, -0.0093305, -0.00974186, -0.00938163, -0.00843934, -0.00714516, -0.00572143, -0.00434741, -0.00314209, -0.00216388, -0.00142183, -0.000892304, -0.000535285, -0.000307151, -0.000168673;
  pcl::GaussianKernel gk;
  Eigen::VectorXf computed_kernel, computed_derivative;

  // Test kernel only version
  gk.compute(5, computed_kernel);
  EXPECT_EQ(kernel.size (), computed_kernel.size ());
  for(int i = 0; i < kernel.size (); i++)
    EXPECT_NEAR(kernel[i], computed_kernel[i], 1e-4);

  // Test kernel and derivative version
  gk.compute(5, computed_kernel, computed_derivative);
  EXPECT_EQ(kernel.size (), computed_kernel.size ());
  for(int i = 0; i < kernel.size (); i++)
    EXPECT_NEAR(kernel[i], computed_kernel[i], 1e-4);
  EXPECT_EQ(derivative.size (), computed_derivative.size ());
  for(int i = 0; i < derivative.size (); i++)
    EXPECT_NEAR(derivative[i], computed_derivative[i], 1e-4);
}

int main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
