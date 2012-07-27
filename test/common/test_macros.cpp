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
 * $Id$
 */

#include <gtest/gtest.h>
#include <pcl/pcl_config.h>
#include <pcl/pcl_tests.h>
#include <pcl/common/eigen.h>
#include <vector>

using namespace pcl::test;

std::vector<float> v1, v2;
Eigen::Vector3f ev1, ev2;

TEST(MACROS, expect_eq_vectors_macro)
{
  for (size_t i = 0; i < 3; i++)
  {
    float val = static_cast<float> (i) * 1.5f;
    v1.push_back (val);
    v2.push_back (val);
    ev1[i] = val;
    ev2[i] = val;
  }

  EXPECT_EQ_VECTORS (v1, v2);
  EXPECT_EQ_VECTORS (ev1, ev2);
  EXPECT_EQ_VECTORS (v1, ev2);
  EXPECT_EQ_VECTORS (ev1, v2);
//  equal_vectors<std::vector<float>, std::vector<float> >(v1, v2);
}

TEST(MACROS, expect_near_vectors_macro)
{
  v1.clear ();
  v2.clear ();
  const static float epsilon = 1e-5f;
  for (size_t i = 0; i < 3; i++)
  {
    float val = static_cast<float> (i) * 1.5f;
    v1.push_back (val);
    v2.push_back (val + epsilon);
    ev1[i] = val;
    ev2[i] = val + epsilon;
  }
  EXPECT_NEAR_VECTORS (v1, v2, 2*epsilon);
  EXPECT_NEAR_VECTORS (ev1, ev2, 2*epsilon);
  EXPECT_NEAR_VECTORS (v1, ev2, 2*epsilon);
  EXPECT_NEAR_VECTORS (ev1, v2, 2*epsilon);
}

int 
main (int argc, char** argv)
{
#if ((PCL_MAJOR_VERSION == 1) && (PCL_MINOR_VERSION == 4))
  std::cerr << "1.4.0 detected" << std::endl;
#elif ((PCL_MAJOR_VERSION == 1) && (PCL_MINOR_VERSION == 3))
  std::cerr << "1.3.0 detected" << std::endl;
#endif
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
