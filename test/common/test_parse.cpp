/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
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

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/console/parse.h>

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, parse_double)
{
  const char arg0[] = {"test_name"};
  const char arg1[] = {"double"};
  const char arg2_1[] = {"-3.14e+0"};
  const char arg2_2[] = {"-3.14f+0"};
  const char arg2_3[] = {"3.14e+309"};

  const char* argv_1[] = { &arg0[0], &arg1[0], &arg2_1[0], nullptr};
  const char* argv_2[] = { &arg0[0], &arg1[0], &arg2_2[0], nullptr};
  const char* argv_3[] = { &arg0[0], &arg1[0], &arg2_3[0], nullptr};
  const int argc = static_cast<int> (sizeof (argv_1)/sizeof (argv_1[0])) - 1;

  int index = -1;
  double value = 0;

  index = pcl::console::parse_argument (argc, argv_1, "double", value);
  EXPECT_DOUBLE_EQ(-3.14, value);
  EXPECT_EQ(1, index);

  index = pcl::console::parse_argument (argc, argv_2, "double", value);
  EXPECT_EQ(-1, index);

  index = pcl::console::parse_argument (argc, argv_3, "double", value);
  EXPECT_EQ(-1, index);
}

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, parse_float)
{
  const char arg0[] = {"test_name"};
  const char arg1[] = {"float"};
  const char arg2_1[] = {"-3.14e+0"};
  const char arg2_2[] = {"-3.14f+0"};
  const char arg2_3[] = {"3.14e+39"};

  const char* argv_1[] = { &arg0[0], &arg1[0], &arg2_1[0], nullptr};
  const char* argv_2[] = { &arg0[0], &arg1[0], &arg2_2[0], nullptr};
  const char* argv_3[] = { &arg0[0], &arg1[0], &arg2_3[0], nullptr};
  const int argc = static_cast<int> (sizeof (argv_1)/sizeof (argv_1[0])) - 1;

  int index = -1;
  float value = 0;

  index = pcl::console::parse_argument (argc, argv_1, "float", value);
  EXPECT_FLOAT_EQ(-3.14, value);
  EXPECT_EQ(1, index);

  index = pcl::console::parse_argument (argc, argv_2, "float", value);
  EXPECT_EQ(-1, index);

  index = pcl::console::parse_argument (argc, argv_3, "float", value);
  EXPECT_EQ(-1, index);
}

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, parse_longint)
{
  const char arg0[] = {"test_name"};
  const char arg1[] = {"long_int"};
  const char arg2_1[] = {"-314"};
  const char arg2_2[] = {"3.14"};
  const char arg2_3[] = {"18446744073709551615"};

  const char* argv_1[] = { &arg0[0], &arg1[0], &arg2_1[0], nullptr};
  const char* argv_2[] = { &arg0[0], &arg1[0], &arg2_2[0], nullptr};
  const char* argv_3[] = { &arg0[0], &arg1[0], &arg2_3[0], nullptr};
  const int argc = static_cast<int> (sizeof (argv_1)/sizeof (argv_1[0])) - 1;

  int index = -1;
  long int value = 0;

  index = pcl::console::parse_argument (argc, argv_1, "long_int", value);
  EXPECT_EQ(-314, value);
  EXPECT_EQ(1, index);

  index = pcl::console::parse_argument (argc, argv_2, "long_int", value);
  EXPECT_EQ(-1, index);

  index = pcl::console::parse_argument (argc, argv_3, "long_int", value);
  EXPECT_EQ(-1, index);
}

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, parse_unsignedint)
{
  const char arg0[] = {"test_name"};
  const char arg1[] = {"unsigned_int"};
  const char arg2_1[] = {"314"};
  const char arg2_2[] = {"-314"};
  const char arg2_3[] = {"18446744073709551615"};

  const char* argv_1[] = { &arg0[0], &arg1[0], &arg2_1[0], nullptr};
  const char* argv_2[] = { &arg0[0], &arg1[0], &arg2_2[0], nullptr};
  const char* argv_3[] = { &arg0[0], &arg1[0], &arg2_3[0], nullptr};
  const int argc = static_cast<int> (sizeof (argv_1)/sizeof (argv_1[0])) - 1;

  int index = -1;
  unsigned int value = 53;

  index = pcl::console::parse_argument (argc, argv_1, "unsigned_int", value);
  EXPECT_EQ(314, value);
  EXPECT_EQ(1, index);

  index = pcl::console::parse_argument (argc, argv_2, "unsigned_int", value);
  EXPECT_EQ(-1, index);

  index = pcl::console::parse_argument (argc, argv_3, "unsigned_int", value);
  EXPECT_EQ(-1, index);
}

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, parse_int)
{
  const char arg0[] = {"test_name"};
  const char arg1[] = {"int"};
  const char arg2_1[] = {"-314"};
  const char arg2_2[] = {"3.14"};
  const char arg2_3[] = {"18446744073709551615"};

  const char* argv_1[] = { &arg0[0], &arg1[0], &arg2_1[0], nullptr};
  const char* argv_2[] = { &arg0[0], &arg1[0], &arg2_2[0], nullptr};
  const char* argv_3[] = { &arg0[0], &arg1[0], &arg2_3[0], nullptr};
  const int argc = static_cast<int> (sizeof (argv_1)/sizeof (argv_1[0])) - 1;

  int index = -1;
  int value = 0;

  index = pcl::console::parse_argument (argc, argv_1, "int", value);
  EXPECT_EQ(-314, value);
  EXPECT_EQ(1, index);

  index = pcl::console::parse_argument (argc, argv_2, "int", value);
  EXPECT_EQ(-1, index);

  index = pcl::console::parse_argument (argc, argv_3, "int", value);
  EXPECT_EQ(-1, index);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
