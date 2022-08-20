/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/io/split.h>
#include <pcl/test/gtest.h>

#include <vector>

TEST(PCL, TestIdentitySplit)
{
  std::vector<std::string> tokens;

  // identity test, empty string as input
  pcl::split(tokens, "", " \r\t");
  EXPECT_EQ(tokens, std::vector<std::string>());
}

TEST(PCL, TestNonEmptyDelimitersSplit)
{
  std::vector<std::string> tokens;

  // test non-empty string with just the delimiters
  pcl::split(tokens, "\r\t ", " \r\t");
  EXPECT_EQ(tokens, std::vector<std::string>());
}

TEST(PCL, TestTokenWithoutDelimitersSplit)
{
  std::vector<std::string> tokens;

  // test a string without delimiters
  pcl::split(tokens, "abcd", " \r\t");
  EXPECT_EQ(tokens, std::vector<std::string>{"abcd"});
}

TEST(PCL, TestSimpleSplit1)
{
  std::vector<std::string> tokens;
  const auto output = std::vector<std::string>{
      "aabb", "ccdd", "eeff", "gghh", "iijj", "kkll", "mmnn", "oopp"};

  // test simple combination of all the delimiters
  const std::string input_1 = "aabb ccdd\reeff\tgghh \riijj \tkkll\r\tmmnn \r\toopp";
  pcl::split(tokens, input_1, " \r\t");
  EXPECT_EQ(tokens, output);
}

TEST(PCL, TestSimpleSplit2)
{
  std::vector<std::string> tokens;
  const auto output = std::vector<std::string>{
      "aabb", "ccdd", "eeff", "gghh", "iijj", "kkll", "mmnn", "oopp"};

  // same as input_1 but we have whitespaces in the front and in the back
  const std::string input_2 =
      "   aabb ccdd\reeff\tgghh \riijj \tkkll\r\tmmnn \r\toopp   ";
  pcl::split(tokens, input_2, " \r\t");
  EXPECT_EQ(tokens, output);
}

TEST(PCL, TestSimpleSplit3)
{
  std::vector<std::string> tokens;
  const auto output = std::vector<std::string>{
      "aabb", "ccdd", "eeff", "gghh", "iijj", "kkll", "mmnn", "oopp"};

  // same as input_2 but we have some double delimiters
  const std::string input_3 =
      "   aabb  ccdd\r\reeff\t\tgghh \r\r\riijj \t\tkkll\r\t\tmmnn \r\r\toopp   ";
  pcl::split(tokens, input_3, " \r\t");
  EXPECT_EQ(tokens, output);
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
