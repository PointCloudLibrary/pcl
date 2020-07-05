/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/experimental/executor/executor.h>

using namespace executor;

TEST (Executor, InlineExecutor)
{
  inline_executor<blocking_t::always_t> exec;
  int a = 0;
  exec.execute([&] { a = 1; });
  EXPECT_TRUE(a == 1);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
