/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2020-, Open Perception
 *
 * All rights reserved
 */

#include <pcl/filters/lambda_filter_indices.h>
#include <pcl/test/gtest.h>
#include <pcl/point_types.h>

using namespace pcl;

TEST(LambdaFilterIndices, CheckCompatibility)
{
  const auto copy_all = [](PointXYZ, index_t) { return 0; };
  EXPECT_TRUE((detail::is_lambda_point_filter_v<PointXYZ, decltype(copy_all)>));
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
