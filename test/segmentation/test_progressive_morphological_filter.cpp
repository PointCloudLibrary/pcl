/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/test/gtest.h>
#include <pcl/point_types.h>

#include <type_traits>

TEST(ProgressiveMorphologicalFilter, FractionalMaximumWindowSize)
{
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> filter;
  static_assert(std::is_same_v<decltype(filter.getMaxWindowSize()), float>);

  filter.setMaxWindowSize(6.25f);

  EXPECT_FLOAT_EQ(filter.getMaxWindowSize(), 6.25f);
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
