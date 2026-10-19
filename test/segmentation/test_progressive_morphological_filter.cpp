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

namespace {
pcl::Indices
classifyScaledCloud(float scale)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(13);
  // The raised seven-point plateau survives the smaller openings and is
  // evaluated against the height threshold at the final window size.
  for (int i = -6; i <= 6; ++i) {
    const float x = 0.25f * static_cast<float>(i) * scale;
    const float z = (i >= -3 && i <= 3) ? 0.8f * scale : 0.0f;
    cloud->emplace_back(x, 0.0f, z);
  }

  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setCellSize(0.25f * scale);
  filter.setMaxWindowSize(2.25f * scale);
  filter.setBase(2.0f);
  filter.setExponential(true);
  filter.setSlope(1.0f);
  filter.setInitialDistance(0.1f * scale);
  filter.setMaxDistance(100.0f * scale);

  pcl::Indices ground;
  filter.extract(ground);
  return ground;
}
} // namespace

TEST(ProgressiveMorphologicalFilter, FractionalMaximumWindowSize)
{
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> filter;
  static_assert(std::is_same_v<decltype(filter.getMaxWindowSize()), float>);

  filter.setMaxWindowSize(6.25f);

  EXPECT_FLOAT_EQ(filter.getMaxWindowSize(), 6.25f);
}

TEST(ProgressiveMorphologicalFilter, ClassificationIsInvariantToCoordinateScale)
{
  // Scaling the coordinates and every distance parameter must not change which
  // points are classified as ground.
  const auto ground = classifyScaledCloud(1.0f);
  EXPECT_EQ(ground.size(), 13u);
  EXPECT_EQ(ground, classifyScaledCloud(4.0f));
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
