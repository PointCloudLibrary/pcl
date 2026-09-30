/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2021, Open Perception
 *
 * All rights reserved
 */

#include <pcl/filters/experimental/voxel_grid_label.h>
#include <pcl/test/gtest.h>
#include <pcl/point_types.h>

using namespace pcl;

TEST(Label, ExperimentalVoxelGridLabelEquivalency)
{
  PointCloud<PointXYZRGBL>::Ptr cloud(new PointCloud<PointXYZRGBL>);

  const std::vector<std::uint32_t> labels = {0, 0, 0, 0, 1, 2, 3, 4, 5, 5};
  // 0.00, 0.01, ...
  for (size_t i = 0; i < 10; ++i) {
    PointXYZRGBL pt(i / 100., i / 100., i / 100., i, i, i, labels.at(i));
    cloud->push_back(pt);
  }

  PointCloud<PointXYZRGBL> out;

  pcl::experimental::VoxelGridLabel vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.05, 0.05, 0.05);
  vg.filter(out);

  auto pt_cmp = [](const PointXYZRGBL& p1, const PointXYZRGBL& p2) -> bool {
    return p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y) ||
           (p1.x == p2.x && p1.y == p2.y && p1.z > p2.z);
  };
  std::sort(out.begin(), out.end(), pt_cmp);

  ASSERT_EQ(out.size(), 2);

  EXPECT_EQ(out.at(0).label, 5);
  EXPECT_EQ(out.at(1).label, 0);

  EXPECT_EQ(out.at(0).r, 7);
  EXPECT_EQ(out.at(1).r, 2);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
