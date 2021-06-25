/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2020-, Open Perception
 *
 * All rights reserved
 */

#include <pcl/filters/experimental/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/test/gtest.h>
#include <pcl/point_types.h>

#include <cmath>

using namespace pcl;
using namespace pcl::io;
using namespace Eigen;

PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
PointCloud<PointXYZRGB>::Ptr cloud_organized(new PointCloud<PointXYZRGB>);

const float PRECISION = Eigen::NumTraits<float>::dummy_precision() * 2;

// helper function
template <typename PointT>
bool
isPointApprox(const PointT& p1, const PointT& p2, const float precision)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)) <=
         precision;
}

TEST(VoxelGridEquivalentTest, GridFilters)
{
  PointCloud<PointXYZ> new_out_cloud, old_out_cloud;

  pcl::experimental::VoxelGrid<PointXYZ> new_grid;
  new_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  new_grid.setInputCloud(cloud);
  new_grid.filter(new_out_cloud);

  pcl::VoxelGrid<PointXYZ> old_grid;
  old_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  old_grid.setInputCloud(cloud);
  old_grid.filter(old_out_cloud);

  EXPECT_EQ(new_out_cloud.size(), old_out_cloud.size());
  EXPECT_EQ(new_out_cloud.width, old_out_cloud.width);
  EXPECT_EQ(new_out_cloud.height, old_out_cloud.height);

  for (size_t i = 0; i < new_out_cloud.size(); ++i) {
    bool same_pt_found = false;
    for (size_t j = 0; j < old_out_cloud.size(); ++j) {
      if (isPointApprox(new_out_cloud[i], old_out_cloud[j], PRECISION)) {
        same_pt_found = true;
        break;
      }
    }

    EXPECT_TRUE(same_pt_found);
  }
}

TEST(GridInfoEquivalentTest, GridFilters)
{
  PointCloud<PointXYZ> new_out_cloud, old_out_cloud;

  experimental::VoxelGrid<PointXYZ> new_grid;
  new_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  new_grid.setInputCloud(cloud);
  new_grid.filter(new_out_cloud);

  pcl::VoxelGrid<PointXYZ> old_grid;
  old_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  old_grid.setInputCloud(cloud);
  old_grid.filter(old_out_cloud);

  const Eigen::Vector3i new_min_b = new_grid.getMinBoxCoordinates();
  const Eigen::Vector3i old_min_b = old_grid.getMinBoxCoordinates();
  EXPECT_TRUE(new_min_b.isApprox(old_min_b, PRECISION));

  const Eigen::Vector3i new_max_b = new_grid.getMaxBoxCoordinates();
  const Eigen::Vector3i old_max_b = old_grid.getMaxBoxCoordinates();
  EXPECT_TRUE(new_max_b.isApprox(old_max_b, PRECISION));

  const Eigen::Vector3i new_div_b = new_grid.getNrDivisions();
  const Eigen::Vector3i old_div_b = old_grid.getNrDivisions();
  EXPECT_TRUE(new_div_b.isApprox(old_div_b, PRECISION));

  const Eigen::Vector3i new_divb_mul = new_grid.getDivisionMultiplier();
  const Eigen::Vector3i old_divb_mul = old_grid.getDivisionMultiplier();
  EXPECT_TRUE(new_divb_mul.isApprox(old_divb_mul, PRECISION));
}

int
main(int argc, char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 3) {
    std::cerr << "No test file given. Please download `bun0.pcd` and "
                 "`milk_cartoon_all_small_clorox.pcd` and pass their paths to the test."
              << std::endl;
    return (-1);
  }

  // Load a standard PCD file from disk
  pcl::io::loadPCDFile(argv[1], *cloud);
  loadPCDFile(argv[2], *cloud_organized);

  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
