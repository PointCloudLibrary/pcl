/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2021-, Open Perception
 *
 * All rights reserved
 */

#include <pcl/filters/experimental/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>

#include <cmath>
#include <limits>

using namespace pcl;
using namespace pcl::io;

PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
PointCloud<PointXYZRGB>::Ptr cloud_rgb(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGBA>::Ptr cloud_rgba(new PointCloud<PointXYZRGBA>);

const float PRECISION = Eigen::NumTraits<float>::dummy_precision() * 2;

template <typename PointT>
void
EXPECT_POINT_EQ(const PointT& pt1, const PointT& pt2)
{
  EXPECT_XYZ_NEAR(pt1, pt2, PRECISION);
}

template <typename>
void
EXPECT_POINT_EQ(const PointXYZRGB& pt1, const PointXYZRGB& pt2)
{
  EXPECT_XYZ_NEAR(pt1, pt2, PRECISION);
  EXPECT_RGB_EQ(pt1, pt2);
}

template <typename>
void
EXPECT_POINT_EQ(const PointXYZRGBA& pt1, const PointXYZRGBA& pt2)
{
  EXPECT_XYZ_NEAR(pt1, pt2, PRECISION);
  EXPECT_RGBA_EQ(pt1, pt2);
}

template <typename PointT>
void
EXPECT_POINTS_EQ(PointCloud<PointT>& pc1, PointCloud<PointT>& pc2)
{
  ASSERT_EQ(pc1.size(), pc2.size());

  auto pt_cmp = [](const PointT& p1, const PointT& p2) -> bool {
    return p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y) ||
           (p1.x == p2.x && p1.y == p2.y && p1.z > p2.z);
  };
  std::sort(pc1.begin(), pc1.end(), pt_cmp);
  std::sort(pc2.begin(), pc2.end(), pt_cmp);

  for (size_t i = 0; i < pc1.size(); ++i)
    EXPECT_POINT_EQ(pc1.at(i), pc2.at(i));
}

TEST(SetUp, ExperimentalVoxelGridEquivalency)
{
  PointCloud<PointXYZ> new_out_cloud, old_out_cloud;

  experimental::VoxelGrid<PointXYZ> new_grid;
  pcl::VoxelGrid<PointXYZ> old_grid;
  new_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  old_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  new_grid.setInputCloud(cloud);
  old_grid.setInputCloud(cloud);
  new_grid.filter(new_out_cloud);
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

TEST(HashingPoint, ExperimentalVoxelGridEquivalency)
{
  Eigen::Vector4f min_p, max_p;
  getMinMax3D<PointXYZ>(*cloud, min_p, max_p);

  Eigen::Vector4i min_b, max_b, div_b, divb_mul;
  Eigen::Array4f inverse_leaf_size = 1 / Eigen::Array4f::Constant(0.02);
  min_b = (min_p.array() * inverse_leaf_size).floor().cast<int>();
  max_b = (max_p.array() * inverse_leaf_size).floor().cast<int>();
  div_b = (max_b - min_b).array() + 1;
  divb_mul = Eigen::Vector4i(1, div_b[0], div_b[0] * div_b[1], 0);

  // Copied from the old VoxelGrid as there is no dedicated method auto old_hash =
  auto old_hash = [&](const PointXYZ& p) {
    int ijk0 = static_cast<int>(std::floor(p.x * inverse_leaf_size[0]) -
                                static_cast<float>(min_b[0]));
    int ijk1 = static_cast<int>(std::floor(p.y * inverse_leaf_size[1]) -
                                static_cast<float>(min_b[1]));
    int ijk2 = static_cast<int>(std::floor(p.z * inverse_leaf_size[2]) -
                                static_cast<float>(min_b[2]));

    // Compute the centroid leaf index
    return ijk0 * divb_mul[0] + ijk1 * divb_mul[1] + ijk2 * divb_mul[2];
  };

  for (size_t i = 0; i < cloud->size(); ++i) {
    if (isXYZFinite(cloud->at(i))) {
      const size_t new_hash = experimental::hashPoint(
          cloud->at(i), inverse_leaf_size, min_b, divb_mul[1], divb_mul[2]);
      EXPECT_EQ(new_hash, old_hash(cloud->at(i)));
    }
  }
}

TEST(LeafLayout, ExperimentalVoxelGridEquivalency)
{
  PointCloud<PointXYZ> new_out_cloud, old_out_cloud;

  experimental::VoxelGrid<PointXYZ> new_grid;
  pcl::VoxelGrid<PointXYZ> old_grid;
  new_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  old_grid.setLeafSize(0.02f, 0.02f, 0.02f);
  new_grid.setInputCloud(cloud);
  old_grid.setInputCloud(cloud);
  new_grid.setSaveLeafLayout(true);
  old_grid.setSaveLeafLayout(true);
  new_grid.filter(new_out_cloud);
  old_grid.filter(old_out_cloud);

  const auto new_leaf = new_grid.getLeafLayout();
  const auto old_leaf = old_grid.getLeafLayout();
  ASSERT_EQ(new_leaf.size(), old_leaf.size());

  // Centroid indices are different from the old implememnt because of the order of
  // iterating grids is different.
  // But the index should still point to the same point

  // leaf layout content
  for (size_t i = 0; i < new_leaf.size(); ++i) {
    if (old_leaf.at(i) == -1) {
      EXPECT_EQ(new_leaf.at(i), -1);
    }
    else {
      ASSERT_NE(new_leaf.at(i), -1);
      const auto& new_pt = new_out_cloud.at(new_leaf.at(i));
      const auto& old_pt = old_out_cloud.at(old_leaf.at(i));
      EXPECT_POINT_EQ(new_pt, old_pt);
    }
  }

  // getCentroidIndex
  for (const auto& pt : *cloud) {
    const auto& new_pt = new_out_cloud.at(new_grid.getCentroidIndex(pt));
    const auto& old_pt = old_out_cloud.at(old_grid.getCentroidIndex(pt));
    EXPECT_POINT_EQ(new_pt, old_pt);
  }

  for (const auto& pt : new_out_cloud) {
    const Eigen::MatrixXi random_pt = Eigen::MatrixXi::Random(3, 1);

    const auto new_idx1 = new_grid.getNeighborCentroidIndices(pt, random_pt)[0];
    const auto new_idx2 =
        new_grid.getNeighborCentroidIndices(pt.x, pt.y, pt.z, random_pt)[0];

    const auto old_idx = old_grid.getNeighborCentroidIndices(pt, random_pt)[0];

    EXPECT_EQ(new_idx1, old_idx);
    EXPECT_EQ(new_idx2, old_idx);
  }
}

template <typename T>
class PointTypesTest : public ::testing::Test {
protected:
  const std::tuple<PointCloud<PointXYZ>::Ptr,
                   PointCloud<PointXYZRGB>::Ptr,
                   PointCloud<PointXYZRGBA>::Ptr>
      clouds{cloud, cloud_rgb, cloud_rgba};
};
using PointTypes = ::testing::Types<PointXYZ, PointXYZRGB, PointXYZRGBA>;
TYPED_TEST_SUITE(PointTypesTest, PointTypes);

TYPED_TEST(PointTypesTest, ExperimentalVoxelGridEquivalency)
{
  using PointT = TypeParam;
  using PointCloudPtr = typename PointCloud<PointT>::Ptr;

  PointCloud<PointT> new_out, old_out;
  const PointCloudPtr input_cloud = std::get<PointCloudPtr>(TestFixture::clouds);

  pcl::experimental::VoxelGrid<PointT> new_grid;
  pcl::VoxelGrid<PointT> old_grid;
  new_grid.setLeafSize(0.05f, 0.05f, 0.05f);
  old_grid.setLeafSize(0.05f, 0.05f, 0.05f);
  new_grid.setInputCloud(input_cloud);
  old_grid.setInputCloud(input_cloud);

  new_grid.setDownsampleAllData(false);
  old_grid.setDownsampleAllData(false);
  new_grid.filter(new_out);
  old_grid.filter(old_out);
  EXPECT_POINTS_EQ(new_out, old_out);
  new_out.clear();
  old_out.clear();

  new_grid.setDownsampleAllData(true);
  old_grid.setDownsampleAllData(true);
  new_grid.filter(new_out);
  old_grid.filter(old_out);
  EXPECT_POINTS_EQ(new_out, old_out);
  new_out.clear();
  old_out.clear();

  new_grid.setMinimumPointsNumberPerVoxel(5);
  old_grid.setMinimumPointsNumberPerVoxel(5);
  new_grid.filter(new_out);
  old_grid.filter(old_out);
  EXPECT_POINTS_EQ(new_out, old_out);
}

int
main(int argc, char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 4) {
    std::cerr << "No test file given. Please download `bun0.pcd`, "
                 "`milk_cartoon_all_small_clorox.pcd` and `milk_color.pcd`, and pass "
                 "their paths to the test."
              << std::endl;
    return (-1);
  }

  // Input
  if (loadPCDFile(argv[1], *cloud) < 0) {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its "
                 "path to the test."
              << std::endl;
    return (-1);
  }
  if (loadPCDFile(argv[2], *cloud_rgb) < 0) {
    std::cerr << "Failed to read test file. Please download "
                 "`milk_cartoon_all_small_clorox.pcd` and pass its path to the test."
              << std::endl;
    return (-1);
  }
  if (loadPCDFile(argv[3], *cloud_rgba) < 0) {
    std::cerr << "Failed to read test file. Please download `milk_color.pcd` and pass "
                 "its path to the test."
              << std::endl;
    return (-1);
  }

  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
