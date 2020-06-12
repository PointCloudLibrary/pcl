/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2020-, Open Perception
 *
 * All rights reserved
 */

#include <pcl/common/generate.h>
#include <pcl/filters/lambda_filter_indices.h>
#include <pcl/test/gtest.h>
#include <pcl/point_types.h>

#include <random>

using namespace pcl;

TEST(LambdaFilter, CheckCompatibility)
{
  const auto copy_all = [](PointCloud<PointXYZ>, index_t) { return 0; };
  EXPECT_TRUE((is_lambda_filter_functor_v<PointXYZ, decltype(copy_all)>));

  const auto ref_all = [](PointCloud<PointXYZ>&, index_t&) { return 0; };
  EXPECT_FALSE((is_lambda_filter_functor_v<PointXYZ, decltype(ref_all)>));

  const auto ref_cloud = [](PointCloud<PointXYZ>&, index_t) { return 0; };
  EXPECT_FALSE((is_lambda_filter_functor_v<PointXYZ, decltype(ref_cloud)>));

  const auto const_ref_cloud = [](const PointCloud<PointXYZ>&, index_t) { return 0; };
  EXPECT_TRUE((is_lambda_filter_functor_v<PointXYZ, decltype(const_ref_cloud)>));

  const auto const_ref_all = [](const PointCloud<PointXYZ>&, const index_t&) {
    return 0;
  };
  EXPECT_TRUE((is_lambda_filter_functor_v<PointXYZ, decltype(const_ref_all)>));
}

TEST(LambdaFilter, FilterTest)
{
  auto cloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto out_cloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::uint32_t seed = 123;
  common::CloudGenerator<PointXYZ, common::UniformGenerator<float>> generator{
      {-10., 10., seed}};
  generator.fill(480, 640, *cloud);

  const auto lambda = [](const PointCloud<PointXYZ>& cloud, index_t idx) {
    const auto& pt = cloud[idx];
    return (pt.getArray3fMap() < 5).all() && (pt.getArray3fMap() > -5).all();
  };

  pcl::LambdaFilter<PointXYZ, decltype(lambda)> filter{lambda};
  filter.setInputCloud(cloud);

  filter.setNegative(false);
  filter.filter(*out_cloud);

  // expect 1/8 the size due to uniform generator
  EXPECT_GT(cloud->size(), (out_cloud->size() * 0.8) * 8);
  EXPECT_LT(cloud->size(), (out_cloud->size() * 1.2) * 8);

  filter.setNegative(true);
  filter.filter(*out_cloud);

  // expect 7/8 the size due to uniform generator
  EXPECT_GT(cloud->size(), (out_cloud->size() * 0.8) * 8 / 7);
  EXPECT_LT(cloud->size(), (out_cloud->size() * 1.2) * 8 / 7);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
