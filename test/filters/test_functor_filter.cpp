/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2020-, Open Perception
 *
 * All rights reserved
 */

#include <pcl/common/generate.h>
#include <pcl/filters/functor_filter.h>
#include <pcl/test/gtest.h>
#include <pcl/point_types.h>

#include <random>

using namespace pcl;

TEST(FunctorFilterTrait, CheckCompatibility)
{
  const auto copy_all = [](PointCloud<PointXYZ>, index_t) { return 0; };
  EXPECT_TRUE((is_functor_for_filter_v<PointXYZ, decltype(copy_all)>));

  const auto ref_all = [](PointCloud<PointXYZ>&, index_t&) { return 0; };
  EXPECT_FALSE((is_functor_for_filter_v<PointXYZ, decltype(ref_all)>));

  const auto ref_cloud = [](PointCloud<PointXYZ>&, index_t) { return 0; };
  EXPECT_FALSE((is_functor_for_filter_v<PointXYZ, decltype(ref_cloud)>));

  const auto const_ref_cloud = [](const PointCloud<PointXYZ>&, index_t) { return 0; };
  EXPECT_TRUE((is_functor_for_filter_v<PointXYZ, decltype(const_ref_cloud)>));

  const auto const_ref_all = [](const PointCloud<PointXYZ>&, const index_t&) {
    return 0;
  };
  EXPECT_TRUE((is_functor_for_filter_v<PointXYZ, decltype(const_ref_all)>));
}

TEST(FunctorFilterTest, implementation)
{
  auto cloud = make_shared<PointCloud<PointXYZ>>();
  auto out_cloud = make_shared<PointCloud<PointXYZ>>();

  std::uint32_t seed = 123;
  common::CloudGenerator<PointXYZ, common::UniformGenerator<float>> generator{
      {-10., 10., seed}};
  generator.fill(480, 640, *cloud);

  const auto lambda = [](const PointCloud<PointXYZ>& cloud, index_t idx) {
    const auto& pt = cloud[idx];
    return (pt.getArray3fMap() < 5).all() && (pt.getArray3fMap() > -5).all();
  };

  for (const auto& keep_removed : {true, false}) {
    FunctorFilter<PointXYZ, decltype(lambda)> filter{lambda, keep_removed};
    filter.setInputCloud(cloud);
    const auto removed_size = filter.getRemovedIndices()->size();

    filter.setNegative(false);
    filter.filter(*out_cloud);

    // expect 1/8 the size due to uniform generator
    EXPECT_GT(cloud->size(), (out_cloud->size() * 0.8) * 8);
    EXPECT_LT(cloud->size(), (out_cloud->size() * 1.2) * 8);
    if (keep_removed) {
      EXPECT_EQ(filter.getRemovedIndices()->size() + out_cloud->size(), cloud->size());
    }
    else {
      EXPECT_EQ(filter.getRemovedIndices()->size(), removed_size);
    }

    filter.setNegative(true);
    filter.filter(*out_cloud);

    // expect 7/8 the size due to uniform generator
    EXPECT_GT(cloud->size(), (out_cloud->size() * 0.8) * 8 / 7);
    EXPECT_LT(cloud->size(), (out_cloud->size() * 1.2) * 8 / 7);
    if (keep_removed) {
      EXPECT_EQ(filter.getRemovedIndices()->size() + out_cloud->size(), cloud->size());
    }
    else {
      EXPECT_EQ(filter.getRemovedIndices()->size(), removed_size);
    }
  }
}

int
free_func(const PointCloud<PointXYZ>&, const index_t& idx)
{
  return idx % 2;
}

TEST(FunctorFilterTest, functor_types)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize(2);

  const auto lambda = [](const PointCloud<PointXYZ>& cloud, index_t idx) {
    return free_func(cloud, idx);
  };
  FunctorFilter<PointXYZ, decltype(lambda)> filter_lambda{lambda};
  EXPECT_EQ(filter_lambda.getFunctor()(cloud, 0), 0);
  EXPECT_EQ(filter_lambda.getFunctor()(cloud, 1), 1);

  std::function<bool(PointCloud<PointXYZ>, index_t)> func = lambda;
  FunctorFilter<PointXYZ, decltype(func)> filter_func{func};
  EXPECT_EQ(filter_func.getFunctor()(cloud, 0), 0);
  EXPECT_EQ(filter_func.getFunctor()(cloud, 1), 1);

  FunctorFilter<PointXYZ, decltype(free_func)*> filter_free_func{free_func};
  EXPECT_EQ(filter_free_func.getFunctor()(cloud, 0), 0);
  EXPECT_EQ(filter_free_func.getFunctor()(cloud, 1), 1);

  struct StaticFunctor {
    static int
    functor(PointCloud<PointXYZ> cloud, index_t idx)
    {
      return free_func(cloud, idx);
    }
    int
    operator()(PointCloud<PointXYZ> cloud, index_t idx)
    {
      return free_func(cloud, idx);
    }
  };
  FunctorFilter<PointXYZ, decltype(StaticFunctor::functor)*> filter_static_func{
      StaticFunctor::functor};
  EXPECT_EQ(filter_static_func.getFunctor()(cloud, 0), 0);
  EXPECT_EQ(filter_static_func.getFunctor()(cloud, 1), 1);

  FunctorFilter<PointXYZ, StaticFunctor> filter_class_func{{}};
  EXPECT_EQ(filter_class_func.getFunctor()(cloud, 0), 0);
  EXPECT_EQ(filter_class_func.getFunctor()(cloud, 1), 1);

  struct StaticFunctorConst {
    int
    operator()(PointCloud<PointXYZ> cloud, index_t idx) const
    {
      return free_func(cloud, idx);
    }
  };
  FunctorFilter<PointXYZ, StaticFunctorConst> filter_class_const_func{{}};
  EXPECT_EQ(filter_class_func.getFunctor()(cloud, 0), 0);
  EXPECT_EQ(filter_class_func.getFunctor()(cloud, 1), 1);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
