/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2020-, Open Perception
 *
 * All rights reserved
 */

#include <pcl/common/generate.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/test/gtest.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace pcl::experimental;

TEST(FunctorFilterTrait, CheckCompatibility)
{
  const auto copy_all = [](PointCloud<PointXYZ>, index_t) { return 0; };
  EXPECT_TRUE((is_function_object_for_filter_v<PointXYZ, decltype(copy_all)>));

  const auto ref_all = [](PointCloud<PointXYZ>&, index_t&) { return 0; };
  EXPECT_FALSE((is_function_object_for_filter_v<PointXYZ, decltype(ref_all)>));

  const auto ref_cloud = [](PointCloud<PointXYZ>&, index_t) { return 0; };
  EXPECT_FALSE((is_function_object_for_filter_v<PointXYZ, decltype(ref_cloud)>));

  const auto const_ref_cloud = [](const PointCloud<PointXYZ>&, index_t) { return 0; };
  EXPECT_TRUE((is_function_object_for_filter_v<PointXYZ, decltype(const_ref_cloud)>));

  const auto const_ref_all = [](const PointCloud<PointXYZ>&, const index_t&) {
    return 0;
  };
  EXPECT_TRUE((is_function_object_for_filter_v<PointXYZ, decltype(const_ref_all)>));
}

struct FunctorFilterRandom : public testing::TestWithParam<std::uint32_t> {
  void
  SetUp() override
  {
    cloud = make_shared<PointCloud<PointXYZ>>();

    std::uint32_t seed = GetParam();
    common::CloudGenerator<PointXYZ, common::UniformGenerator<float>> generator{
        {-10., 0., seed}};
    generator.fill(20, 20, negative_cloud);
    generator.setParameters({0., 10., seed});
    generator.fill(10, 10, positive_cloud);
    *cloud = negative_cloud + positive_cloud;
  }

  shared_ptr<PointCloud<PointXYZ>> cloud;
  PointCloud<PointXYZ> out_cloud, negative_cloud, positive_cloud;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_P(FunctorFilterRandom, functioning)
{

  const auto lambda = [](const PointCloud<PointXYZ>& cloud, index_t idx) {
    const auto& pt = cloud[idx];
    return (pt.getArray3fMap() < 0).all();
  };

  for (const auto& keep_removed : {true, false}) {
    advanced::FunctorFilter<PointXYZ, decltype(lambda)> filter{lambda, keep_removed};
    filter.setInputCloud(cloud);
    const auto removed_size = filter.getRemovedIndices()->size();

    filter.setNegative(false);
    filter.filter(out_cloud);

    EXPECT_EQ(out_cloud.size(), negative_cloud.size());
    if (keep_removed) {
      EXPECT_EQ(filter.getRemovedIndices()->size(), positive_cloud.size());
    }
    else {
      EXPECT_EQ(filter.getRemovedIndices()->size(), removed_size);
    }

    filter.setNegative(true);
    filter.filter(out_cloud);

    EXPECT_EQ(out_cloud.size(), positive_cloud.size());
    if (keep_removed) {
      EXPECT_EQ(filter.getRemovedIndices()->size(), negative_cloud.size());
    }
    else {
      EXPECT_EQ(filter.getRemovedIndices()->size(), removed_size);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(RandomSeed,
                         FunctorFilterRandom,
                         testing::Values(123, 456, 789));

namespace type_test {
int
free_func(const PointCloud<PointXYZ>&, const index_t& idx)
{
  return idx % 2;
}

static const auto lambda_func = [](const PointCloud<PointXYZ>& cloud, index_t idx) {
  return free_func(cloud, idx);
};

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
struct StaticFunctorConst {
  int
  operator()(PointCloud<PointXYZ> cloud, index_t idx) const
  {
    return free_func(cloud, idx);
  }
};

using LambdaT = decltype(lambda_func);
using StdFunctorBoolT = std::function<bool(PointCloud<PointXYZ>, index_t)>;
using StdFunctorIntT = std::function<int(PointCloud<PointXYZ>, index_t)>;
using FreeFuncT = decltype(free_func)*;
using StaticFunctorT = decltype(StaticFunctor::functor)*;
using NonConstFuntorT = StaticFunctor;
using ConstFuntorT = StaticFunctorConst;

template <typename FunctorT>
struct Helper {};

#define HELPER_MACRO(TYPE, VALUE)                                                      \
  template <>                                                                          \
  struct Helper<TYPE> {                                                                \
    using type = TYPE;                                                                 \
    static type value;                                                                 \
  };                                                                                   \
  TYPE Helper<TYPE>::value = VALUE

HELPER_MACRO(LambdaT, lambda_func);
HELPER_MACRO(StdFunctorBoolT, lambda_func);
HELPER_MACRO(StdFunctorIntT, lambda_func);
HELPER_MACRO(FreeFuncT, free_func);
HELPER_MACRO(StaticFunctorT, StaticFunctor::functor);
HELPER_MACRO(NonConstFuntorT, {});
HELPER_MACRO(ConstFuntorT, {});

#undef HELPER_MACRO

using types = ::testing::Types<LambdaT,
                               StdFunctorBoolT,
                               StdFunctorIntT,
                               FreeFuncT,
                               StaticFunctorT,
                               NonConstFuntorT,
                               ConstFuntorT>;
} // namespace type_test

template <typename T>
struct FunctorFilterFunctionObject : public ::testing::Test {
  void
  SetUp() override
  {
    cloud.resize(2);
  }
  PointCloud<PointXYZ> cloud;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
TYPED_TEST_SUITE_P(FunctorFilterFunctionObject);

TYPED_TEST_P(FunctorFilterFunctionObject, type_check)
{
  using FunctorT = TypeParam;
  const auto& functor = type_test::Helper<FunctorT>::value;

  advanced::FunctorFilter<PointXYZ, FunctorT> filter_lambda{functor};
  EXPECT_EQ(filter_lambda.getFunctionObject()(this->cloud, 0), 0);
  EXPECT_EQ(filter_lambda.getFunctionObject()(this->cloud, 1), 1);
}

REGISTER_TYPED_TEST_SUITE_P(FunctorFilterFunctionObject, type_check);
INSTANTIATE_TYPED_TEST_SUITE_P(pcl, FunctorFilterFunctionObject, type_test::types);

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
