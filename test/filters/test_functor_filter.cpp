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

template <typename T>
struct FunctorFilterExecutor : public ::testing::Test {

  shared_ptr<PointCloud<PointXYZ>> cloud;
  shared_ptr<Indices> indices;
  PointCloud<PointXYZ> negative_cloud, positive_cloud;
  Indices out_indices;
  std::array<std::uint32_t, 3> seeds = {123, 456, 789};

  void
  SetUp() override
  {
    cloud = make_shared<PointCloud<PointXYZ>>();
  }

  void
  fillCloud(const std::uint32_t seed)
  {
    cloud->clear();
    negative_cloud.clear(); // suppress console warnings
    positive_cloud.clear(); // suppress console warnings

    common::CloudGenerator<PointXYZ, common::UniformGenerator<float>> generator{
        {-10., 0., seed}};
    generator.fill(20, 20, negative_cloud);
    generator.setParameters({0., 10., seed});
    generator.fill(10, 10, positive_cloud);
    *cloud = negative_cloud + positive_cloud;

    // Build indices selection, fetching the second half of the negative cloud
    // and the first half of the positive cloud
    indices = make_shared<Indices>();
    indices->reserve(cloud->size() / 2);
    for (index_t i = negative_cloud.size() / 2;
         i < static_cast<index_t>(negative_cloud.size() + positive_cloud.size() / 2);
         ++i) {
      indices->push_back(i);
    }
  }
  template <typename Executor,
            typename Filter,
            typename std::enable_if<std::is_same<Executor, void>::value, int>::type = 0>
  void
  executeFilter(Filter& filter, Indices& indices)
  {
    filter.filter(indices);
  }

  template <
      typename Executor,
      typename Filter,
      typename std::enable_if<!std::is_same<Executor, void>::value, int>::type = 0>
  void
  executeFilter(Filter& filter, Indices& indices)
  {
    filter.filter(Executor{}, indices);
  }

  template <typename Executor = void>
  void
  runTest()
  {
    const auto lambda = [](const PointCloud<PointXYZ>& cloud, index_t idx) {
      const auto& pt = cloud[idx];
      return (pt.getArray3fMap() < 0).all();
    };

    const auto compare_indices =
        [](const auto first, const auto last, const index_t start) {
          index_t i = start;
          for (auto it = first; it != last; ++it, ++i)
            EXPECT_EQ(*it, i);
        };

    for (const auto& keep_removed : {true, false}) {
      FunctorFilter<PointXYZ, decltype(lambda)> filter{lambda, keep_removed};
      filter.setInputCloud(this->cloud);
      filter.setIndices(this->indices);
      const auto removed_size = filter.getRemovedIndices()->size();

      filter.setNegative(false);
      executeFilter<Executor>(filter, this->out_indices);

      EXPECT_EQ(this->out_indices.size(), this->negative_cloud.size() / 2);
      compare_indices(this->out_indices.cbegin(),
                      this->out_indices.cend(),
                      this->negative_cloud.size() / 2);

      if (keep_removed) {
        const auto& removed_indices = filter.getRemovedIndices();
        EXPECT_EQ(removed_indices->size(), this->positive_cloud.size() / 2);
        compare_indices(removed_indices->cbegin(),
                        removed_indices->cend(),
                        this->negative_cloud.size());
      }
      else {
        EXPECT_EQ(filter.getRemovedIndices()->size(), removed_size);
      }

      filter.setNegative(true);
      executeFilter<Executor>(filter, this->out_indices);

      EXPECT_EQ(this->out_indices.size(), this->positive_cloud.size() / 2);
      compare_indices(this->out_indices.cbegin(),
                      this->out_indices.cend(),
                      this->negative_cloud.size());

      if (keep_removed) {
        const auto& removed_indices = filter.getRemovedIndices();
        EXPECT_EQ(removed_indices->size(), this->negative_cloud.size() / 2);
        compare_indices(removed_indices->cbegin(),
                        removed_indices->cend(),
                        this->negative_cloud.size() / 2);
      }
      else {
        EXPECT_EQ(filter.getRemovedIndices()->size(), removed_size);
      }
    }
  }
};

using ExecutorTypes = ::testing::Types<executor::inline_executor<>,
                                       executor::omp_executor<>,
                                       void>; // void type to run the filter without any
                                              // executor i.e. using best fit
TYPED_TEST_SUITE(FunctorFilterExecutor, ExecutorTypes);

TYPED_TEST(FunctorFilterExecutor, executors)
{
  for (auto seed : this->seeds) {
    this->fillCloud(seed);
    this->template runTest<TypeParam>();
  }
}

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
struct FunctorFilterFunctor : public ::testing::Test {
  void
  SetUp() override
  {
    cloud.resize(2);
  }
  PointCloud<PointXYZ> cloud;
};
TYPED_TEST_SUITE_P(FunctorFilterFunctor);

TYPED_TEST_P(FunctorFilterFunctor, type_check)
{
  using FunctorT = TypeParam;
  const auto& functor = type_test::Helper<FunctorT>::value;

  FunctorFilter<PointXYZ, FunctorT> filter_lambda{functor};
  EXPECT_EQ(filter_lambda.getFunctor()(this->cloud, 0), 0);
  EXPECT_EQ(filter_lambda.getFunctor()(this->cloud, 1), 1);
}

REGISTER_TYPED_TEST_SUITE_P(FunctorFilterFunctor, type_check);
INSTANTIATE_TYPED_TEST_SUITE_P(pcl, FunctorFilterFunctor, type_test::types);

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
