/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <pcl/experimental/executor/executor.h>
#include <pcl/test/gtest.h>

#include <Eigen/Dense>

using namespace pcl;
using namespace executor;

using AvailableExecutorsTuple =
    typename pcl::filter_tuple_values<is_executor_instance_available,
                                      std::tuple<const default_inline_executor,
                                                 default_inline_executor,
                                                 const default_sse_executor,
                                                 default_sse_executor,
                                                 const default_omp_executor,
                                                 default_omp_executor>>::type;

template <typename T>
struct TestExecutorTypes {};

template <typename... Executors>
struct TestExecutorTypes<std::tuple<Executors...>> {
  using type = ::testing::Types<Executors...>;
};

using TestAvailableExecutorTypes =
    typename TestExecutorTypes<AvailableExecutorsTuple>::type;


template <typename Executor>
class ExecutorValidity : public ::testing::Test {};

TYPED_TEST_SUITE(ExecutorValidity, TestAvailableExecutorTypes);

TYPED_TEST(ExecutorValidity, executors)
{
  EXPECT_TRUE(is_executor_v<TypeParam>);
  EXPECT_FALSE(is_executor_v<int>);
}

template <typename Executor>
class ExecutorPropertyTraits : public ::testing::Test {};

TYPED_TEST_SUITE(ExecutorPropertyTraits, TestAvailableExecutorTypes);

TYPED_TEST(ExecutorPropertyTraits, executors)
{
  EXPECT_TRUE((can_require_v<TypeParam, blocking_t::always_t>));
  EXPECT_FALSE((can_require_v<TypeParam, blocking_t::never_t>));

  EXPECT_TRUE((can_prefer_v<TypeParam, blocking_t::always_t>));
  EXPECT_TRUE((can_prefer_v<TypeParam, blocking_t::never_t>));

  EXPECT_TRUE((can_query_v<TypeParam, blocking_t::always_t>));
  EXPECT_TRUE((can_query_v<TypeParam, blocking_t::never_t>));
}

template <typename Executor>
class ExecutorProperties : public ::testing::Test {};

TYPED_TEST_SUITE(ExecutorProperties, TestAvailableExecutorTypes);

TYPED_TEST(ExecutorProperties, executors)
{
  TypeParam exec;

  EXPECT_EQ(exec.query(blocking), blocking_t::always);
  const auto new_exec1 = exec.require(blocking_t::always);
  EXPECT_EQ(new_exec1.query(blocking), blocking_t::always);

  EXPECT_EQ(query(exec, blocking_t::always), blocking_t::always);
  const auto new_exec2 = require(exec, blocking_t::always);
  EXPECT_EQ(query(new_exec2, blocking), blocking_t::always);
}

template <typename Executor>
class ExecutorExecute : public ::testing::Test {};

TYPED_TEST_SUITE(ExecutorExecute, TestAvailableExecutorTypes);

TYPED_TEST(ExecutorExecute, executors)
{
  TypeParam exec;
  const int a = 1, b = 2;

  int c = 0;
  exec.execute([&]() { c = a + b; });
  EXPECT_EQ(c, 3);

  std::array<int, 3> c_vec = {0};
  exec.bulk_execute([&c_vec](auto index) { c_vec[index] = 1; }, 3);
  EXPECT_EQ(c_vec[0] + c_vec[1] + c_vec[2], c_vec.size());
}

template <typename Executor>
class ExecutorInstanceOfAny : public ::testing::Test {
protected:
  template <typename Blocking = executor::blocking_t::always_t>
  struct derived_inline : executor::inline_executor<Blocking> {};
};

TYPED_TEST_SUITE(ExecutorInstanceOfAny, TestAvailableExecutorTypes);

TYPED_TEST(ExecutorInstanceOfAny, executors)
{
  using DerivedExecutor = typename TestFixture::template derived_inline<>;

  EXPECT_TRUE((is_instance_of_v<default_inline_executor, inline_executor>));
  EXPECT_FALSE((is_instance_of_v<default_inline_executor, omp_executor>));

  EXPECT_TRUE(
      (is_instance_of_any_v<default_inline_executor, inline_executor, omp_executor>));
  EXPECT_FALSE(
      (is_instance_of_any_v<default_inline_executor, sse_executor, omp_executor>));

  EXPECT_TRUE((is_instance_of_any_v<DerivedExecutor, inline_executor, omp_executor>));
  EXPECT_FALSE((is_instance_of_any_v<DerivedExecutor, sse_executor, omp_executor>));
}

TEST(ExecutorRuntimeChecks, ExecutorEnviornmentVariableCheck)
{
  auto env_check = [&](auto exec, const char* env_name) {

#ifdef _WIN32
    _putenv((std::string(env_name) + "=OFF").c_str());
#else
    EXPECT_EQ(setenv(env_name, "OFF", true), 0);
#endif
    EXPECT_FALSE(executor_runtime_checks::check(exec));

#ifdef _WIN32
    _putenv((std::string(env_name) + "=ON").c_str());
#else
    EXPECT_EQ(setenv(env_name, "ON", true), 0);
#endif
    EXPECT_TRUE(executor_runtime_checks::check(exec));
  };

  env_check(default_sse_executor{}, "PCL_ENABLE_SSE_EXEC");
  env_check(default_omp_executor{}, "PCL_ENABLE_OMP_EXEC");
}

class MatrixMultiplication {
protected:
  template <
      typename ExecutorT,
      typename executor::InstanceOfAny<ExecutorT, inline_executor, sse_executor> = 0>
  void
  mmul(const ExecutorT ex,
       const Eigen::MatrixXd& a,
       const Eigen::MatrixXd& b,
       Eigen::MatrixXd& c)
  {
    auto mul = [&]() { c = a * b; };
    ex.execute(mul);
  }

  template <typename ExecutorT,
      typename executor::InstanceOf<ExecutorT, omp_executor> = 0>
  void
  mmul(const ExecutorT ex,
       const Eigen::MatrixXd& a,
       const Eigen::MatrixXd& b,
       Eigen::MatrixXd& c)
  {
    auto mul = [&](auto index) {
        for (int j = 0; j < b.cols(); ++j) {
          c(index, j) = 0.0;
          for (int k = 0; k < a.cols(); ++k) {
            c(index, j) += a(index, k) * b(k, j);
          }
        }
    };
    ex.bulk_execute(mul, a.rows());
  }

  void
  mmul(const Eigen::MatrixXd& a,
       const Eigen::MatrixXd& b,
       Eigen::MatrixXd& c,
       bool custom_priority)
  {
    auto mul = [&](auto& exec) { mmul(exec, a, b, c); };

    auto supported_executors =
        std::conditional_t<is_executor_available_v<omp_executor>,
            std::tuple<default_omp_executor, default_inline_executor>,
            std::tuple<default_inline_executor>>();

    if (custom_priority)
      enable_exec_with_priority(mul, supported_executors);
    else
      enable_exec_on_desc_priority(mul, supported_executors);
  }
};

template <typename Executor>
class ExecutorMatrixMultiplication : public ::testing::Test,
                                     public MatrixMultiplication {};

TYPED_TEST_SUITE(ExecutorMatrixMultiplication, TestAvailableExecutorTypes);

TYPED_TEST(ExecutorMatrixMultiplication, executors)
{
  TypeParam exec;

  double dataA[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9}, dataB[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9},
      dataC[9] = {0};
  Eigen::MatrixXd a = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dataA);
  Eigen::MatrixXd b = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dataB);
  Eigen::MatrixXd c = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dataC);

  Eigen::MatrixXd ans(3, 3);
  ans << 30, 36, 42, 66, 81, 96, 102, 126, 150;

  c.setZero();
  this->mmul(exec, a, b, c);
  EXPECT_TRUE(c.isApprox(ans));
}

class ExecutorBestFitMatrixMultiplication : public ::testing::Test,
                                            public MatrixMultiplication {};

TEST_F(ExecutorBestFitMatrixMultiplication, executors)
{
  double dataA[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9}, dataB[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9},
         dataC[9] = {0};
  Eigen::MatrixXd a = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dataA);
  Eigen::MatrixXd b = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dataB);
  Eigen::MatrixXd c = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dataC);

  const auto ans = (Eigen::Matrix3d() << 30, 36, 42, 66, 81, 96, 102, 126, 150).finished();

  c.setZero();
  this->mmul(a, b, c, true);
  EXPECT_TRUE(c.isApprox(ans));

  c.setZero();
  this->mmul(a, b, c, false);
  EXPECT_TRUE(c.isApprox(ans));
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
