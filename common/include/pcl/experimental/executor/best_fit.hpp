/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/common/tuple.h>
#include <pcl/experimental/executor/cuda_executor.hpp>
#include <pcl/experimental/executor/inline_executor.hpp>
#include <pcl/experimental/executor/omp_executor.hpp>
#include <pcl/experimental/executor/sse_executor.hpp>
#include <pcl/experimental/executor/type_trait.h>

#include <boost/algorithm/string.hpp>

#include <iostream>
#include <thread>

namespace pcl {
namespace executor {

/**
 * \brief One of the following executors is used when a user doesn't specify an
 * executor explicitly in an implementation supporting executors.
 *
 * It is used internally inside \ref enable_exec_on_desc_priority
 */
static const auto best_fit_executors = std::make_tuple(executor::omp_executor<>{},
                                                       executor::sse_executor<>{},
                                                       executor::inline_executor<>{});

struct executor_runtime_checks {
  template <typename Executor,
            typename executor::InstanceOf<Executor, executor::inline_executor> = 0>
  static bool
  check(Executor&)
  {
    return true;
  }

  template <typename Executor,
            typename executor::InstanceOf<Executor, executor::sse_executor> = 0>
  static bool
  check(Executor&)
  {
    if (const char* env_p = std::getenv("PCL_ENABLE_SSE_EXEC"))
      if (boost::iequals(env_p, "OFF"))
        return false;
    return true;
  }

  template <typename Executor,
            typename executor::InstanceOf<Executor, executor::omp_executor> = 0>
  static bool
  check(Executor& exec)
  {
    if (const char* env_p = std::getenv("PCL_ENABLE_OMP_EXEC"))
      if (boost::iequals(env_p, "OFF"))
        return false;

    // If hardware_concurrency() fails to get the number of threads than set max
    // threads to 2 as a fallback to prevent unwanted scaling in machines with
    // large number of available threads
    auto max_threads = std::max(std::min(std::thread::hardware_concurrency(), 8u), 2u);
    exec.set_max_threads(max_threads);

    return true;
  }

  template <typename Executor,
            typename executor::InstanceOf<Executor, executor::cuda_executor> = 0>
  static bool
  check(Executor&)
  {
    if (const char* env_p = std::getenv("PCL_ENABLE_CUDA_EXEC"))
      if (boost::iequals(env_p, "OFF"))
        return false;
    return true;
  }
};

namespace detail {

#ifdef DOXYGEN_ONLY
/**
 * \brief A helper function to invoke the callable with the specified executor
 * if that the executor is available (See \ref is_executor_available)
 *
 * \param Function a callable which is invoked with the specified executor
 * \param Executor an executor
 * \return a boolean indicating whether the callable was invoked or not
 */
template <typename Function, typename Executor>
bool
execute(Function&& f, Executor& exec);
#else
template <typename Function, typename Executor, typename = void>
bool
execute(Function&&, Executor&)
{
  return false;
}

// clang-format off
template <
    typename Function,
    template <typename...> class Executor,
    typename... Properties,
    typename std::enable_if<executor::is_executor_available_v<Executor>, int>::type = 0>
// clang-format on
bool
execute(Function&& f, Executor<Properties...>& exec)
{
  f(exec);
  return true;
}
#endif

template <typename Supported>
struct executor_predicate {
  template <typename T, typename = void>
  struct condition : std::false_type {};

  template <typename T>
  struct condition<T,
                   std::enable_if_t<is_executor_instance_available<T>::value &&
                                    pcl::tuple_contains_type_v<T, Supported>>>
  : std::true_type {};
};

} // namespace detail

/**
 * \brief Selects an executor from the specified list of supported executors
 * based on the provided runtime checks and priority and then invokes the
 * callable with the selected executor as the argument.
 *
 * \details This function provides a mechanism for the best fitting
 * executor to execute the specified callable to get selected.
 *
 * The supported executors which are specified serve two purposes, one
 * being to provide a list of executor types which have valid overloads for
 * the specified callable, the second being that the order in which the
 * executors are specified indicates the priority (descending order of priority)
 * of each executor.
 *
 * For each supported executor a set of runtime checks are performed one by one
 * (highest to lowest priority) until one of the checks passes, and that executor
 * is selected.
 * Custom runtime checks can be created by deriving from the class
 * \ref executor_runtime_checks and can be passed as a template parameter
 * to this function.
 *
 * Finally, the selected executor is passed as an argument to the callable and
 * the callable is invoked.
 *
 * \tparam RuntimeChecks runtime checks to be performed on each executor
 * \param Function a callable which is invoked with the selected executor
 * \param std::tuple<SupportedExecutors...> a tuple of supported executor types
 * arranged in descending order of priority
 */
template <typename RuntimeChecks = executor_runtime_checks,
          typename Function,
          typename... SupportedExecutors>
void
enable_exec_with_priority(Function&& f,
                          std::tuple<SupportedExecutors...> supported_execs)
{
  static_assert(std::is_base_of<executor_runtime_checks, RuntimeChecks>::value,
                "Runtime checks should inherit from executor_runtime_checks");
  bool executor_selected = false;
  pcl::for_each_until_true(supported_execs, [&](auto& exec) {
    if (RuntimeChecks::check(exec)) {
      executor_selected = detail::execute(f, exec);
      return executor_selected;
    }
    return false;
  });

  // This should not happen, at least one executor should always be selected. So
  // either all runtime checks failed which is incorrect behaviour or no
  // supported executors were passed which should not be done
  if (!executor_selected)
    std::cerr << "No executor selected. All runtime checks returned false or "
                 "no executors were passed."
              << std::endl;
}

/**
 * \brief Selects an executor from the specified list of supported executors
 * based on the provided runtime checks and priority and then invokes the
 * callable with the selected executor as the argument.
 *
 * \details This function provides a mechanism for the best fitting
 * executor to execute the specified callable to get selected.
 *
 * The supported executors which are specified serve two purposes, one
 * being to provide a list of executor types which have valid overloads for
 * the specified callable, the second being that the order in which the
 * executors are specified indicates the priority (descending order of priority)
 * of each executor.
 *
 * For each supported executor a set of runtime checks are performed one by one
 * (highest to lowest priority) until one of the checks passes, and that executor
 * is selected.
 * Custom runtime checks can be created by deriving from the class
 * \ref executor_runtime_checks and can be passed as a template parameter
 * to this function.
 *
 * Finally, the selected executor is passed as an argument to the callable and
 * the callable is invoked.
 *
 * \tparam RuntimeChecks runtime checks to be performed on each executor
 * \param Function a callable which is invoked with the selected executor
 * \param SupportedExecutors a parameter pack of supported executor types
 * arranged in descending order of priority
 */
template <typename RuntimeChecks = executor_runtime_checks,
          typename Function,
          typename... SupportedExecutors>
void
enable_exec_with_priority(Function&& f, SupportedExecutors&&... execs)
{
  enable_exec_with_priority<RuntimeChecks>(f, std::make_tuple(execs...));
}

/**
 * \brief Selects an executor from the specified list of supported executors
 * based on the provided runtime checks and then invokes the
 * callable with the selected executor as the argument.
 *
 * \details This function provides a mechanism for the best fitting
 * executor to execute the specified callable to get selected.
 *
 * The supported executors which are used to provide a list of executor
 * types which have valid overloads for the specified callable.
 *
 * The priority if the supported executor is set according to the order
 * of the executors in \ref best_fit_executors in descending order.
 *
 * For each supported executor a set of runtime checks are performed one by one
 * (highest to lowest priority) until one of the checks passes, and that executor
 * is selected.
 * Custom runtime checks can be created by deriving from the class
 * \ref executor_runtime_checks and can be passed as a template parameter
 * to this function.
 *
 * Finally, the selected executor is passed as an argument to the callable and
 * the callable is invoked.
 *
 * \tparam RuntimeChecks runtime checks to be performed on each executor
 * \param Function a callable which is invoked with the selected executor
 * \param std::tuple<SupportedExecutors...> a tuple of supported executor types
 */
template <typename RuntimeChecks = executor_runtime_checks,
          typename Function,
          typename... SupportedExecutors>
void
enable_exec_on_desc_priority(Function&& f,
                             std::tuple<SupportedExecutors...> supported_execs)
{
  static_assert(std::is_base_of<executor_runtime_checks, RuntimeChecks>::value,
                "Runtime checks should inherit from executor_runtime_checks");

  using predicate = detail::executor_predicate<decltype(supported_execs)>;
  pcl::filter_tuple_values<predicate::template condition, decltype(best_fit_executors)>
      filter_available;

  auto filtered = filter_available(best_fit_executors);

  enable_exec_with_priority(f, filtered);
}

/**
 * \brief Selects an executor from the specified list of supported executors
 * based on the provided runtime checks and then invokes the
 * callable with the selected executor as the argument.
 *
 * \details This function provides a mechanism for the best fitting
 * executor to execute the specified callable to get selected.
 *
 * The supported executors which are used to provide a list of executor
 * types which have valid overloads for the specified callable.
 *
 * The priority if the supported executor is set according to the order
 * of the executors in \ref best_fit_executors in descending order.
 *
 * For each supported executor a set of runtime checks are performed one by one
 * (highest to lowest priority) until one of the checks passes, and that executor
 * is selected.
 * Custom runtime checks can be created by deriving from the class
 * \ref executor_runtime_checks and can be passed as a template parameter
 * to this function.
 *
 * Finally, the selected executor is passed as an argument to the callable and
 * the callable is invoked.
 *
 * \tparam RuntimeChecks runtime checks to be performed on each executor
 * \param Function a callable which is invoked with the selected executor
 * \param SupportedExecutors a parameter pack of supported executor types
 */
template <typename RuntimeChecks = executor_runtime_checks,
          typename Function,
          typename... SupportedExecutors>
void
enable_exec_on_desc_priority(Function&& f, SupportedExecutors&&... supported_execs)
{
  enable_exec_on_desc_priority<RuntimeChecks>(f, std::make_tuple(supported_execs...));
}

} // namespace executor
} // namespace pcl
