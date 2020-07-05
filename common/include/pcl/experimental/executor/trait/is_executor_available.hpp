/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

namespace pcl {
namespace executor {

/**
 * \brief Checks whether the given executor class template is for available use
 *
 * \details The availability of an executor depends on the hardware/software
 * capability of a given system. So we need a mechanism to prevent
 * use of an executor if the system doesn't support it.
 * In PCL you can define instances of an executor type without any issues
 * even if that type is not supported by the system but you cannot call any of
 * the execution functions (i.e. \p execute & \p bulk_execute) of that executor.
 * Any attempt to call the execution functions in such a scenario leads to a
 * compile time error.
 *
 * For an executor to be marked as available it must provide an explicit
 * specialization for this trait and inherit from std::true_type
 *
 * Example:
 * \code{.cpp}
 *
 * // Mark the OpenMP executor as available
 * #ifdef _OPENMP
 *  template <> struct is_executor_available<omp_executor> : std::true_type {};
 * #endif
 *
 * // To query if the executor class template is available
 * is_executor_available<omp_executor>::value
 * \endcode
 *
 * This is not part of the executor proposal and is needed for PCL, since the
 * availability of executors offered by PCL depends on the users system
 * configuration.
 *
 * \tparam Executor an executor class template
 */
template <template <typename...> class Executor>
struct is_executor_available : std::false_type {};

template <template <typename...> class Executor>
static constexpr bool is_executor_available_v = is_executor_available<Executor>::value;

/**
 * \brief Checks whether the given executor is available to use
 *
 * \details For details on availability of executors refer to
 * \ref is_executor_available
 *
 * This is not part of the executor proposal
 *
 * Example:
 * \code{.cpp}
 * // To query if the executor is available
 * is_executor_instance_available<inline_executor<>>::value
 * \endcode
 *
 * \tparam Executor an executor type
 */
template <typename Executor>
struct is_executor_instance_available : std::false_type {};

template <template <typename...> class Executor, typename... Properties>
struct is_executor_instance_available<Executor<Properties...>>
: is_executor_available<Executor> {};

template <template <typename...> class Executor>
constexpr bool is_executor_instance_available_v =
    is_executor_available<Executor>::value;

} // namespace executor
} // namespace pcl
