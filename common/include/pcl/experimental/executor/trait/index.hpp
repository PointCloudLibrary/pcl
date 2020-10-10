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

#ifdef DOXYGEN_ONLY
/**
 * \brief A trait to access the index of the specified executor
 *
 * \details: An executor can have a custom index type, so this trait helps
 * provide a uniform way of accessing the index type of any executor.
 *
 * The index represents the index of execution unit which is currently
 * running. By default if not explicitly specified by the executor the index is
 * std::size_t.
 *
 * Any executor can define a custom index by defining the alias index_type for
 * the custom index.
 *
 * The type of the index can be accessed using the member variable \p type
 *
 * Example:
 * \code{.cpp}
 * // To access the index type of inline executor
 * executor_index<inline_executor<>>::type
 * \endcode
 *
 * Part of proposal P0443R13
 *
 * \tparam an executor whose index type you want to access
 */
template <typename Executor>
struct executor_index;
#else
template <typename Executor, typename = void>
struct executor_index {
  using type = std::size_t;
}; // namespace executor
template <typename Executor>
struct executor_index<Executor, pcl::void_t<typename Executor::index_type>> {
  using type = typename Executor::index_type;
};

template <class Executor>
using executor_index_t = typename executor_index<Executor>::type;
#endif

} // namespace executor
} // namespace pcl
