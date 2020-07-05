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
 * \brief A trait to access the shape of the specified executor
 *
 * \details: An executor can have a custom shape type, so this trait helps
 * provide a uniform way of accessing the shape type of any executor.
 *
 * The shape represents the shape of execution unit which is currently
 * running. By default if not explicitly specified by the executor the shape is
 * std::size_t.
 *
 * Any executor can define a custom shape by defining the alias shape_type for
 * the custom shape.
 *
 * The type of the shape can be accessed using the member variable \p type
 *
 * Example:
 * \code{.cpp}
 * // To access the shape type of inline executor
 * executor_shape<inline_executor<>>::type
 * \endcode
 *
 * Part of proposal P0443R13
 *
 * \tparam an executor whose shape type you want to access
 */
template <typename Executor>
struct executor_shape;
#else
template <typename Executor, typename = void>
struct executor_shape {
  using type = std::size_t;
};

template <typename Executor>
struct executor_shape<Executor, pcl::void_t<typename Executor::shape_type>> {
  using type = typename Executor::shape_type;
};

template <class Executor>
using executor_shape_t = typename executor_shape<Executor>::type;
#endif
} // namespace executor
} // namespace pcl
