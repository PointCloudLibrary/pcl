/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

namespace executor {

template <typename Executor, typename = void>
struct executor_index {
  using type = std::size_t;
};  // namespace executor

template <typename Executor>
struct executor_index<Executor,
                      executor::void_t<typename decltype(
                          std::declval<const Executor>())::shape_type>> {
  using type = typename decltype(std::declval<const Executor>())::index_type;
};

template <class Executor>
using executor_index_t = typename executor_index<Executor>::type;

}  // namespace executor
