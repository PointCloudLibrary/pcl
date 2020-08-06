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
struct executor_shape {
  using type = std::size_t;
};  // namespace executor

template <typename Executor>
struct executor_shape<Executor,
                      executor::void_t<typename Executor::shape_type>> {
  using type = typename Executor::shape_type;
};

template <class Executor>
using executor_shape_t = typename executor_shape<Executor>::type;

}  // namespace executor
