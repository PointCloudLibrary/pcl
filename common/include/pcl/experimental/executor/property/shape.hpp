/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/property/base_property.hpp>

namespace executor {

// Part of Proposal P0443R13: 2.3.1, Needs to be customized to add support for
// CUDA executors
template <class Executor>
struct executor_shape
    : basic_executor_property<executor_shape<Executor>, true, true> {
  template <unsigned _s0 = 0, unsigned... _sizes>
  using type = std::size_t;
};

template <class Executor>
using executor_shape_t = typename executor_shape<Executor>::type;

}  // namespace executor
