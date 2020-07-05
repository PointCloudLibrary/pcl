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

template <template <typename...> class Executor>
struct is_executor_available : std::false_type {};

template <template <typename...> class Executor>
constexpr bool is_executor_available_v = is_executor_available<Executor>::value;

}  // namespace executor
