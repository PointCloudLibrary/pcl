/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/trait/common_traits.hpp>
#include <experimental/type_traits>
#include <type_traits>

namespace executor {

template <class Executor, typename = void>
struct is_executor : std::false_type {};

namespace detail {
const auto noop = [] {};

template <typename Executor, typename = void>
struct contains_execute : std::false_type {};

template <typename Executor>
struct contains_execute<
    Executor,
    std::enable_if_t<std::is_same<
        decltype(std::declval<Executor>().execute(detail::noop)), void>::value>>
    : std::true_type {};

}  // namespace detail

// Part of Proposal P0443R10: Removed from R11 in favour on concepts
template <typename Executor>
struct is_executor<
    Executor,
    std::enable_if_t<std::is_copy_constructible<Executor>::value &&
                     detail::contains_execute<Executor>::value &&
                     executor::equality_comparable<Executor, Executor>::value>>
    : std::true_type {};

template <typename Executor>
constexpr bool is_executor_v = is_executor<Executor>::value;

}  // namespace executor
