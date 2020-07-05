/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/default/base_executor.hpp>
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
    Executor, std::enable_if_t<std::is_same<
                  decltype(std::declval<executor::remove_cv_ref_t<Executor>>()
                               .execute(detail::noop)),
                  void>::value>> : std::true_type {};

template <typename Executor>
using check_equality_comparable =
    decltype(std::declval<Executor>() == std::declval<Executor>());

}  // namespace detail

// Part of Proposal P0443R10: Removed from R11 in favour on concepts
template <typename Executor>
struct is_executor<
    Executor,
    std::enable_if_t<std::is_copy_constructible<Executor>::value &&
                         detail::contains_execute<Executor>::value,
                     void_t<detail::check_equality_comparable<Executor>>>>
    : std::true_type {};

template <typename Executor>
constexpr bool is_executor_v = is_executor<Executor>::value;

}  // namespace executor
