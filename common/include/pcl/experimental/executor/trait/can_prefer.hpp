/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/trait/can_require.hpp>
#include <pcl/experimental/executor/trait/common_traits.hpp>
#include <type_traits>

namespace executor {

template <typename Executor, typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property_v<Executor> &&
                  Property::is_preferable && can_require_v<Executor, Property>,
              int> = 0>
constexpr auto prefer(Executor&& ex, const Property& p) noexcept {
  return ex.require(p);
}

template <typename Executor, typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property_v<Executor> &&
                  Property::is_preferable && !can_require_v<Executor, Property>,
              int> = 0>
constexpr auto prefer(Executor&& ex, const Property& p) noexcept {
  return std::forward<Executor>(ex);
}

// Part of Proposal P1393R0
template <typename Executor, typename Properties, typename = void>
struct can_prefer : std::false_type {};

template <typename Executor, typename Property>
struct can_prefer<Executor, Property,
                  void_t<decltype(prefer(
                      std::declval<executor::remove_cv_ref_t<Executor>>(),
                      std::declval<executor::remove_cv_ref_t<Property>>()))>>
    : std::true_type {};

template <typename Executor, typename Property>
constexpr bool can_prefer_v = can_prefer<Executor, Property>::value;

}  // namespace executor
