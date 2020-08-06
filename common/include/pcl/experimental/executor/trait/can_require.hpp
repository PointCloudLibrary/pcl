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
#include <type_traits>

namespace executor {

namespace detail {
template <typename Executor, typename Property>
using contains_property = std::is_same<
    std::remove_const_t<decltype(Property::template static_query_v<Executor>)>,
    Property>;
}

template <typename Executor, typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property_v<Executor> &&
                  Property::is_requirable &&
                  detail::contains_property<Executor, Property>::value,
              int> = 0>
constexpr decltype(auto) require(const Executor& ex,
                                 const Property& p) noexcept {
  return ex.require(p);
}

// Part of Proposal P1393R0
template <typename Executor, typename Properties, typename = void>
struct can_require : std::false_type {};

template <typename Executor, typename Property>
struct can_require<Executor, Property,
                   void_t<decltype(require(std::declval<Executor>(),
                                           std::declval<Property>()))>>
    : std::true_type {};

template <typename Executor, typename Property>
constexpr bool can_require_v = can_require<Executor, Property>::value;

}  // namespace executor
