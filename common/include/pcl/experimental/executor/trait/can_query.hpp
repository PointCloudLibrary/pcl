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

// Part of Proposal P1393R0
template <typename Executor, typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property_v<Executor>, int> = 0>
constexpr auto query(Executor&& ex, const Property& p) noexcept {
  return Property::template static_query_v<Executor>;
}

}  // namespace executor
