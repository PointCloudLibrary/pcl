/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/trait/is_executor.hpp>
#include <functional>
#include <iostream>
#include <string>

namespace executor {

template <typename Derived, bool requireable, bool preferable>
struct basic_executor_property {
  // Part of Proposal P1393R0
  static constexpr bool is_requirable = requireable;
  static constexpr bool is_preferable = preferable;

  // Part of Proposal P1393R0
  template <class Executor>
  static constexpr bool is_applicable_property() {
    return executor::is_executor<Executor>();
  }

  // Part of Proposal P0443R13: 2.2.11 & 2.2.12
  template <class Executor>
  static constexpr auto static_query() {
    return executor::remove_cv_ref_t<Executor>::query(Derived());
  }

  template <typename T>
  static constexpr bool is_applicable_property_v = is_applicable_property<T>();

  // static constexpr Type static_query_v = static_query<Executor>() doesn't
  // work due to Clang complaining about `invalid operands to binary expression`
  template <class Executor,
            class Type = decltype(executor::remove_cv_ref_t<Executor>::query(
                *static_cast<Derived*>(0)))>
  static constexpr Type static_query_v =
      executor::remove_cv_ref_t<Executor>::query(Derived());
};

}  // namespace executor
