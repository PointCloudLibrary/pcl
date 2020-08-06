/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/common/utils.h>
#include <pcl/experimental/executor/property.h>
#include <pcl/experimental/executor/type_trait.h>

namespace executor {

template <typename Blocking, typename ProtoAllocator>
struct inline_executor;

template <>
struct is_executor_available<inline_executor> : std::true_type {};

template <typename Blocking = blocking_t::always_t,
          typename ProtoAllocator = std::allocator<void>>
struct inline_executor {
  using shape_type = std::size_t;

  template <typename Executor, instance_of_base<Executor, inline_executor> = 0>
  friend bool operator==(const inline_executor& lhs,
                         const Executor& rhs) noexcept {
    return std::is_same<inline_executor, Executor>::value;
  }

  template <typename Executor, instance_of_base<Executor, inline_executor> = 0>
  friend bool operator!=(const inline_executor& lhs,
                         const Executor& rhs) noexcept {
    return !operator==(lhs, rhs);
  }

  template <typename F>
  void execute(F&& f) const {
    f();
  }

  template <typename F, typename... Args>
  void bulk_execute(F&& f, const std::size_t n) const {
    pcl::utils::ignore(n);
    f(0);
  }

  static constexpr auto query(const blocking_t&) noexcept { return Blocking(); }

  inline_executor<blocking_t::always_t, ProtoAllocator> require(
      const blocking_t::always_t&) const {
    return {};
  }

  static constexpr auto name() { return "inline"; }
};

}  // namespace executor
