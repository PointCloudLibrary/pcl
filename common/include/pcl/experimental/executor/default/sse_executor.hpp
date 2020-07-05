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

namespace executor {

template <typename Blocking, typename ProtoAllocator>
struct sse_executor;

#ifdef __SSE__
template <>
struct is_executor_available<sse_executor> : std::true_type {};
#endif

template <typename Blocking = blocking_t::always_t,
          typename ProtoAllocator = std::allocator<void>>
struct sse_executor {
  using shape_type = std::size_t;

  template <typename Executor, instance_of_base<sse_executor, Executor> = 0>
  friend bool operator==(const sse_executor& lhs,
                         const Executor& rhs) noexcept {
    return std::is_same<sse_executor, Executor>::value;
  }

  template <typename Executor, instance_of_base<sse_executor, Executor> = 0>
  friend bool operator!=(const sse_executor& lhs,
                         const Executor& rhs) noexcept {
    return !operator==(lhs, rhs);
  }

  template <typename F>
  void execute(F&& f) const {
    std::forward<F>(f)();
  }

  template <typename F>
  void bulk_execute(F&& f, shape_type n) const {
#pragma simd
    for (std::size_t i = 0; i < n; ++i) {
      std::forward<F>(f)(i);
    }
  }

  static constexpr auto query(blocking_t) noexcept { return Blocking{}; }

  sse_executor<blocking_t::always_t, ProtoAllocator> require(
      const blocking_t::always_t&) const {
    return {};
  }

  static constexpr auto name() { return "sse"; }
};

}  // namespace executor
