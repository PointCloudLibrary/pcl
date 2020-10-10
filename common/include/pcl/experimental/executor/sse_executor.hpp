/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/property.h>
#include <pcl/experimental/executor/type_trait.h>
#include <pcl/types.h>

namespace pcl {
namespace executor {

template <typename Blocking, typename ProtoAllocator>
struct sse_executor;

#ifdef __SSE__
template <>
struct is_executor_available<sse_executor> : std::true_type {};
#endif

/**
 * \brief Enforces vectorization of the code
 *
 * \todo
 * 1. Remove Blocking property since it cannot offer non-blocking behaviour
 * 2. Figure out how to vectorize the callable. E.g. using #pragma simd
 */
template <typename Blocking = blocking_t::always_t,
          typename ProtoAllocator = std::allocator<void>>
struct sse_executor {
  using shape_type = uindex_t;
  using index_type = uindex_t;

  template <typename Executor, InstanceOf<Executor, executor::sse_executor> = 0>
  friend constexpr bool
  operator==(const sse_executor&, const Executor&) noexcept
  {
    return std::is_same<sse_executor, Executor>::value;
  }

  template <typename Executor, InstanceOf<Executor, executor::sse_executor> = 0>
  friend constexpr bool
  operator!=(const sse_executor& lhs, const Executor& rhs) noexcept
  {
    return !operator==(lhs, rhs);
  }

  /**
   * \brief Launches a single execution agent which invokes the callable
   *
   * \param f a callable
   */
  template <typename Function>
  void
  execute(Function&& f) const
  {
    static_assert(is_executor_available_v<sse_executor>, "SSE executor unavailable");
    f();
  }

  /**
   * \brief Eagerly launches execution agents in bulk and which invoke
   * the callable with the associated agent's index.
   *
   * \param f a callable
   * \param shape number of execution agents to be launched
   */
  template <typename Function>
  void
  bulk_execute(Function&& f, const shape_type& shape) const
  {
    static_assert(is_executor_available_v<sse_executor>, "SSE executor unavailable");
    for (index_type idx = 0; idx < shape; ++idx)
      f(idx);
  }

  static constexpr decltype(auto)
  query(const blocking_t&) noexcept
  {
    return blocking_t::always;
  }

  sse_executor<blocking_t::always_t, ProtoAllocator>
  require(const blocking_t::always_t&) const
  {
    return {};
  }
};

using default_sse_executor = sse_executor<>;

} // namespace executor
} // namespace pcl
