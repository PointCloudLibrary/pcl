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
struct inline_executor;

template <>
struct is_executor_available<inline_executor> : std::true_type {};

/**
 * \brief Executes code sequentially on the currently running thread
 * and doesn't offer any sort of parallelism.
 *
 * \todo Remove Blocking property since it cannot offer non-blocking behaviour
 */
template <typename Blocking = blocking_t::always_t,
          typename ProtoAllocator = std::allocator<void>>
struct inline_executor {
  using shape_type = uindex_t;
  using index_type = uindex_t;

  template <typename Executor, InstanceOf<Executor, executor::inline_executor> = 0>
  friend constexpr bool
  operator==(const inline_executor&, const Executor&) noexcept
  {
    return std::is_same<inline_executor, Executor>::value;
  }

  template <typename Executor, InstanceOf<Executor, executor::inline_executor> = 0>
  friend constexpr bool
  operator!=(const inline_executor& lhs, const Executor& rhs) noexcept
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
  bulk_execute(Function&& f, const std::size_t& shape) const
  {
    for (index_type idx = 0; idx < shape; ++idx)
      f(idx);
  }

  static constexpr decltype(auto)
  query(const blocking_t&) noexcept
  {
    return blocking_t::always;
  }

  inline_executor<blocking_t::always_t, ProtoAllocator>
  require(const blocking_t::always_t&) const
  {
    return {};
  }
};

using default_inline_executor = inline_executor<>;

} // namespace executor
} // namespace pcl
