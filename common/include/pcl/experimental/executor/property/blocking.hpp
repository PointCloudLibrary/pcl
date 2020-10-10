/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/property/base_property.hpp>

namespace pcl {
namespace executor {

/**
 * \brief A behavioral property (P0443R13 - 2.2.12) that guarantees executors provide
 * about the blocking behavior of their execution functions.
 *
 * \details Blocking behaviour here refers to whether or not the execution function
 * should return immediately after invoking the callable.
 * The blocking property provides 3 nested property which represent different blocking
 * behaviours:
 * 1. Possible - May or may not block depending on context no guarantees are made
 * 2. Always - Always blocking
 * 3. Never - Never blocking
 *
 * Part of Proposal P0443R13 (2.2.12.1)
 *
 * \todo Look into and implement blocking_adaptation_t if needed
 */
struct blocking_t : base_executor_property<blocking_t, false, false> {
  /**
   * \brief The equality operator is overloaded to be able to compare if the
   * property instance queried from and executor matches the property we expect
   */
  friend constexpr bool
  operator==(const blocking_t& a, const blocking_t& b)
  {
    return a.value_ == b.value_;
  }

  friend constexpr bool
  operator!=(const blocking_t& a, const blocking_t& b)
  {
    return !(a == b);
  }

  constexpr blocking_t() = default;

  struct always_t : base_executor_property<always_t, true, true> {
    static constexpr blocking_t
    value()
    {
      return {};
    }
  };

  static constexpr always_t always{};

  constexpr blocking_t(const always_t&) : value_{ALWAYS} {};

  struct never_t : base_executor_property<never_t, true, true> {
    static constexpr blocking_t
    value()
    {
      return {};
    }
  };

  static constexpr never_t never{};

  constexpr blocking_t(const never_t&) : value_{NEVER} {};

  struct possibly_t : base_executor_property<possibly_t, true, true> {
    static constexpr blocking_t
    value()
    {
      return {};
    }
  };

  static constexpr possibly_t possibly{};

  constexpr blocking_t(const possibly_t&) : value_{POSSIBLY} {};

  /**
   * \brief Default property value i.e. always
   */
  template <typename Executor>
  struct static_query {
    static constexpr auto value = always;
  };

  template <typename Executor>
  static constexpr decltype(auto) static_query_v = static_query<Executor>::value;

  /**
   * \brief Used for having an order between the nested properties
   */
  const enum { DEFAULT, ALWAYS, NEVER, POSSIBLY } value_ = DEFAULT;
};

static constexpr blocking_t blocking{};

} // namespace executor
} // namespace pcl
