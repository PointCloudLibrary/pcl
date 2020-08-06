/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/property/base_property.hpp>

namespace executor {

// Part of Proposal P0443R13: 2.2.12.1
struct blocking_t : basic_executor_property<blocking_t, false, false> {
  friend constexpr bool operator==(const blocking_t& a, const blocking_t& b) {
    return a.value_ == b.value_;
  }

  friend constexpr bool operator!=(const blocking_t& a, const blocking_t& b) {
    return !(a == b);
  }

  constexpr blocking_t() : value_{0} {};

  struct always_t : basic_executor_property<always_t, true, true> {
    static constexpr blocking_t value() { return {}; }
  };

  static constexpr always_t always{};
  constexpr blocking_t(const always_t&) : value_{1} {};

  struct never_t : basic_executor_property<never_t, true, true> {
    static constexpr blocking_t value() { return {}; }
  };

  static constexpr never_t never{};
  constexpr blocking_t(const never_t&) : value_{2} {};

  struct possibly_t : basic_executor_property<possibly_t, true, true> {
    static constexpr blocking_t value() { return {}; }
  };

  static constexpr possibly_t possibly{};
  constexpr blocking_t(const possibly_t&) : value_{3} {};

  // Default property value
  template <typename Executor>
  static constexpr decltype(auto) static_query_v = always;

  int value_;
};

static constexpr blocking_t blocking{};

// Can inline member variable in C++17 eliminating need for sperate CPP file
// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
// inline constexpr blocking_t::possibly_t blocking_t::possibly;

}  // namespace executor
