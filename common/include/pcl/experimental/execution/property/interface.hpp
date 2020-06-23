//
// Created by Shrijit Singh on 14/06/20.
//

#pragma once

#include <pcl/experimental/execution//property/base_property.hpp>

class single_t : public basic_executor_property<single_t, true, true> {
  template <typename Executor>
  friend Executor require(Executor &ex, const single_t &t) {
    return ex.require(t);
  }
  template <class Executor>
  friend bool query(const Executor &ex, const single_t &t) {
    return std::is_same<single_t, decltype(ex.interface)>();
  }
};

static constexpr single_t single{};

class bulk_t : public basic_executor_property<bulk_t, true, true> {
  template <typename Executor>
  friend Executor require(Executor &ex, const bulk_t &t) {
    return ex.require(t);
  }
  template <class Executor>
  friend bool query(const Executor &ex, const bulk_t &t) {
    return std::is_same<bulk_t, decltype(ex.interface)>();
  }
};

static constexpr bulk_t bulk{};
