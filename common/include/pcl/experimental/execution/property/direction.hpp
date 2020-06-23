//
// Created by Shrijit Singh on 14/06/20.
//

#pragma once

#include <pcl/experimental/execution//property/base_property.hpp>

class oneway_t : public basic_executor_property<oneway_t, true, true> {
  template <typename Executor>
  friend Executor require(Executor &ex, const oneway_t &t) {
    return ex.require(t);
  }
  template <class Executor>
  friend bool query(const Executor &ex, const oneway_t &t) {
    return std::is_same<oneway_t, decltype(ex.interface)>();
  }
};

static constexpr oneway_t oneway{};

class twoway_t : public basic_executor_property<twoway_t, true, true> {
  template <typename Executor>
  friend Executor require(Executor &ex, const twoway_t &t) {
    return ex.require(t);
  }
  template <class Executor>
  friend bool query(const Executor &ex, const twoway_t &t) {
    return std::is_same<twoway_t, decltype(ex.interface)>();
  }
};

static constexpr twoway_t twoway{};
