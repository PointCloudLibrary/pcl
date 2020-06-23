//
// Created by Shrijit Singh on 14/06/20.
//

#pragma once

#include <pcl/experimental/execution//property/base_property.hpp>

class blocking_t {
 public:
  class always_t : public basic_executor_property<always_t, true, true> {
   public:
    template <class Executor>
    friend Executor require(Executor &ex, const always_t &t) {
      return ex.require(t);
    }
    template <class Executor>
    friend bool query(const Executor &ex, const always_t &t) {
      return std::is_same<always_t, decltype(ex.interface)>();
    }
  };

  always_t always;

  class never_t : public basic_executor_property<never_t, true, true> {
   public:
    template <class Executor>
    friend Executor require(Executor &ex, const never_t &t) {
      return ex.require(t);
    }
    template <class Executor>
    friend bool query(const Executor &ex, const never_t &t) {
      return std::is_same<never_t, decltype(ex.interface)>();
    }
  };

  never_t never;

  class possibly_t : public basic_executor_property<possibly_t, true, true> {
   public:
    template <class Executor>
    friend Executor require(Executor &ex, const possibly_t &t) {
      return ex.require(t);
    }
    template <class Executor>
    friend bool query(const Executor &ex, const possibly_t &t) {
      return std::is_same<never_t, decltype(ex.interface)>();
    }
  };

  possibly_t possibly;
};

static const blocking_t blocking{};
