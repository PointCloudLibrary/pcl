//
// Created by Shrijit Singh on 2020-06-14.
//

#pragma once

#include <array>
#include <pcl/experimental/execution//property.hpp>
#include <pcl/experimental/execution//type_trait.hpp>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

template <template <typename...> class Derived, typename Interface,
          typename Cardinality, typename Blocking, typename ProtoAllocator>
class executor {
 public:
  template <typename Executor,
            typename execution::instance_of_base<Derived, Executor> = 0>
  bool operator==(const Executor &) const noexcept {
    return std::is_same<
        Derived<Interface, Cardinality, Blocking, ProtoAllocator>,
        Executor>::value;
  }

  template <typename Executor,
            typename execution::instance_of_base<Derived, Executor> = 0>
  bool operator!=(const Executor &) const noexcept {
    return std::is_same<
        Derived<Interface, Cardinality, Blocking, ProtoAllocator>,
        Executor>::value;
  }

  static constexpr bool query(const blocking_t::always_t &t) {
    return std::is_same<Blocking, blocking_t::always_t>();
  }

  static constexpr bool query(const blocking_t::never_t &t) {
    return std::is_same<Blocking, blocking_t::never_t>();
  }

  static constexpr bool query(const blocking_t::possibly_t &t) {
    return std::is_same<Blocking, blocking_t::possibly_t>();
  }

  static constexpr bool query(const oneway_t &t) {
    return std::is_same<Interface, oneway_t>();
  }

  static constexpr bool query(const twoway_t &t) {
    return std::is_same<Interface, twoway_t>();
  }

  template <typename F, typename... Args>
  void bulk_execute(F &&f, Args &&... args, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
      invoke_hpp::invoke(std::forward<F>(f), std::forward<Args>(args)..., i);
    }
  }
};
