//
// Created by Shrijit Singh on 2020-06-14.
//

#pragma once

#include <pcl/experimental/execution//executor/base_executor.hpp>

template <typename Interface, typename Cardinality, typename Blocking,
          typename ProtoAllocator>
struct inline_executor;

namespace execution {
template <>
struct is_executor_available<inline_executor> : std::true_type {};
}  // namespace experimental

template <typename Interface, typename Cardinality, typename Blocking,
          typename ProtoAllocator = std::allocator<void>>
struct inline_executor
    : public executor<inline_executor, Interface, Cardinality, Blocking,
                      ProtoAllocator> {
  using shape_type = std::size_t;

  template <typename F>
  void execute(F &&f) {
    invoke_hpp::invoke(std::forward<F>(f));
  }

  //  inline_executor &decay_t() { return *this; };

  inline_executor<Interface, Cardinality, blocking_t::always_t, ProtoAllocator>
  require(const blocking_t::always_t &t) {
    return {};
  }

  std::string name() { return "inline"; }
};
