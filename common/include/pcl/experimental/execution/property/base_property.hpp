//
// Created by Shrijit Singh on 2020-06-14.
//

#pragma once

#include <pcl/experimental/execution//type_trait.hpp>
#include <functional>
#include <iostream>
#include <string>

template <typename Derived, bool requireable, bool preferable>
class basic_executor_property {
 public:
  static constexpr bool is_requirable = requireable;
  static constexpr bool is_preferable = preferable;

  template <class T>
  static constexpr bool is_applicable_property() {
    return execution::is_executor<T>();
  }

  template <class Executor>
  static constexpr auto static_query()
      -> decltype(Executor::query(std::declval<Derived>())) {
    return Executor::query(Derived{});
  }

  template <typename T>
  static constexpr bool is_applicable_property_v = is_applicable_property<T>();

  template <class Executor>
  static constexpr decltype(auto) static_query_v = static_query<Executor>();
};
