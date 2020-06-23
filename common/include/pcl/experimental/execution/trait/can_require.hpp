//
// Created by Shrijit Singh on 14/06/20.
//

#pragma once

#include <pcl/experimental/execution//trait/common_traits.hpp>
#include <type_traits>

namespace execution {

template <typename Executor, typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property_v<Executor> &&
                  Property::is_requirable &&
                  Property::template static_query<Executor>(),
              int> = 0>
Executor require(Executor ex, const Property t) {
  return ex.require(t);
}

template <typename Executor, typename Properties, typename = void>
struct can_require : std::false_type {};

template <typename Executor, typename Property>
struct can_require<Executor, Property,
                   detail::void_t<decltype(execution::require(
                       std::declval<Executor>(), std::declval<Property>()))>>
    : std::true_type {};

template <typename Executor, typename Property>
constexpr bool can_require_v = can_require<Executor, Property>::value;

}  // namespace experimental
