//
// Created by Shrijit Singh on 14/06/20.
//

#pragma once

#include <experimental/type_traits>
#include <type_traits>

namespace execution {

template <template <typename...> class Type, typename Executor>
struct is_instance_of_base : std::false_type {};

template <template <typename...> class Type, typename... Args>
struct is_instance_of_base<Type, Type<Args...>> : std::true_type {};

template <template <typename...> class Type, typename Executor>
constexpr bool is_instance_of_base_v =
    is_instance_of_base<Type, Executor>::value;

template <template <typename...> class Type, typename Executor>
using instance_of_base =
    std::enable_if_t<is_instance_of_base<Type, Executor>::value, int>;

}  // namespace experimental
