/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <experimental/type_traits>
#include <type_traits>

namespace executor {

template <typename Executor, template <typename...> class Type>
struct is_instance_of_base_impl : std::false_type {};

template <template <typename...> class Type, typename... Args>
struct is_instance_of_base_impl<Type<Args...>, Type> : std::true_type {};

template <typename Executor, template <typename...> class... Type>
using is_instance_of_base = executor::disjunction<is_instance_of_base_impl<Executor, Type>...>;

template <typename Executor, template <typename...> class... Type>
constexpr bool is_instance_of_base_v =
    is_instance_of_base<Executor, Type...>::value;

template <typename Executor, template <typename...> class... Type>
using instance_of_base =
    std::enable_if_t<is_instance_of_base_v<Executor, Type...>, int>;

}  // namespace executor
