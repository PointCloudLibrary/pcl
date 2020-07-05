/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/type_traits.h>

#include <type_traits>

namespace pcl {
namespace executor {

/**
 * \brief In accordance with the named requirement EqualityComparable which
 * will be formalized in C++20 using concepts
 * Requirements: https://en.cppreference.com/w/cpp/named_req/EqualityComparable
 */
template <typename T1, typename T2, typename = void>
struct equality_comparable : std::false_type {};

template <typename T1, typename T2>
struct equality_comparable<
    T1,
    T2,
    pcl::void_t<decltype(std::declval<T1>() == std::declval<T2>(),
                         std::declval<T2>() == std::declval<T1>(),
                         std::declval<T1>() != std::declval<T2>(),
                         std::declval<T2>() != std::declval<T1>())>> : std::true_type {
};

template <typename T1, typename T2>
constexpr bool equality_comparable_v = equality_comparable<T1, T2>::value;

/**
 * \brief Checks whether the executor is an instance of the specified
 * executor class template
 *
 * Example:
 * \code{.cpp}
 * is_instance_of<inline_executor<>, inline_executor>::value // true
 * is_instance_of<inline_executor<>, inline_executor>::value // false
 * \endcode
 *
 * \tparam Executor an executor
 * \tparam Type an executor class template
 */
template <typename Executor, template <typename...> class Type, typename = void>
struct is_instance_of : std::false_type {};

// clang-format off
template <template <typename...> class Executor,
          template <typename...> class Type,
          typename... Args>
struct is_instance_of<
    Executor<Args...>,
    Type,
    std::enable_if_t<std::is_base_of<Type<Args...>, Executor<Args...>>::value>>
: std::true_type {};
// clang-format on

template <typename Executor, template <typename...> class Type>
constexpr bool is_instance_of_v = is_instance_of<Executor, Type>::value;

/**
 * \brief A helper trait which uses enable_if in which the boolean condition is
 * \ref is_instance_of and the type of enable_if is int
 *
 * This is used to use SFINAE to disable/remove functions if the
 * executor is an instance of the specified executor class template
 *
 * In PCL this is used to choose an overload of a function based on the
 * executor type and throw a compile time error if a overload of that function
 * corresponding to that executor type doesn't exist but the function is
 * being called with the unsupported executor type.
 *
 * Example:
 * \code{.cpp}
 * template <typename Executor, InstanceOf<Executor, inline_executor> = 0>
 * void foo(Executor exec) {...}
 * \endcode
 * In the above example the foo function is disabled if the template parameter
 * Executor is not an instance of inline_executor.
 *
 * \tparam Executor an executor
 * \tparam Type an executor class template
 */
template <typename Executor, template <typename...> class Type>
using InstanceOf = std::enable_if_t<is_instance_of<Executor, Type>::value, int>;

/**
 * \brief Checks whether the executor is an instance any of the specified
 * executor class templates
 *
 * Example:
 * \code{.cpp}
 * is_instance_of_any<inline_executor<>, inline_executor, omp_executor>::value // true
 * is_instance_of_any<inline_executor<>, sse_executor, omp_executor>::value // false
 * \endcode
 *
 * \tparam Executor an executor
 * \tparam Types a parameter pack of executor class templates
 */
template <typename Executor, template <typename...> class... Types>
using is_instance_of_any = pcl::disjunction<is_instance_of<Executor, Types>...>;

template <typename Executor, template <typename...> class... Types>
constexpr bool is_instance_of_any_v = is_instance_of_any<Executor, Types...>::value;

/**
 * \brief A helper trait which uses enable_if in which the boolean condition is
 * \ref is_instance_of_any and the type of enable_if is int
 *
 * This is used to use SFINAE to disable/remove functions if the
 * executor is an instance of any of the specified executor class templates
 *
 * In PCL this is used to choose an overload of a function based on the
 * executor type and throw a compile time error if a overload of that function
 * corresponding to that executor type doesn't exist but the function is
 * being called with the unsupported executor type.
 *
 * Example:
 * \code{.cpp}
 * template <typename Executor, InstanceOfAny<Executor, sse_executor, omp_executor> = 0>
 * void foo(Executor exec) {...}
 * \endcode
 * In the above example the foo function is disabled if the template parameter
 * Executor is neither an instance of sse_executor nor omp_executor.
 *
 * \tparam Executor an executor
 * \tparam Type an executor class template
 */
template <typename Executor, template <typename...> class... Types>
using InstanceOfAny = std::enable_if_t<is_instance_of_any_v<Executor, Types...>, int>;

} // namespace executor
} // namespace pcl
