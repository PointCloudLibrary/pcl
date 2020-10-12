/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/trait/common_traits.hpp>

#include <type_traits>

namespace pcl {
namespace executor {

namespace detail {
/**
 * \todo Convert to a static constexpr in C++ 17 onwards
 * Lambda closure types are non-literal types before C++17 and
 * non-literal types cannot be constexpr expressions
 */
static const auto noop = [] {};

/**
 * \brief Checks if the given type provides a member function named \p execute
 * that takes a callable which takes no arguments and returns void
 *
 * \detail In PCL it is used to check for the presence of the \p execute member
 * function inside an executor as having an \p execute member function is one the
 * requirements for a type to qualify as an executor
 *
 * \tparam T type to check for member function `execute`
 */
template <typename T, typename = void>
struct contains_execute : std::false_type {};

template <typename T>
struct contains_execute<
    T,
    std::enable_if_t<
        std::is_same<decltype(std::declval<T>().execute(detail::noop)), void>::value>>
: std::true_type {};

} // namespace detail

/**
 * \brief Checks whether the type is an executor
 *
 * \details A given type T is an executor if it satisfies the following
 * conditions:
 * 1. Provides a function named \p execute that eagerly submits work on a single
 * execution agent created for it by the executor i.e. take a callable and invokes
 * that callable. See \ref contains_execute
 * 2. Satisfies the named requirement
 * \ref https://en.cppreference.com/w/cpp/named_req/CopyConstructible
 * "CopyConstructible"
 * 3. Satisfies the named requirement
 * \ref https://en.cppreference.com/w/cpp/named_req/EqualityComparable
 * "EqualityComparable"
 *
 * This concept was finalized in P1660R0 and merged in the draft P0443R13 in
 * the form of a concept
 *
 * \todo Convert to a concept in C++20
 *
 * \tparam T type to check whether is an executor or not
 */
template <class T, typename = void>
struct is_executor : std::false_type {};

template <typename T>
struct is_executor<T,
                   std::enable_if_t<std::is_copy_constructible<T>::value &&
                                    detail::contains_execute<T>::value &&
                                    executor::equality_comparable<T, T>::value>>
: std::true_type {};

template <typename T>
static constexpr bool is_executor_v = is_executor<T>::value;

} // namespace executor
} // namespace pcl
