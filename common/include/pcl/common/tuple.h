/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *
 */

#pragma once

#include <pcl/type_traits.h>

#include <type_traits>
#include <tuple>

namespace pcl {

namespace detail {
template <typename Tuple, typename Function>
void
for_each_until_true(
    Tuple&&,
    Function,
    std::integral_constant<
        std::size_t,
        std::tuple_size<typename std::remove_reference<Tuple>::type>::value>)
{}

template <
    typename Tuple,
    typename Function,
    std::size_t I,
    typename = typename std::enable_if<
        I != std::tuple_size<typename std::remove_reference<Tuple>::type>::value>::type>
void
for_each_until_true(Tuple&& t, Function f, std::integral_constant<size_t, I>)
{
  bool exit = f(std::get<I>(std::forward<Tuple>(t)));

  if (!exit)
    for_each_until_true(
        std::forward<Tuple>(t), f, std::integral_constant<size_t, I + 1>());
}
} // namespace detail

/**
 * \brief Iterates over all elements of tuples until the function called returns true
 *
 * \tparam Tuple The tuple to iterate through
 * \tparam Function A callable that is invoked for every tuple element and returns a
 * boolean indicating whether to continue iteration or not
 *
 * \remark Implementation taken from
 * https://stackoverflow.com/questions/26902633/how-to-iterate-over-a-stdtuple-in-c-11
 */
template <typename Tuple, typename Function>
void
for_each_until_true(Tuple&& t, Function f)
{
  detail::for_each_until_true(std::forward<Tuple>(t), f, std::integral_constant<size_t, 0>());
}

namespace detail {

template <typename T, typename Tuple>
struct tuple_contains_type_impl;

template <typename T, typename... Us>
struct tuple_contains_type_impl<T, std::tuple<Us...>>
: pcl::disjunction<std::is_same<T, Us>...> {};

} // namespace detail

/**
 * \brief If the \Tuple contains the type \p T then provides the member constant value
 * equal true. For any other type, value is false.
 * *
 * \tparam T a type to check
 * \tparam Tuple a tuple in which to check for the type
 *
 */
template <typename T, typename Tuple>
using tuple_contains_type = detail::tuple_contains_type_impl<T, Tuple>;

template <typename T, typename Tuple>
constexpr bool tuple_contains_type_v = detail::tuple_contains_type_impl<T, Tuple>::value;

static_assert(tuple_contains_type_v<int, std::tuple<int, float, double, unsigned>>,
              "Failed to check type in tuple");

namespace detail {

template <template <typename...> class Predicate, typename... TupleElements>
struct filter_tuple_values_impl {
  using type = decltype(std::tuple_cat(
      typename std::conditional<Predicate<TupleElements>::value, std::tuple<TupleElements>, std::tuple<>>::
          type()...));

  /**
   * \brief Checks whether a tuple contains a specified type
   *
   * \tparam TupleElements the elements of the tuple you want to filter
   * \param std::tuple<TupleElements...> a tuple of the elements you want to filter
   * \return a tuple containing the filtered elements
   *
   */
  auto
  operator()(const std::tuple<TupleElements...>& in)
  {
    return (*this)(in, type{});
  }

private:
  // Utility function to fetch the types we're interest in outputting
  template <typename... To>
  auto
  operator()(const std::tuple<TupleElements...>& in, std::tuple<To...>)
  {
    return std::make_tuple(std::get<To>(in)...);
  }
};

} // namespace detail

/**
 * \brief Filters elements of \p Tuple based on the provided predicate/condition
 *
 * \tparam Predicate A trait which takes a tuple element as parameter and defines a
 * static boolean member \p value which dictates whether to filter the tuple element or not
 * \tparam Tuple a tuple to filter
 *
 */
template <template <typename...> class Predicate, typename Tuple>
struct filter_tuple_values;

template <template <typename...> class Predicate, typename... TupleElements>
struct filter_tuple_values<Predicate, std::tuple<TupleElements...>>
: detail::filter_tuple_values_impl<Predicate, TupleElements...> {};

template <template <typename...> class Predicate, typename... TupleElements>
struct filter_tuple_values<Predicate, const std::tuple<TupleElements...>>
: detail::filter_tuple_values_impl<Predicate, TupleElements...> {};

static_assert(
    std::is_same<
        std::tuple<float, double>,
        filter_tuple_values<std::is_floating_point,
                            std::tuple<int, float, double, unsigned>>::type>::value,
    "Filtered types do not match");

} // namespace pcl
