/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/common/utils.h>
#include <pcl/experimental/executor/trait/common_traits.hpp>

#include <type_traits>

namespace pcl {
namespace executor {

/**
 * \brief Checks whether a specified Property is supported by the executor.
 *
 * \details If supported it returns the current instance of that Property.
 * prefer denotes a customization point and should satisfy the following
 * conditions to be applicable:
 * 1. The Property should be applicable which can be checked using
 * \ref base_executor_property::is_applicable_property
 * "Property::template is_applicable_property<Executor>::value"
 * 2. The expression
 * \ref base_executor_property::static_query
 * "Property::template static_query<Executor>::value" should be a valid
 * constant expression
 *
 * If all the above conditions are met, then the overload query member function
 * in the executor is called with the Property.
 *
 * Part of Proposal P1393R0
 *
 * \tparam Executor an executor to query the property on
 * \tparam Property the property to query
 */
template <typename Executor,
          typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property<Executor>::value,
              int> = 0>
constexpr auto
query(Executor&&, const Property&) noexcept
{
  return Property::template static_query<Executor>::value;
}

/**
 * \brief Checks whether the given property and executor support the query
 * customization point.
 *
 * Part of Proposal P1393R0
 *
 * \tparam Executor an executor to check the property for
 * \tparam Property a property to check
 */
template <typename Executor, typename Properties, typename = void>
struct can_query : std::false_type {};

template <typename Executor, typename Property>
struct can_query<
    Executor,
    Property,
    pcl::void_t<decltype(query(std::declval<Executor>(), std::declval<Property>()))>>
: std::true_type {};

template <typename Executor, typename Property>
constexpr bool can_query_v = can_query<Executor, Property>::value;

} // namespace executor
} // namespace pcl
