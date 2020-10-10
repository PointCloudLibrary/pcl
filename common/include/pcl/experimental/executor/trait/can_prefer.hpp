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
#include <pcl/experimental/executor/trait/can_require.hpp>
#include <pcl/experimental/executor/trait/common_traits.hpp>

#include <type_traits>

namespace pcl {
namespace executor {

/**
 * \brief Enforces a specified property on an executor if possible else returns
 * the same executor.
 *
 * \details If enforced a new executor instance which implements that
 * property is created and returned. prefer denotes a customization point and
 * should satisfy the following conditions to be applicable:
 * 1. The Property should be applicable and preferable which can be checked using
 * \ref base_executor_property::is_applicable_property
 * "Property::template is_applicable_property<Executor>::value" and
 * \ref base_executor_property::is_preferable "Property::is_preferable"
 * 2. The expression
 * \ref base_executor_property::static_query
 * "Property::template static_query<Executor>::value" == Property::value() should
 * be true, which implies that the executor supports that property
 *
 * If all the above conditions are met, prefer customization point is valid.
 * If it is possible, then the require customization point is invoked
 * and the property is enforced for the executor. If the above case is not
 * possible, then the same executor is returned.
 *
 * Part of Proposal P1393R0
 *
 * \todo
 * 1. Support multiple querying multiple properties in the trait:
 * template <typename Executor, typename... Properties>
 *
 * \tparam Executor an executor to try and enforce the property on
 * \tparam Property the property to try enforce
 */
template <typename Executor,
          typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property<Executor>::value &&
                  Property::is_preferable && can_require_v<Executor, Property>,
              int> = 0>
constexpr decltype(auto)
prefer(const Executor& ex, const Property& p) noexcept
{
  return ex.require(p);
}

template <typename Executor,
          typename Property,
          typename std::enable_if_t<
              Property::template is_applicable_property<Executor>::value &&
                  Property::is_preferable && !can_require_v<Executor, Property>,
              int> = 0>
constexpr decltype(auto)
prefer(const Executor& ex, const Property&) noexcept
{
  return ex;
}

/**
 * \brief Checks whether the given Property and Executor support the \ref prefer
 * customization point.
 *
 * Part of Proposal P1393R0
 *
 * \tparam Executor an executor to check the property for
 * \tparam Property a property to check
 */
template <typename Executor, typename Property, typename = void>
struct can_prefer : std::false_type {};

template <typename Executor, typename Property>
struct can_prefer<
    Executor,
    Property,
    pcl::void_t<decltype(prefer(std::declval<Executor>(), std::declval<Property>()))>>
: std::true_type {};

template <typename Executor, typename Property>
constexpr bool can_prefer_v = can_prefer<Executor, Property>::value;

} // namespace executor
} // namespace pcl
