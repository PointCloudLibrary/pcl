/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/trait/is_executor.hpp>

#include <functional>

namespace pcl {
namespace executor {

/**
 * \brief A base class for all properties, containing the common functionality
 * needed by all properties
 *
 * \details A base class was needed as each property needed to specify a minimum
 * set of functionality such as
 * \ref base_executor_property::is_applicable_property "is_applicable_property",
 * \ref base_executor_property::static_query "static query",
 * \ref base_executor_property::is_requirable "is_requirable",
 * \ref base_executor_property::is_preferable "is_preferable" etc.
 * which would lead to code duplication without a base class.
 *
 * A CRTP design is used as static query needs to be able to access the
 * DerivedProperty in order to call the overloaded query member function in an
 * executor. It also allows to specify on a property basis whether it is
 * requirable and preferable.
 * This is useful for behavioral properties (P0443R13- 2.2.12)
 * which are not requirable nor preferable unlike the properties nested in them.
 */
template <typename DerivedProperty, bool requireable, bool preferable>
struct base_executor_property {
  /**
   * \brief Specifies whether property supports require customization point
   *
   * Part of Proposal P1393R0
   */
  static constexpr bool is_requirable = requireable;

  /**
   * \brief Specifies whether property supports prefer customization point
   *
   * Part of Proposal P1393R0
   */
  static constexpr bool is_preferable = preferable;

  /**
   * \brief Checks whether the given Property is applicable for the given type T.
   *
   * \details Currently for the property to be applicable T should be an executor type.
   * In proposal P1393R0 there are no conditions specified for a property to be
   * applicable and the design is left up to the implementer
   *
   * Part of Proposal P1393R0
   *
   * \todo Convert is_applicable_property to a static constexpr in GCC 6 onwards
   * Workaround:
   * stackoverflow.com/questions/45607450/gcc5-nested-variable-template-is-not-a-function-template
   */
  template <typename T>
  struct is_applicable_property {
    static constexpr bool value = is_executor_v<T>;
  };

  /**
   * \brief A static query for a property supported by an Executor
   *
   * \details Allows checking whether a property is supported by an Executor
   * without an instance of an executor using the overloaded query memeber function
   * in Executor
   *
   * Part of Proposal P0443R13 (2.2.11 & 2.2.12) and P1393R0
   *
   * \todo Convert static_query to a static constexpr in GCC 6 onwards
   * Workaround:
   * stackoverflow.com/questions/45607450/gcc5-nested-variable-template-is-not-a-function-template
   */
  template <typename Executor>
  struct static_query {
    static constexpr auto value =
        std::remove_reference_t<Executor>::query(DerivedProperty{});
  };

  /**
   * \warning: Not compatible with GCC 5 and lower versions
   * Workaround:
   * stackoverflow.com/questions/45607450/gcc5-nested-variable-template-is-not-a-function-template
   */
  template <typename T>
  static constexpr bool is_applicable_property_v = is_applicable_property<T>::value;

  /**
   * \warning: Not compatible with GCC 5 and lower versions
   * Workaround:
   * stackoverflow.com/questions/45607450/gcc5-nested-variable-template-is-not-a-function-template
   */
  template <class Executor>
  static constexpr auto static_query_v = static_query<Executor>::value;
};

namespace detail {
/**
 * \brief Checks if the given Executor supports the Property
 *
 *\details This is checked through a template variable static_query_v which is
 * provided by all properties
 */
template <typename Executor, typename Property>
using contains_property = std::is_same<
    std::remove_const_t<decltype(Property::template static_query<Executor>::value)>,
    Property>;
} // namespace detail

} // namespace executor
} // namespace pcl
