/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#ifdef __GNUC__
#pragma GCC system_header
#endif

#include <algorithm>
#if __cplusplus >= 201703L
#include <execution>
#endif
#include <type_traits>

#define _PCL_EXECUTION_POLICY true

namespace pcl {
  namespace execution {
    struct sequenced_policy;
    struct parallel_policy;
    struct parallel_unsequenced_policy;
    struct unsequenced_policy;

    namespace detail {
      struct execution_policy {};

      template <class T, class = void>
      struct is_execution_policy_helper : public std::false_type {};
#if __cplusplus >= 201703L
      template <class T>
      struct is_execution_policy_helper<
        T,
        std::enable_if_t<std::is_base_of<execution_policy, T>::value>>
      : public std::true_type {};
#else
      template <class T>
      struct is_execution_policy_helper<T, sequenced_policy> : public std::true_type {};
      template <class T>
      struct is_execution_policy_helper<T, parallel_policy> : public std::true_type {};
      template <class T>
      struct is_execution_policy_helper<T, parallel_unsequenced_policy>
      : public std::true_type {};
      template <class T>
      struct is_execution_policy_helper<T, unsequenced_policy> : public std::true_type {};
#endif
    } // namespace detail

    struct sequenced_policy : public detail::execution_policy {};
    struct parallel_policy : public detail::execution_policy {};
    struct parallel_unsequenced_policy : public detail::execution_policy {};
    struct unsequenced_policy : public detail::execution_policy {};

    constexpr sequenced_policy seq{};
    constexpr parallel_policy par{};
    constexpr parallel_unsequenced_policy par_unseq{};
    constexpr unsequenced_policy unseq{};
  } // namespace execution
  template <class T>
  struct is_execution_policy : public execution::detail::is_execution_policy_helper<T> {};
  template <class T>
  constexpr bool is_execution_policy_v = is_execution_policy<T>::value;
} // namespace pcl