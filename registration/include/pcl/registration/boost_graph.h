/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id$
 *
 */

#pragma once

#include <boost/graph/adjacency_list.hpp>

#include <Eigen/StdVector>

#include <list>

namespace boost {

struct eigen_vecS {};

template <class ValueType>
struct container_gen<eigen_vecS, ValueType> {
  using type = std::vector<ValueType, Eigen::aligned_allocator<ValueType>>;
};

template <>
struct parallel_edge_traits<eigen_vecS> {
  using type = allow_parallel_edge_tag;
};

namespace detail {
template <>
struct is_random_access<eigen_vecS> {
  enum { value = true };
  using type = mpl::true_;
};
} // namespace detail

struct eigen_listS {};

template <class ValueType>
struct container_gen<eigen_listS, ValueType> {
  using type = std::list<ValueType, Eigen::aligned_allocator<ValueType>>;
};

template <>
struct parallel_edge_traits<eigen_listS> {
  using type = allow_parallel_edge_tag;
};

namespace detail {
template <>
struct is_random_access<eigen_listS> {
  enum { value = false };
  using type = mpl::false_;
};
} // namespace detail
} // namespace boost
