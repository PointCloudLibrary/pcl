/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, OpenPerception
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
 */

#pragma once

/**
 * \file pcl/types.h
 *
 * \brief Defines basic non-point types used by PCL
 * \ingroup common
 */

#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>
#include <vector>

#include <cstdint>

#include <Eigen/Core>

namespace pcl
{
  namespace detail {
    /**
     * \brief int_type::type refers to an integral type that satisfies template parameters
     * \tparam Bits number of bits in the integral type
     * \tparam Signed signed or unsigned nature of the type
     */
    template <std::size_t Bits, bool Signed = true>
    struct int_type { using type = void; };

    /**
     * \brief helper type to use for `int_type::type`
     * \see int_type
     */
    template <std::size_t Bits, bool Signed = true>
    using int_type_t = typename int_type<Bits, Signed>::type;

    template <>
    struct int_type<8, true> { using type = std::int8_t; };
    template <>
    struct int_type<8, false> { using type = std::uint8_t; };
    template <>
    struct int_type<16, true> { using type = std::int16_t; };
    template <>
    struct int_type<16, false> { using type = std::uint16_t; };
    template <>
    struct int_type<32, true> { using type = std::int32_t; };
    template <>
    struct int_type<32, false> { using type = std::uint32_t; };
    template <>
    struct int_type<64, true> { using type = std::int64_t; };
    template <>
    struct int_type<64, false> { using type = std::uint64_t; };

    /**
     * \brief number of bits in PCL's index type
     *
     * Please use PCL_INDEX_SIZE when building PCL to choose a size best suited for your needs.
     * PCL 1.12 will come with default 32
     *
     * PCL 1.11 has a default size = sizeof(int)
     */
    constexpr std::uint8_t index_type_size = PCL_INDEX_SIZE;

    /**
     * \brief signed/unsigned nature of PCL's index type
     * Please use PCL_INDEX_SIGNED when building PCL to choose a type best suited for your needs.
     * Default: signed
     */
    constexpr bool index_type_signed = PCL_INDEX_SIGNED;
}  // namespace detail

  /**
   * \brief Type used for an index in PCL
   *
   * Default index_t = int for PCL 1.11, std::int32_t for PCL >= 1.12
   */
  using index_t = detail::int_type_t<detail::index_type_size, detail::index_type_signed>;
  static_assert(!std::is_void<index_t>::value, "`index_t` can't have type `void`");

     /**
   * \brief Type used for an unsigned index in PCL
   *
   * Unsigned index that mirrors the type of the index_t
   */
  using uindex_t = detail::int_type_t<detail::index_type_size, false>;
  static_assert(!std::is_signed<uindex_t>::value, "`uindex_t` must be unsigned");

  /**
   * \brief Type used for indices in PCL
   * \todo Remove with C++20
   */
  template <typename Allocator = std::allocator<index_t>>
  using IndicesAllocator = std::vector<index_t, Allocator>;

  /**
   * \brief Type used for indices in PCL
   */
  using Indices = IndicesAllocator<>;

  /**
   * \brief Type used for aligned vector of Eigen objects in PCL
   */
  template <typename T>
  using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
}  // namespace pcl

