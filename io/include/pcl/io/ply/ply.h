/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2007-2012, Ares Lagae
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <pcl/io/ply/byte_order.h>
#include <cstdint> // for int8_t, int16_t, ...

/** \file ply.h contains standard typedefs and generic type traits
  * \author Ares Lagae as part of libply, Nizar Sallem
  * Ported with agreement from the author under the terms of the BSD
  * license.
  * \ingroup io
  */
namespace pcl
{
  namespace io
  {
    namespace ply 
    {
      using int8 = std::int8_t;
      using int16 = std::int16_t;
      using int32 = std::int32_t;
      using uint8 = std::uint8_t;
      using uint16 = std::uint16_t;
      using uint32 = std::uint32_t;         
      
      using float32 = float;
      using float64 = double;
      
      template <typename ScalarType>
        struct type_traits;
      
#ifdef PLY_TYPE_TRAITS
#  error
#endif
      
#define PLY_TYPE_TRAITS(TYPE, PARSE_TYPE, NAME, OLD_NAME)   \
      template <>                                           \
      struct type_traits<TYPE>                              \
      {                                                     \
        using type = TYPE;                                  \
        using parse_type = PARSE_TYPE;                      \
        static const char* name () { return NAME; }         \
        static const char* old_name () { return OLD_NAME; } \
      };

      PLY_TYPE_TRAITS(int8, int16, "int8", "char")
      PLY_TYPE_TRAITS(int16, int16, "int16", "short")
      PLY_TYPE_TRAITS(int32, int32, "int32", "int")
      PLY_TYPE_TRAITS(uint8, uint16, "uint8", "uchar")
      PLY_TYPE_TRAITS(uint16, uint16, "uint16", "ushort")
      PLY_TYPE_TRAITS(uint32, uint32, "uint32", "uint")
      PLY_TYPE_TRAITS(float32, float32, "float32", "float")
      PLY_TYPE_TRAITS(float64, float64, "float64", "double")

      
#undef PLY_TYPE_TRAITS
      
      using format_type = int;
      enum format { ascii_format, binary_little_endian_format, binary_big_endian_format, unknown };  
    } // namespace ply
  } // namespace io
} // namespace pcl
