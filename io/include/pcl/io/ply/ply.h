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

#ifndef PLY_PLY_H
#define PLY_PLY_H

#include <pcl/io/boost.h>
#include <pcl/io/ply/byte_order.h>

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
      typedef boost::int8_t int8;
      typedef boost::int16_t int16;
      typedef boost::int32_t int32;
      typedef boost::uint8_t uint8;
      typedef boost::uint16_t uint16;
      typedef boost::uint32_t uint32;         
      
      typedef float float32;
      typedef double float64;
      
      template <typename ScalarType>
        struct type_traits;
      
#ifdef PLY_TYPE_TRAITS
#  error
#endif
      
#define PLY_TYPE_TRAITS(TYPE, NAME, OLD_NAME)               \
      template <>                                           \
      struct type_traits<TYPE>                              \
      {                                                     \
        typedef TYPE type;                                  \
        static const char* name() { return NAME; }          \
        static const char* old_name() { return OLD_NAME; }  \
      }

      PLY_TYPE_TRAITS(int8, "int8", "char");
      PLY_TYPE_TRAITS(int16, "int16", "short");
      PLY_TYPE_TRAITS(int32, "int32", "int");
      PLY_TYPE_TRAITS(uint8, "uint8", "uchar");
      PLY_TYPE_TRAITS(uint16, "uint16", "ushort");
      PLY_TYPE_TRAITS(uint32, "uint32", "uint");
      PLY_TYPE_TRAITS(float32, "float32", "float");
      PLY_TYPE_TRAITS(float64, "float64", "double");
      
#undef PLY_TYPE_TRAITS
      
      typedef int format_type;
      enum format { ascii_format, binary_little_endian_format, binary_big_endian_format, unknown };  
    } // namespace ply
  } // namespace io
} // namespace pcl
#endif // PCL_IO_PLY_PLY_H
