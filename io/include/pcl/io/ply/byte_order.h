/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2007-2012, Ares Lagae
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <boost/predef/other/endian.h>

#include <cstddef>
#include <utility> // for swap

namespace pcl
{
  namespace io
  {
    namespace ply
    {
      /** \file byte_order.h
        * defines byte shift operations and endianness.
        * \author Ares Lagae as part of libply, Nizar Sallem
        * \ingroup io
        */

      enum byte_order
      {
        little_endian_byte_order,
        big_endian_byte_order,
#if BOOST_ENDIAN_BIG_BYTE
        host_byte_order = big_endian_byte_order,
#elif BOOST_ENDIAN_LITTLE_BYTE
        host_byte_order = little_endian_byte_order,
#else
#error "unable to determine system endianness"
#endif
        network_byte_order = big_endian_byte_order
      };

      template <std::size_t N>
      void swap_byte_order (char* bytes);

      template <>
      inline void swap_byte_order<1> (char*) {}

      template <>
      inline void swap_byte_order<2> (char* bytes)
      {
        std::swap (bytes[0], bytes[1]);
      }

      template <>
        inline void swap_byte_order<4> (char* bytes)
      {
        std::swap (bytes[0], bytes[3]);
        std::swap (bytes[1], bytes[2]);
      }
      
      template <>
        inline void swap_byte_order<8> (char* bytes)
      {
        std::swap (bytes[0], bytes[7]);
        std::swap (bytes[1], bytes[6]);
        std::swap (bytes[2], bytes[5]);
        std::swap (bytes[3], bytes[4]);
      }
      
      template <typename T>
      void swap_byte_order (T& value)
      {
        swap_byte_order<sizeof (T)> (reinterpret_cast<char*> (&value));
      }

    } // namespace ply
  } // namespace io
} // namespace pcl
