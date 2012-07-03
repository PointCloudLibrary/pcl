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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: byte_order.h 4868 2012-03-01 02:50:19Z rusu $
 *
 */

#ifndef PCL_IO_PLY_BYTE_ORDER_H
#define PCL_IO_PLY_BYTE_ORDER_H

namespace pcl
{
  namespace io
  {
    namespace ply
    {
      /** \file byte_order.h
        * defines byte shift operations and endianess.
        * \author Ares Lagae as part of libply, Nizar Sallem
        * \ingroup io
        */

#if defined (PLY_BIG_ENDIAN) || defined (PLY_LITTLE_ENDIAN)
#  error
#endif

#if (defined (__powerpc) || defined (__powerpc__) || defined (__POWERPC__) || defined (__ppc__) || defined (_M_PPC) || defined (__ARCH_PPC))
#  define PLY_BIG_ENDIAN
#elif (defined (i386) || defined (__i386__) || defined (__i386) || defined (_M_IX86) || defined (_X86_) || defined (__THW_INTEL__) || defined (__I86__) || defined (__INTEL__)) \
  || (defined (__amd64__) || defined (__amd64) || defined (__x86_64__) || defined (__x86_64) || defined (_M_X64) || defined (ANDROID))
#  define PLY_LITTLE_ENDIAN
#else
#  error
#endif

      enum byte_order
      {
        little_endian_byte_order,
        big_endian_byte_order,
#if defined (PLY_BIG_ENDIAN)
        host_byte_order = big_endian_byte_order,
#elif defined (PLY_LITTLE_ENDIAN)
        host_byte_order = little_endian_byte_order,
#else
#  error
#endif
        network_byte_order = big_endian_byte_order
      };
      
#undef PLY_BIG_ENDIAN
#undef PLY_LITTLE_ENDIAN
      
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

#endif // PLY_BYTE_ORDER_H
