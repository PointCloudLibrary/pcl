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

#include <istream>
#include <limits>
#include <ostream>

namespace pcl
{
  namespace io
  {
    namespace ply 
    {
      /** \file io_operators.h
        * defines output operators for int8 and uint8
        * \author Ares Lagae as part of libply, Nizar Sallem
        * \ingroup io
        */
      namespace io_operators 
      {

        inline std::istream& operator>> (std::istream& istream, int8 &value)
        {
          int16 tmp;
          if (istream >> tmp)
          {
            if (tmp <= std::numeric_limits<int8>::max ())
              value = static_cast<int8> (tmp);
            else
              istream.setstate (std::ios_base::failbit);
          }
          return (istream);
        }

        inline std::istream& operator>> (std::istream& istream, uint8 &value)
        {
          uint16 tmp;
          if (istream >> tmp)
          {
            if (tmp <= std::numeric_limits<uint8>::max ())
              value = static_cast<uint8> (tmp);
            else
              istream.setstate (std::ios_base::failbit);
          }
          return (istream);
        }

        inline std::ostream& operator<<(std::ostream& ostream, int8 value)
        {
          return (ostream << static_cast<int16> (value));
        }

        inline std::ostream& operator<<(std::ostream& ostream, uint8 value)
        {
          return (ostream << static_cast<uint16> (value));
        }

      } // namespace io_operators
    } // namespace ply
  } // namespace io
} // namespace pcl
