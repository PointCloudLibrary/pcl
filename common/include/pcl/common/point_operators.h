/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_COMMON_POINT_OPERATORS_H
#define PCL_COMMON_POINT_OPERATORS_H

#include <pcl/point_types.h>

/**
  * \file pcl/common/point_operators.h
  * Define a set of operator on PCL point types.
  * 
  * \ingroup common
  */
namespace pcl
{
  namespace common
  {
    // Ugly hack: override the automatic template declaration for PointXYZRGB
    // note: must deal with overflow
    inline const pcl::PointXYZRGB& 
    operator+= (pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
    {
      lhs.x += rhs.x; lhs.y += rhs.y; lhs.z += rhs.z;
      lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (lhs);
    }
 
    inline const pcl::PointXYZRGB&
    operator-= (pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
    {
      lhs.x -= rhs.x; lhs.y -= rhs.y; lhs.z -= rhs.z;
      lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (lhs);
    }
 
    inline const pcl::PointXYZRGB&
    operator*= (pcl::PointXYZRGB& lhs, const float& scalar)
    {
      lhs.x *= scalar; lhs.y *= scalar; lhs.z *= scalar;
      lhs.r = static_cast<uint8_t> (lhs.r * scalar);
      lhs.g = static_cast<uint8_t> (lhs.g * scalar);
      lhs.b = static_cast<uint8_t> (lhs.b * scalar);
      return (lhs);
    }
 
 //   // ///addition operator for PointXYZRGBAA
 //   // template <> inline pcl::PointXYZRGBA
 //   // operator+ (const pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
 //   // {
 //   //   pcl::PointXYZRGBA result;
 //   //   result.getVector3fMap () = lhs.getVector3fMap ();
 //   //   result.getVector3fMap () += rhs.getVector3fMap ();
 //   //   result.r = static_cast<uint8_t> (lhs.r + rhs.r);
 //   //   result.g = static_cast<uint8_t> (lhs.g + rhs.g);
 //   //   result.b = static_cast<uint8_t> (lhs.b + rhs.b);
 //   //   return (result);
 //   // }
 //   // ///subtraction operator for PointXYZRGBA
 //   // template <> inline pcl::PointXYZRGBA
 //   // operator- (const pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
 //   // {
 //   //   pcl::PointXYZRGBA result;
 //   //   result.getVector3fMap () = lhs.getVector3fMap ();
 //   //   result.getVector3fMap () -= rhs.getVector3fMap ();
 //   //   result.r = static_cast<uint8_t> (lhs.r - rhs.r);
 //   //   result.g = static_cast<uint8_t> (lhs.g - rhs.g);
 //   //   result.b = static_cast<uint8_t> (lhs.b - rhs.b);
 //   //   return (result);
 //   // }
 //
 //   // template <> inline pcl::PointXYZRGBA
 //   // operator* (const float& scalar, const pcl::PointXYZRGBA& p)
 //   // {
 //   //   pcl::PointXYZRGBA result;
 //   //   result.getVector3fMap () = p.getVector3fMap ();
 //   //   result.getVector3fMap () *= scalar;
 //   //   result.r = static_cast<uint8_t> (scalar * p.r);
 //   //   result.g = static_cast<uint8_t> (scalar * p.g);
 //   //   result.b = static_cast<uint8_t> (scalar * p.b);
 //   //   return (result);
 //   // }
 //
 //   // template <> inline pcl::PointXYZRGBA
 //   // operator* (const pcl::PointXYZRGBA& p, const float& scalar)
 //   // {
 //   //   pcl::PointXYZRGBA result;
 //   //   result.getVector3fMap () = p.getVector3fMap ();
 //   //   result.getVector3fMap () *= scalar;
 //   //   result.r = static_cast<uint8_t> (scalar * p.r);
 //   //   result.g = static_cast<uint8_t> (scalar * p.g);
 //   //   result.b = static_cast<uint8_t> (scalar * p.b);
 //   //   return (result);
 //   // }
 //
 //   // template <> inline pcl::PointXYZRGBA&
 //   // operator+= (pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
 //   // {
 //   //   lhs.getVector3fMap () += rhs.getVector3fMap ();
 //   //   lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
 //   //   lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
 //   //   lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
 //   //   return (lhs);
 //   // }
 //
 //   // template <> inline pcl::PointXYZRGBA&
 //   // operator-= (pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
 //   // {
 //   //   lhs.getVector3fMap () -= rhs.getVector3fMap ();
 //   //   lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
 //   //   lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
 //   //   lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
 //   //   return (lhs);
 //   // }
 //
 //   // template <> inline pcl::PointXYZRGBA&
 //   // operator*= (pcl::PointXYZRGBA& lhs, const float& scalar)
 //   // {
 //   //   lhs.getVector3fMap () *= scalar;
 //   //   lhs.r = static_cast<uint8_t> (lhs.r * scalar);
 //   //   lhs.g = static_cast<uint8_t> (lhs.g * scalar);
 //   //   lhs.b = static_cast<uint8_t> (lhs.b * scalar);
 //   //   return (lhs);
 //   // }
 //
 //   // ///addition operator for RGBA
 //   // template <> inline pcl::RGB
 //   // operator+ (const pcl::RGB& lhs, const pcl::RGB& rhs)
 //   // {
 //   //   pcl::RGB result;
 //   //   result.r = static_cast<uint8_t> (lhs.r + rhs.r);
 //   //   result.g = static_cast<uint8_t> (lhs.g + rhs.g);
 //   //   result.b = static_cast<uint8_t> (lhs.b + rhs.b);
 //   //   return (result);
 //   // }
 //   // ///subtraction operator for RGB
 //   // template <> inline pcl::RGB
 //   // operator- (const pcl::RGB& lhs, const pcl::RGB& rhs)
 //   // {
 //   //   pcl::RGB result;
 //   //   result.r = static_cast<uint8_t> (lhs.r - rhs.r);
 //   //   result.g = static_cast<uint8_t> (lhs.g - rhs.g);
 //   //   result.b = static_cast<uint8_t> (lhs.b - rhs.b);
 //   //   return (result);
 //   // }
 //
 //   // template <> inline pcl::RGB
 //   // operator* (const float& scalar, const pcl::RGB& p)
 //   // {
 //   //   pcl::RGB result;
 //   //   result.r = static_cast<uint8_t> (scalar * p.r);
 //   //   result.g = static_cast<uint8_t> (scalar * p.g);
 //   //   result.b = static_cast<uint8_t> (scalar * p.b);
 //   //   return (result);
 //   // }
 //
 //   // template <> inline pcl::RGB
 //   // operator* (const pcl::RGB& p, const float& scalar)
 //   // {
 //   //   pcl::RGB result;
 //   //   result.r = static_cast<uint8_t> (scalar * p.r);
 //   //   result.g = static_cast<uint8_t> (scalar * p.g);
 //   //   result.b = static_cast<uint8_t> (scalar * p.b);
 //   //   return (result);
 //   // }
 //
 //   // template <> inline pcl::RGB&
 //   // operator+= (pcl::RGB& lhs, const pcl::RGB& rhs)
 //   // {
 //   //   lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
 //   //   lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
 //   //   lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
 //   //   return (lhs);
 //   // }
 //
 //   // template <> inline pcl::RGB&
 //   // operator-= (pcl::RGB& lhs, const pcl::RGB& rhs)
 //   // {
 //   //   lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
 //   //   lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
 //   //   lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
 //   //   return (lhs);
 //   // }
 //
 //   // template <> inline pcl::RGB&
 //   // operator*= (pcl::RGB& lhs, const float& scalar)
 //   // {
 //   //   lhs.r = static_cast<uint8_t> (lhs.r * scalar);
 //   //   lhs.g = static_cast<uint8_t> (lhs.g * scalar);
 //   //   lhs.b = static_cast<uint8_t> (lhs.b * scalar);
 //   //   return (lhs);
 //   // }
  }
}

#endif
