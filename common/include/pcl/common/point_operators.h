/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
    //////////////////////////////////////////////////////////////////////////////////////
    // +++
    template <typename PointT> inline PointT
    operator+ (const PointT& lhs, const PointT& rhs)
    {
      PointT result = lhs;
      result.x += rhs.x; result.y += rhs.y; result.z += rhs.z;
      return (result);
    }
    template <typename PointT> inline PointT
    operator+ (const float& scalar, const PointT& p)
    {
      PointT result = p;
      result.x += scalar; result.y += scalar; result.z += scalar;
      return (result);
    }
    template <typename PointT> inline PointT
    operator+ (const PointT& p, const float& scalar)
    {
      PointT result = p;
      result.x += scalar; result.y += scalar; result.z += scalar;
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // ---
    template <typename PointT> inline PointT
    operator- (const PointT& lhs, const PointT& rhs)
    {
      PointT result = lhs;
      result.x -= rhs.x; result.y -= rhs.y; result.z -= rhs.z;
      return (result);
    }
    template <typename PointT> inline PointT
    operator- (const float& scalar, const PointT& p)
    {
      PointT result = p;
      result.x -= scalar; result.y -= scalar; result.z -= scalar;
      return (result);
    }
    template <typename PointT> inline PointT
    operator- (const PointT& p, const float& scalar)
    {
      PointT result = p;
      result.x -= scalar; result.y -= scalar; result.z -= scalar;
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // ***
    template <typename PointT> inline PointT
    operator* (const PointT& lhs, const PointT& rhs)
    {
      PointT result = lhs;
      lhs.x *= rhs.x; lhs.y *= rhs.y; lhs.z *= rhs.z;
      return (result);
    }
    template <typename PointT> inline PointT
    operator* (const float& scalar, const PointT& p)
    {
      PointT result = p;
      result.x *= scalar; result.y *= scalar; result.z *= scalar;
      return (result);
    }
    template <typename PointT> inline PointT
    operator* (const PointT& p, const float& scalar)
    {
      PointT result = p;
      result.x *= scalar; result.y *= scalar; result.z *= scalar;
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // ///
    template <typename PointT> inline PointT
    operator/ (const PointT& lhs, const PointT& rhs)
    {
      PointT result = lhs;
      lhs.x /= rhs.x; lhs.y /= rhs.y; lhs.z /= rhs.z;
      return (result);
    }
    template <typename PointT> inline PointT
    operator/ (const float& scalar, const PointT& p)
    {
      PointT result = p;
      result.x /= scalar; result.y /= scalar; result.z /= scalar;
      return (result);
    }
    template <typename PointT> inline PointT
    operator/ (const PointT& p, const float& scalar)
    {
      PointT result = p;
      result.x /= scalar; result.y /= scalar; result.z /= scalar;
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // +=
    template <typename PointT> inline PointT&
    operator+= (PointT& lhs, const PointT& rhs)
    {
      lhs.x += rhs.x; lhs.y += rhs.y; lhs.z += rhs.z;
      return (lhs);
    }
    template <typename PointT> inline PointT
    operator+= (PointT& p, const float& scalar)
    {
      p.x += scalar; p.y += scalar; p.z += scalar;
      return (p);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // -=
    template <typename PointT> inline PointT&
    operator-= (PointT& lhs, const PointT& rhs)
    {
      lhs.x -= rhs.x; lhs.y -= rhs.y; lhs.z -= rhs.z;
      return (lhs);
    }
    template <typename PointT> inline PointT
    operator-= (PointT& p, const float& scalar)
    {
      p.x -= scalar; p.y -= scalar; p.z -= scalar;
      return (p);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // *=
    template <typename PointT> inline PointT&
    operator*= (PointT& lhs, const PointT& rhs)
    {
      lhs.x *= rhs.x; lhs.y *= rhs.y; lhs.z *= rhs.z;
      return (lhs);
    }
    template <typename PointT> inline PointT&
    operator*= (PointT& p, const float& scalar)
    {
      p.x *= scalar; p.y *= scalar; p.z *= scalar;
      return (p);
    }

    template <typename PointT> inline PointT
    operator*= (const PointT& p, const float& scalar)
    {
      PointT result = p;
      result.x *= scalar; result.y *= scalar; result.z *= scalar;
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // /=
    template <typename PointT> inline PointT&
    operator/= (PointT& lhs, const PointT& rhs)
    {
      lhs.x /= rhs.x; lhs.y /= rhs.y; lhs.z /= rhs.z;
      return (lhs);
    }
    template <typename PointT> inline PointT&
    operator/= (PointT& p, const float& scalar)
    {
      p.x /= scalar; p.y /= scalar; p.z /= scalar;
      return (p);
    }
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    // --[ PointXYZ
      // +++
      template <> inline pcl::PointXYZ
      operator+ (const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        pcl::PointXYZ result (lhs.x, lhs.y, lhs.z);
        result.x += rhs.x; result.y += rhs.y; result.z += rhs.z;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator+ (const float& scalar, const pcl::PointXYZ& p)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x += scalar; result.y += scalar; result.z += scalar;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator+ (const pcl::PointXYZ& p, const float& scalar)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x += scalar; result.y += scalar; result.z += scalar;
        return (result);
      }

      // ---
      template <> inline pcl::PointXYZ
      operator- (const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        pcl::PointXYZ result (lhs.x, lhs.y, lhs.z);
        result.x -= rhs.x; result.y -= rhs.y; result.z -= rhs.z;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator- (const float& scalar, const pcl::PointXYZ& p)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x -= scalar; result.y -= scalar; result.z -= scalar;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator- (const pcl::PointXYZ& p, const float& scalar)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x -= scalar; result.y -= scalar; result.z -= scalar;
        return (result);
      }

      // ***
      template <> inline pcl::PointXYZ
      operator* (const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        pcl::PointXYZ result (lhs.x, lhs.y, lhs.z);
        result.x *= rhs.x; result.y *= rhs.y; result.z *= rhs.z;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator* (const float& scalar, const pcl::PointXYZ& p)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x *= scalar; result.y *= scalar; result.z *= scalar;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator* (const pcl::PointXYZ& p, const float& scalar)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x *= scalar; result.y *= scalar; result.z *= scalar;
        return (result);
      }

      // ///
      template <> inline pcl::PointXYZ
      operator/ (const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        pcl::PointXYZ result (lhs.x, lhs.y, lhs.z);
        result.x /= rhs.x; result.y /= rhs.y; result.z /= rhs.z;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator/ (const float& scalar, const pcl::PointXYZ& p)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x /= scalar; result.y /= scalar; result.z /= scalar;
        return (result);
      }
      template <> inline pcl::PointXYZ
      operator/ (const pcl::PointXYZ& p, const float& scalar)
      {
        pcl::PointXYZ result (p.x, p.y, p.z);
        result.x /= scalar; result.y /= scalar; result.z /= scalar;
        return (result);
      }

      // +=
      template <> inline pcl::PointXYZ&
      operator+= (pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        lhs.x += rhs.x; lhs.y += rhs.y; lhs.z += rhs.z;
        return (lhs);
      }
      template <> inline pcl::PointXYZ
      operator+= (pcl::PointXYZ& p, const float& scalar)
      {
        p.x += scalar; p.y += scalar; p.z += scalar;
        return (p);
      }

      // -=
      template <> inline pcl::PointXYZ&
      operator-= (pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        lhs.x -= rhs.x; lhs.y -= rhs.y; lhs.z -= rhs.z;
        return (lhs);
      }
      template <> inline pcl::PointXYZ
      operator-= (pcl::PointXYZ& p, const float& scalar)
      {
        p.x -= scalar; p.y -= scalar; p.z -= scalar;
        return (p);
      }

      // *=
      template <> inline pcl::PointXYZ&
      operator*= (pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        lhs.x *= rhs.x; lhs.y *= rhs.y; lhs.z *= rhs.z;
        return (lhs);
      }
      template <> inline pcl::PointXYZ&
      operator*= (pcl::PointXYZ& p, const float& scalar)
      {
        p.x *= scalar; p.y *= scalar; p.z *= scalar;
        return (p);
      }

      // /=
      template <> inline pcl::PointXYZ&
      operator/= (pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
      {
        lhs.x /= rhs.x; lhs.y /= rhs.y; lhs.z /= rhs.z;
        return (lhs);
      }
      template <> inline pcl::PointXYZ&
      operator/= (pcl::PointXYZ& p, const float& scalar)
      {
        p.x /= scalar; p.y /= scalar; p.z /= scalar;
        return (p);
      }
    // ]---
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    // --[ PointXYZI
      // +++
      template <> inline pcl::PointXYZI
      operator+ (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        pcl::PointXYZI result = lhs;
        result.x += rhs.x; result.y += rhs.y; result.z += rhs.z;
        result.intensity += rhs.intensity;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator+ (const float& scalar, const pcl::PointXYZI& p)
      {
        pcl::PointXYZI result = p;
        result.x += scalar; result.y += scalar; result.z += scalar;
        result.intensity += scalar;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator+ (const pcl::PointXYZI& p, const float& scalar)
      {
        pcl::PointXYZI result = p;
        result.x += scalar; result.y += scalar; result.z += scalar;
        result.intensity += scalar;
        return (result);
      }

      //////////////////////////////////////////////////////////////////////////////////////
      // ---
      template <> inline pcl::PointXYZI
      operator- (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        pcl::PointXYZI result = lhs;
        result.x -= rhs.x; result.y -= rhs.y; result.z -= rhs.z;
        result.intensity -= rhs.intensity;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator- (const float& scalar, const pcl::PointXYZI& p)
      {
        pcl::PointXYZI result = p;
        result.x -= scalar; result.y -= scalar; result.z -= scalar;
        result.intensity -= scalar;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator- (const pcl::PointXYZI& p, const float& scalar)
      {
        pcl::PointXYZI result = p;
        result.x -= scalar; result.y -= scalar; result.z -= scalar;
        result.intensity -= scalar;
        return (result);
      }

      //////////////////////////////////////////////////////////////////////////////////////
      // ***
      template <> inline pcl::PointXYZI
      operator* (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        pcl::PointXYZI result = lhs;
        result.x *= rhs.x; result.y *= rhs.y; result.z *= rhs.z;
        result.intensity *= rhs.intensity;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator* (const float& scalar, const pcl::PointXYZI& p)
      {
        pcl::PointXYZI result = p;
        result.x *= scalar; result.y *= scalar; result.z *= scalar;
        result.intensity *= scalar;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator* (const pcl::PointXYZI& p, const float& scalar)
      {
        pcl::PointXYZI result = p;
        result.x *= scalar; result.y *= scalar; result.z *= scalar;
        result.intensity *= scalar;
        return (result);
      }

      //////////////////////////////////////////////////////////////////////////////////////
      // ///
      template <> inline pcl::PointXYZI
      operator/ (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        pcl::PointXYZI result = lhs;
        result.x /= rhs.x; result.y /= rhs.y; result.z /= rhs.z;
        result.intensity /= rhs.intensity;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator/ (const float& scalar, const pcl::PointXYZI& p)
      {
        pcl::PointXYZI result = p;
        result.x /= scalar; result.y /= scalar; result.z /= scalar;
        result.intensity /= scalar;
        return (result);
      }
      template <> inline pcl::PointXYZI
      operator/ (const pcl::PointXYZI& p, const float& scalar)
      {
        pcl::PointXYZI result = p;
        result.x /= scalar; result.y /= scalar; result.z /= scalar;
        result.intensity /= scalar;
        return (result);
      }

      //////////////////////////////////////////////////////////////////////////////////////
      // +=
      template <> inline pcl::PointXYZI&
      operator+= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        lhs.x += rhs.x; lhs.y += rhs.y; lhs.z += rhs.z;
        lhs.intensity += rhs.intensity;
        return (lhs);
      }
      template <> inline pcl::PointXYZI
      operator+= (pcl::PointXYZI& p, const float& scalar)
      {
        p.x += scalar; p.y += scalar; p.z += scalar;
        p.intensity += scalar;
        return (p);
      }

      //////////////////////////////////////////////////////////////////////////////////////
      // -=
      template <> inline pcl::PointXYZI&
      operator-= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        lhs.x -= rhs.x; lhs.y -= rhs.y; lhs.z -= rhs.z;
        lhs.intensity -= rhs.intensity;
        return (lhs);
      }
      template <> inline pcl::PointXYZI
      operator-= (pcl::PointXYZI& p, const float& scalar)
      {
        p.x -= scalar; p.y -= scalar; p.z -= scalar;
        p.intensity -= scalar;
        return (p);
      }

      //////////////////////////////////////////////////////////////////////////////////////
      // *=
      template <> inline pcl::PointXYZI&
      operator*= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        lhs.x *= rhs.x; lhs.y *= rhs.y; lhs.z *= rhs.z;
        lhs.intensity *= rhs.intensity;
        return (lhs);
      }
      template <> inline pcl::PointXYZI&
      operator*= (pcl::PointXYZI& p, const float& scalar)
      {
        p.x *= scalar; p.y *= scalar; p.z *= scalar;
        p.intensity *= scalar;
        return (p);
      }

      //////////////////////////////////////////////////////////////////////////////////////
      // /=
      template <> inline pcl::PointXYZI&
      operator/= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        lhs.x /= rhs.x; lhs.y /= rhs.y; lhs.z /= rhs.z;
        lhs.intensity /= rhs.intensity;
        return (lhs);
      }
      template <> inline pcl::PointXYZI&
      operator/= (pcl::PointXYZI& p, const float& scalar)
      {
        p.x /= scalar; p.y /= scalar; p.z /= scalar;
        p.intensity /= scalar;
        return (p);
      }
    // ]---
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////


    ///addition operator for PointXYZI
   // template <> inline pcl::PointXYZI
   // operator+ (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
   // {
   //   pcl::PointXYZI result = lhs;
   //   result.getVector3fMap () += rhs.getVector3fMap ();
   //   result.intensity += rhs.intensity;
   //   return (result);
   // }
   // ///subtraction operator for PointXYZI
   // template <> inline pcl::PointXYZI
   // operator- (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
   // {
   //   pcl::PointXYZI result = lhs;
   //   result.getVector3fMap () -= rhs.getVector3fMap ();
   //   result.intensity -= rhs.intensity;
   //  return (result);
   // }
   // ///multiplication operator for PointXYZI and a scalar
   // template <> inline pcl::PointXYZI
   // operator* (const float& scalar, const pcl::PointXYZI& p)
   // {
   //   pcl::PointXYZI result = p;
   //   result.getVector3fMap () *= scalar;
   //   result.intensity *= scalar;
   //   return (result);
   // }
   // ///multiplication operator for PointXYZI and a scalar
   // template <> inline pcl::PointXYZI
   // operator* (const pcl::PointXYZI& p, const float& scalar)
   // {
   //   pcl::PointXYZI result = p;
   //   result.getVector3fMap () *= scalar;
   //   result.intensity *= scalar;
   //   return (result);
   // }
   // ///plus assign operator for PointXYZI
   // template <> inline pcl::PointXYZI&
   // operator+= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
   // {
   //   lhs.getVector3fMap () += rhs.getVector3fMap ();
   //   lhs.intensity += rhs.intensity;
   //   return (lhs);
   // }
   // ///minus assign operator for PointXYZI
   // template <> inline pcl::PointXYZI&
   // operator-= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
   // {
   //   lhs.getVector3fMap () -= rhs.getVector3fMap ();
   //   lhs.intensity -= rhs.intensity;
   //   return (lhs);
   // }
   // ///multiply assign operator for PointXYZI
   // template <> inline pcl::PointXYZI&
   // operator*= (pcl::PointXYZI& lhs, const float& scalar)
   // {
   //   lhs.getVector3fMap () *= scalar;
   //   lhs.intensity *= scalar;
   //   return (lhs);
   // }
   // template <> inline pcl::PointXYZINormal
   // operator+ (const pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs)
   // {
   //   pcl::PointXYZINormal result = lhs;
   //   result.getVector3fMap () += rhs.getVector3fMap ();
   //   result.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
   //   result.intensity += rhs.intensity;
   //   result.curvature += rhs.curvature;
   //   return (result);
   // }

   // template <> inline pcl::PointXYZINormal
   // operator- (const pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs)
   // {
   //   pcl::PointXYZINormal result = lhs;
   //   result.getVector3fMap () -= rhs.getVector3fMap ();
   //   result.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
   //   result.intensity -= rhs.intensity;
   //   result.curvature -= rhs.curvature;
   //   return (result);
   // }

   // template <> inline pcl::PointXYZINormal&
   // operator+= (pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs)
   // {
   //   lhs.getVector3fMap () += rhs.getVector3fMap ();
   //   lhs.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
   //   lhs.intensity += rhs.intensity;
   //   lhs.curvature += rhs.curvature;
   //   return (lhs);
   // }

   // template <> inline pcl::PointXYZINormal&
   // operator-= (pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs)
   // {
   //   lhs.getVector3fMap () -= rhs.getVector3fMap ();
   //   lhs.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
   //   lhs.intensity-= rhs.intensity;
   //   lhs.curvature-= rhs.curvature;
   //   return (lhs);
   // }

   // template <> inline pcl::PointXYZINormal&
   // operator*= (pcl::PointXYZINormal& lhs, const float& scalar)
   // {
   //   lhs.getVector3fMap () *= scalar;
   //   lhs.getNormalVector3fMap () *= scalar;
   //   lhs.intensity *= scalar;
   //   lhs.curvature *= scalar;
   //   return (lhs);
   // }

   // template <> inline pcl::PointXYZINormal
   // operator* (const float& scalar, const pcl::PointXYZINormal& p)
   // {
   //   pcl::PointXYZINormal result = p;
   //   result.getVector3fMap () *= scalar;
   //   result.getNormalVector3fMap () *= scalar;
   //   result.intensity *= scalar;
   //   result.curvature *= scalar;
   //   return (result);
   // }

   // template <> inline pcl::PointXYZINormal
   // operator* (const pcl::PointXYZINormal& p, const float& scalar)
   // {
   //   return (operator* (scalar, p));
   // }

   // // ---[ PointNormal
   //   template <> inline pcl::PointNormal
   //   operator+ (const pcl::PointNormal& lhs, const pcl::PointNormal& rhs)
   //   {
   //     pcl::PointNormal result = lhs;
   //     result.getVector3fMap () += rhs.getVector3fMap ();
   //     result.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
   //     result.curvature += rhs.curvature;
   //     return (result);
   //   }

   //   template <> inline pcl::PointNormal
   //   operator+ (const float& scalar, const pcl::PointNormal& p)
   //   {
   //     pcl::PointNormal result = p;
   //     result.x += scalar;
   //     result.y += scalar;
   //     result.z += scalar;
   //     result.normal_x += scalar;
   //     result.normal_y += scalar;
   //     result.normal_z += scalar;
   //     result.curvature += scalar;
   //     return (result);
   //   }

   //   template <> inline pcl::PointNormal
   //   operator+ (const pcl::PointNormal& p, const float& scalar)
   //   {
   //     pcl::PointNormal result = p;
   //     result.x += scalar;
   //     result.y += scalar;
   //     result.z += scalar;
   //     result.normal_x += scalar;
   //     result.normal_y += scalar;
   //     result.normal_z += scalar;
   //     result.curvature += scalar;
   //     return (result);
   //   }

   //   template <> inline pcl::PointNormal&
   //   operator+= (pcl::PointNormal& lhs, const pcl::PointNormal& rhs)
   //   {
   //     lhs.getVector3fMap () += rhs.getVector3fMap ();
   //     lhs.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
   //     lhs.curvature += rhs.curvature;
   //     return (lhs);
   //   }

   //   template <> inline pcl::PointNormal
   //   operator- (const pcl::PointNormal& lhs, const pcl::PointNormal& rhs)
   //   {
   //     pcl::PointNormal result = lhs;
   //     result.getVector3fMap () -= rhs.getVector3fMap ();
   //     result.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
   //     result.curvature -= rhs.curvature;
   //     return (result);
   //   }

   //   template <> inline pcl::PointNormal&
   //   operator-= (pcl::PointNormal& lhs, const pcl::PointNormal& rhs)
   //   {
   //     lhs.getVector3fMap () -= rhs.getVector3fMap ();
   //     lhs.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
   //     lhs.curvature-= rhs.curvature;
   //     return (lhs);
   //   }

   //   template <> inline pcl::PointNormal
   //   operator* (const float& scalar, const pcl::PointNormal& p)
   //   {
   //     pcl::PointNormal result = p;
   //     result.getVector3fMap () *= scalar;
   //     result.getNormalVector3fMap () *= scalar;
   //     result.curvature *= scalar;
   //     return (result);
   //   }

   //   template <> inline pcl::PointNormal
   //   operator* (const pcl::PointNormal& p, const float& scalar)
   //   {
   //     return (operator* (scalar, p));
   //   }

   //   template <> inline pcl::PointNormal&
   //   operator*= (pcl::PointNormal& p, const float& scalar)
   //   {
   //     p.getVector3fMap () *= scalar;
   //     p.getNormalVector3fMap () *= scalar;
   //     p.curvature *= scalar;
   //     return (p);
   //   }

   //   template <> inline pcl::PointNormal
   //   operator/ (const float& scalar, const pcl::PointNormal& p)
   //   {
   //     pcl::PointNormal result = p;
   //     result.getVector3fMap () /= scalar;
   //     result.getNormalVector3fMap () /= scalar;
   //     result.curvature /= scalar;
   //     return (result);
   //   }
   //   ///division operator for PointT and a scalar
   //   template <> inline pcl::PointNormal
   //   operator/ (const pcl::PointNormal& p, const float& scalar)
   //   {
   //     pcl::PointNormal result = p;
   //     result.getVector3fMap () /= scalar;
   //     result.getNormalVector3fMap () /= scalar;
   //     result.curvature /= scalar;
   //     return (result);
   //   }
   //   template <> inline pcl::PointNormal&
   //   operator/= (pcl::PointNormal& p, const float& scalar)
   //   {
   //     p.getVector3fMap () /= scalar;
   //     p.getNormalVector3fMap () /= scalar;
   //     p.curvature /= scalar;
   //     return (p);
   //   }
   // // ]---

   // ///addition operator for Normal
   // template <> inline pcl::Normal
   // operator+ (const pcl::Normal& lhs, const pcl::Normal& rhs)
   // {
   //   pcl::Normal result = lhs;
   //   result.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
   //   result.curvature += rhs.curvature;
   //   return (result);
   // }
   // ///subtraction operator for Normal
   // template <> inline pcl::Normal
   // operator- (const pcl::Normal& lhs, const pcl::Normal& rhs)
   // {
   //   pcl::Normal result = lhs;
   //   result.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
   //   result.curvature -= rhs.curvature;
   //   return (result);
   // }
   // ///multiplication operator for Normal and a scalar
   // template <> inline pcl::Normal
   // operator* (const float& scalar, const pcl::Normal& p)
   // {
   //   pcl::Normal result = p;
   //   result.getNormalVector3fMap () *= scalar;
   //   result.curvature *= scalar;
   //   return (result);
   // }
   // ///multiplication operator for Normal and a scalar
   // template <> inline pcl::Normal
   // operator* (const pcl::Normal& p, const float& scalar)
   // {
   //   pcl::Normal result = p;
   //   result.getNormalVector3fMap () *= scalar;
   //   result.curvature *= scalar;
   //   return (result);
   // }
   // ///plus assign operator for Normal
   // template <> inline pcl::Normal&
   // operator+= (pcl::Normal& lhs, const pcl::Normal& rhs)
   // {
   //   lhs.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
   //   lhs.curvature += rhs.curvature;
   //   return (lhs);
   // }
   // ///minus assign operator for Normal
   // template <> inline pcl::Normal&
   // operator-= (pcl::Normal& lhs, const pcl::Normal& rhs)
   // {
   //   lhs.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
   //   lhs.curvature -= rhs.curvature;
   //   return (lhs);
   // }
   // ///multiply assign operator for Normal
   // template <> inline pcl::Normal&
   // operator*= (pcl::Normal& lhs, const float& scalar)
   // {
   //   lhs.getNormalVector3fMap () *= scalar;
   //   lhs.curvature *= scalar;
   //   return (lhs);
   // }

   // ///addition operator for PointXYZRGB
   // template <> inline pcl::PointXYZRGB
   // operator+ (const pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
   // {
   //   pcl::PointXYZRGB result;
   //   result.getVector3fMap () = lhs.getVector3fMap ();
   //   result.getVector3fMap () += rhs.getVector3fMap ();
   //   result.r = static_cast<uint8_t> (lhs.r + rhs.r);
   //   result.g = static_cast<uint8_t> (lhs.g + rhs.g);
   //   result.b = static_cast<uint8_t> (lhs.b + rhs.b);
   //   return (result);
   // }
   // ///subtraction operator for PointXYZRGB
   // template <> inline pcl::PointXYZRGB
   // operator- (const pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
   // {
   //   pcl::PointXYZRGB result;
   //   result.getVector3fMap () = lhs.getVector3fMap ();
   //   result.getVector3fMap () -= rhs.getVector3fMap ();
   //   result.r = static_cast<uint8_t> (lhs.r - rhs.r);
   //   result.g = static_cast<uint8_t> (lhs.g - rhs.g);
   //   result.b = static_cast<uint8_t> (lhs.b - rhs.b);
   //   return (result);
   // }

   // template <> inline pcl::PointXYZRGB
   // operator* (const float& scalar, const pcl::PointXYZRGB& p)
   // {
   //   pcl::PointXYZRGB result;
   //   result.getVector3fMap () = p.getVector3fMap ();
   //   result.getVector3fMap () *= scalar;
   //   result.r = static_cast<uint8_t> (scalar * p.r);
   //   result.g = static_cast<uint8_t> (scalar * p.g);
   //   result.b = static_cast<uint8_t> (scalar * p.b);
   //   return (result);
   // }

   // template <> inline pcl::PointXYZRGB
   // operator* (const pcl::PointXYZRGB& p, const float& scalar)
   // {
   //   pcl::PointXYZRGB result;
   //   result.getVector3fMap () = p.getVector3fMap ();
   //   result.getVector3fMap () *= scalar;
   //   result.r = static_cast<uint8_t> (scalar * p.r);
   //   result.g = static_cast<uint8_t> (scalar * p.g);
   //   result.b = static_cast<uint8_t> (scalar * p.b);
   //   return (result);
   // }

   // template <> inline pcl::PointXYZRGB&
   // operator+= (pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
   // {
   //   lhs.getVector3fMap () += rhs.getVector3fMap ();
   //   lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
   //   lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
   //   lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
   //   return (lhs);
   // }

   // template <> inline pcl::PointXYZRGB&
   // operator-= (pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
   // {
   //   lhs.getVector3fMap () -= rhs.getVector3fMap ();
   //   lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
   //   lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
   //   lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
   //   return (lhs);
   // }

   // template <> inline pcl::PointXYZRGB&
   // operator*= (pcl::PointXYZRGB& lhs, const float& scalar)
   // {
   //   lhs.getVector3fMap () *= scalar;
   //   lhs.r = static_cast<uint8_t> (lhs.r * scalar);
   //   lhs.g = static_cast<uint8_t> (lhs.g * scalar);
   //   lhs.b = static_cast<uint8_t> (lhs.b * scalar);
   //   return (lhs);
   // }

   // ///addition operator for PointXYZRGBAA
   // template <> inline pcl::PointXYZRGBA
   // operator+ (const pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
   // {
   //   pcl::PointXYZRGBA result;
   //   result.getVector3fMap () = lhs.getVector3fMap ();
   //   result.getVector3fMap () += rhs.getVector3fMap ();
   //   result.r = static_cast<uint8_t> (lhs.r + rhs.r);
   //   result.g = static_cast<uint8_t> (lhs.g + rhs.g);
   //   result.b = static_cast<uint8_t> (lhs.b + rhs.b);
   //   return (result);
   // }
   // ///subtraction operator for PointXYZRGBA
   // template <> inline pcl::PointXYZRGBA
   // operator- (const pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
   // {
   //   pcl::PointXYZRGBA result;
   //   result.getVector3fMap () = lhs.getVector3fMap ();
   //   result.getVector3fMap () -= rhs.getVector3fMap ();
   //   result.r = static_cast<uint8_t> (lhs.r - rhs.r);
   //   result.g = static_cast<uint8_t> (lhs.g - rhs.g);
   //   result.b = static_cast<uint8_t> (lhs.b - rhs.b);
   //   return (result);
   // }

   // template <> inline pcl::PointXYZRGBA
   // operator* (const float& scalar, const pcl::PointXYZRGBA& p)
   // {
   //   pcl::PointXYZRGBA result;
   //   result.getVector3fMap () = p.getVector3fMap ();
   //   result.getVector3fMap () *= scalar;
   //   result.r = static_cast<uint8_t> (scalar * p.r);
   //   result.g = static_cast<uint8_t> (scalar * p.g);
   //   result.b = static_cast<uint8_t> (scalar * p.b);
   //   return (result);
   // }

   // template <> inline pcl::PointXYZRGBA
   // operator* (const pcl::PointXYZRGBA& p, const float& scalar)
   // {
   //   pcl::PointXYZRGBA result;
   //   result.getVector3fMap () = p.getVector3fMap ();
   //   result.getVector3fMap () *= scalar;
   //   result.r = static_cast<uint8_t> (scalar * p.r);
   //   result.g = static_cast<uint8_t> (scalar * p.g);
   //   result.b = static_cast<uint8_t> (scalar * p.b);
   //   return (result);
   // }

   // template <> inline pcl::PointXYZRGBA&
   // operator+= (pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
   // {
   //   lhs.getVector3fMap () += rhs.getVector3fMap ();
   //   lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
   //   lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
   //   lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
   //   return (lhs);
   // }

   // template <> inline pcl::PointXYZRGBA&
   // operator-= (pcl::PointXYZRGBA& lhs, const pcl::PointXYZRGBA& rhs)
   // {
   //   lhs.getVector3fMap () -= rhs.getVector3fMap ();
   //   lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
   //   lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
   //   lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
   //   return (lhs);
   // }

   // template <> inline pcl::PointXYZRGBA&
   // operator*= (pcl::PointXYZRGBA& lhs, const float& scalar)
   // {
   //   lhs.getVector3fMap () *= scalar;
   //   lhs.r = static_cast<uint8_t> (lhs.r * scalar);
   //   lhs.g = static_cast<uint8_t> (lhs.g * scalar);
   //   lhs.b = static_cast<uint8_t> (lhs.b * scalar);
   //   return (lhs);
   // }

   // ///addition operator for RGBA
   // template <> inline pcl::RGB
   // operator+ (const pcl::RGB& lhs, const pcl::RGB& rhs)
   // {
   //   pcl::RGB result;
   //   result.r = static_cast<uint8_t> (lhs.r + rhs.r);
   //   result.g = static_cast<uint8_t> (lhs.g + rhs.g);
   //   result.b = static_cast<uint8_t> (lhs.b + rhs.b);
   //   return (result);
   // }
   // ///subtraction operator for RGB
   // template <> inline pcl::RGB
   // operator- (const pcl::RGB& lhs, const pcl::RGB& rhs)
   // {
   //   pcl::RGB result;
   //   result.r = static_cast<uint8_t> (lhs.r - rhs.r);
   //   result.g = static_cast<uint8_t> (lhs.g - rhs.g);
   //   result.b = static_cast<uint8_t> (lhs.b - rhs.b);
   //   return (result);
   // }

   // template <> inline pcl::RGB
   // operator* (const float& scalar, const pcl::RGB& p)
   // {
   //   pcl::RGB result;
   //   result.r = static_cast<uint8_t> (scalar * p.r);
   //   result.g = static_cast<uint8_t> (scalar * p.g);
   //   result.b = static_cast<uint8_t> (scalar * p.b);
   //   return (result);
   // }

   // template <> inline pcl::RGB
   // operator* (const pcl::RGB& p, const float& scalar)
   // {
   //   pcl::RGB result;
   //   result.r = static_cast<uint8_t> (scalar * p.r);
   //   result.g = static_cast<uint8_t> (scalar * p.g);
   //   result.b = static_cast<uint8_t> (scalar * p.b);
   //   return (result);
   // }

   // template <> inline pcl::RGB&
   // operator+= (pcl::RGB& lhs, const pcl::RGB& rhs)
   // {
   //   lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
   //   lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
   //   lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
   //   return (lhs);
   // }

   // template <> inline pcl::RGB&
   // operator-= (pcl::RGB& lhs, const pcl::RGB& rhs)
   // {
   //   lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
   //   lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
   //   lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
   //   return (lhs);
   // }

   // template <> inline pcl::RGB&
   // operator*= (pcl::RGB& lhs, const float& scalar)
   // {
   //   lhs.r = static_cast<uint8_t> (lhs.r * scalar);
   //   lhs.g = static_cast<uint8_t> (lhs.g * scalar);
   //   lhs.b = static_cast<uint8_t> (lhs.b * scalar);
   //   return (lhs);
   // }
  }
}

#endif
