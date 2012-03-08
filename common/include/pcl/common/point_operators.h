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
#include <pcl/exceptions.h>

namespace pcl
{
  namespace common
  {
    /// abstract class to convert from PointIn to PointOut
    template <typename PointIn, typename PointOut>
    struct PointInToPointOut 
    {
      inline PointOut operator() (const float& value = 0) 
      {
        PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                             "no suitable implementation available");
      }

      inline PointOut operator() (const PointIn& p)
      {
        PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                             "no suitable implementation available");
      }
    };

    /// extracts intensity from PointXYZI
    template <> 
    struct PointInToPointOut <pcl::PointXYZI, float>
    {
      typedef pcl::PointXYZI PointIn;
      typedef float PointOut;
      
      inline float 
      operator() (const float& val = 0) { return (val); }
      
      inline float 
      operator() (const pcl::PointXYZI& p) { return (p.intensity); }
    };
    
    /// extracts intensity from PointXYZRGB
    template<>
    struct PointInToPointOut <pcl::PointXYZRGB, float>
    {
      typedef pcl::PointXYZRGB PointIn;
      typedef float PointOut;

      inline float 
      operator() (const float& val = 0) { return (val); }
      
      inline float 
      operator() (const pcl::PointXYZRGB& p) { return (static_cast<float> (299*p.r + 587*p.g + 114*p.b)/1000.0f); }
    };

    /// Converts a PointXYZRGB to PointXYZI
    template<>
    struct PointInToPointOut <pcl::PointXYZRGB, pcl::PointXYZI>
    {
      typedef pcl::PointXYZRGB PointIn;
      typedef pcl::PointXYZI PointOut;

      inline PointXYZI
      operator() (const float& val = 0)
      { 
        pcl::PointXYZI zero;
        zero.x = val; zero.y = val; zero.z = val; zero.intensity = val;
        return (zero);
      }
      
      inline PointXYZI
      operator() (const pcl::PointXYZRGB& p)
      {
        pcl::PointXYZI result;
        result.x = p.x; result.y = p.y; result.z = p.z;
        result.intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b)/1000.0f;
        return (result);
      }
    };

    ///addition operator for PointT
    template <typename PointT> inline PointT 
    operator+ (const PointT& lhs, const PointT& rhs) 
    {
      PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                           "no suitable implementation available");
      return (PointT ());    
    }
    ///substraction operator for PointT
    template <typename PointT> inline PointT 
    operator- (const PointT& lhs, const PointT& rhs) 
    {
      PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                           "no suitable implementation available");
      return (PointT ());
    }
    ///multiplication operator for PointT and a scalar
    template <typename PointT> inline PointT 
    operator* (const float& scalar, const PointT& p) 
    {
      PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                           "no suitable implementation available");
      return (PointT ());
    }
    ///multiplication operator for PointT and a scalar
    template <typename PointT> inline PointT 
    operator* (const PointT& p, const float& scalar)
    {
      PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                           "no suitable implementation available");
      return (PointT ());
    }
    ///plus assign operator for PointT
    template <typename PointT> inline PointT&
    operator+= (PointT& lhs, const PointT& rhs)
    {
      PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                           "no suitable implementation available");
      return (lhs);
    }
    ///minus assign operator for PointT
    template <typename PointT> inline PointT&
    operator-= (PointT& lhs, const PointT& rhs)
    {
      PCL_THROW_EXCEPTION (UnhandledPointTypeException,
                           "no suitable implementation available");
      return (lhs);
    }
    ///addition operator for PointXYZI
    template <> inline pcl::PointXYZI 
    operator+ (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs) 
    {
      pcl::PointXYZI result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () += rhs.getVector3fMap ();
      result.intensity = lhs.intensity + rhs.intensity;
      return (result);
    }
    ///substraction operator for PointXYZI
    template <> inline pcl::PointXYZI 
    operator- (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs) 
    {
      pcl::PointXYZI result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () -= rhs.getVector3fMap ();
      result.intensity = lhs.intensity - rhs.intensity;
      return (result);
    }
    ///multiplication operator for PointXYZI and a scalar
    template <> inline pcl::PointXYZI 
    operator* (const float& scalar, const pcl::PointXYZI& p) 
    {
      pcl::PointXYZI result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.intensity = scalar * p.intensity;
      return (result);
    }
    ///multiplication operator for PointXYZI and a scalar
    template <> inline pcl::PointXYZI 
    operator* (const pcl::PointXYZI& p, const float& scalar)
    {
      pcl::PointXYZI result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.intensity = scalar * p.intensity;
      return (result);
    }
    ///plus assign operator for PointXYZI
    template <> inline pcl::PointXYZI&
    operator+= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      lhs.intensity+= rhs.intensity;
      return (lhs);
    }
    ///minus assign operator for PointXYZI
    template <> inline pcl::PointXYZI&
    operator-= (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      lhs.intensity-= rhs.intensity;
      return (lhs);
    }
    ///addition operator for PointXYZRGB
    template <> inline pcl::PointXYZRGB
    operator+ (const pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs) 
    {
      pcl::PointXYZRGB result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () += rhs.getVector3fMap ();
      result.r = static_cast<uint8_t> (lhs.r + rhs.r); 
      result.g = static_cast<uint8_t> (lhs.g + rhs.g);
      result.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (result);
    }
    ///substraction operator for PointXYZRGB
    template <> inline pcl::PointXYZRGB
    operator- (const pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs) 
    {
      pcl::PointXYZRGB result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () -= rhs.getVector3fMap ();
      result.r = static_cast<uint8_t> (lhs.r - rhs.r); 
      result.g = static_cast<uint8_t> (lhs.g - rhs.g); 
      result.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (result);
    }

    template <> inline pcl::PointXYZRGB
    operator* (const float& scalar, const pcl::PointXYZRGB& p)
    {
      pcl::PointXYZRGB result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.r = static_cast<uint8_t> (scalar * p.r); 
      result.g = static_cast<uint8_t> (scalar * p.g); 
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl::PointXYZRGB
    operator* (const pcl::PointXYZRGB& p, const float& scalar)
    {
      pcl::PointXYZRGB result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.r = static_cast<uint8_t> (scalar * p.r); 
      result.g = static_cast<uint8_t> (scalar * p.g); 
      result.b = static_cast<uint8_t> (scalar * p.b);
      return (result);
    }

    template <> inline pcl::PointXYZRGB&
    operator+= (pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      lhs.r = static_cast<uint8_t> (lhs.r + rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g + rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b + rhs.b);
      return (lhs);
    }

    template <> inline pcl::PointXYZRGB&
    operator-= (pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      lhs.r = static_cast<uint8_t> (lhs.r - rhs.r);
      lhs.g = static_cast<uint8_t> (lhs.g - rhs.g);
      lhs.b = static_cast<uint8_t> (lhs.b - rhs.b);
      return (lhs);
    }

    template <> inline pcl::PointXYZINormal
    operator+ (const pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs) 
    {
      pcl::PointXYZINormal result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () += rhs.getVector3fMap ();
      result.getNormalVector3fMap () = lhs.getNormalVector3fMap ();
      result.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      result.intensity = lhs.intensity + rhs.intensity;
      result.curvature = lhs.curvature + rhs.curvature;
      return (result);
    }

    template <> inline pcl::PointXYZINormal 
    operator- (const pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs) 
    {
      pcl::PointXYZINormal result;
      result.getVector3fMap () = lhs.getVector3fMap ();
      result.getVector3fMap () -= rhs.getVector3fMap ();
      result.getNormalVector3fMap () = lhs.getNormalVector3fMap ();
      result.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      result.intensity = lhs.intensity - rhs.intensity;
      result.curvature = lhs.curvature - rhs.curvature;
      return (result);
    }

    template <> inline pcl::PointXYZINormal&
    operator+= (pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs)
    {
      lhs.getVector3fMap () += rhs.getVector3fMap ();
      lhs.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      lhs.intensity+= rhs.intensity;
      lhs.curvature+= rhs.curvature;
      return (lhs);
    }

    template <> inline pcl::PointXYZINormal&
    operator-= (pcl::PointXYZINormal& lhs, const pcl::PointXYZINormal& rhs)
    {
      lhs.getVector3fMap () -= rhs.getVector3fMap ();
      lhs.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      lhs.intensity-= rhs.intensity;
      lhs.curvature-= rhs.curvature;
      return (lhs);
    }

    template <> inline pcl::PointXYZINormal 
    operator* (const float& scalar, const pcl::PointXYZINormal& p) 
    {
      pcl::PointXYZINormal result;
      result.getVector3fMap () = p.getVector3fMap ();
      result.getVector3fMap () *= scalar;
      result.getNormalVector3fMap () = p.getNormalVector3fMap ();
      result.getNormalVector3fMap () *= scalar;
      result.intensity = scalar * p.intensity;
      result.curvature = scalar * p.curvature;
      return (result);
    }

    template <> inline pcl::PointXYZINormal 
    operator* (const pcl::PointXYZINormal& p, const float& scalar)
    {
      return ( operator* (scalar, p));
    }

    ///addition operator for Normal
    template <> inline pcl::Normal 
    operator+ (const pcl::Normal& lhs, const pcl::Normal& rhs) 
    {
      pcl::Normal result;
      result.getNormalVector3fMap () = lhs.getNormalVector3fMap ();
      result.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      result.curvature = lhs.curvature + rhs.curvature;
      return (result);
    }
    ///substraction operator for Normal
    template <> inline pcl::Normal 
    operator- (const pcl::Normal& lhs, const pcl::Normal& rhs) 
    {
      pcl::Normal result;
      result.getNormalVector3fMap () = lhs.getNormalVector3fMap ();
      result.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      result.curvature = lhs.curvature - rhs.curvature;
      return (result);
    }
    ///multiplication operator for Normal and a scalar
    template <> inline pcl::Normal 
    operator* (const float& scalar, const pcl::Normal& p) 
    {
      pcl::Normal result;
      result.getNormalVector3fMap () = p.getNormalVector3fMap ();
      result.getNormalVector3fMap () *= scalar;
      result.curvature = scalar * p.curvature;
      return (result);
    }
    ///multiplication operator for Normal and a scalar
    template <> inline pcl::Normal 
    operator* (const pcl::Normal& p, const float& scalar)
    {
      pcl::Normal result;
      result.getNormalVector3fMap () = p.getNormalVector3fMap ();
      result.getNormalVector3fMap () *= scalar;
      result.curvature = scalar * p.curvature;
      return (result);
    }
    ///plus assign operator for Normal
    template <> inline pcl::Normal&
    operator+= (pcl::Normal& lhs, const pcl::Normal& rhs)
    {
      lhs.getNormalVector3fMap () += rhs.getNormalVector3fMap ();
      lhs.curvature+= rhs.curvature;
      return (lhs);
    }
    ///minus assign operator for Normal
    template <> inline pcl::Normal&
    operator-= (pcl::Normal& lhs, const pcl::Normal& rhs)
    {
      lhs.getNormalVector3fMap () -= rhs.getNormalVector3fMap ();
      lhs.curvature-= rhs.curvature;
      return (lhs);
    }

  }
}

#endif
