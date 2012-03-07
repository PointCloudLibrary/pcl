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
 * $Id: point_operators.h 3657 2011-12-26 23:11:38Z nizar $
 *
 */

#ifndef PCL_COMMON_POINT_OPERATORS_H
#define PCL_COMMON_POINT_OPERATORS_H

#include <pcl/point_types.h>

namespace pcl
{
  namespace common
  {

    /** PointOperators is a struct that provides basic arithmetic 
      * operations on points: addition, product and plus-assign operation.
      * It also provide an operator() for the transformation from a 
      * PointIN to PointOUT (the transformation can be as simple as 
      * accessing a member of PointIN).
      *
      * \author Nizar Sallem
      * \ingroup common
      */
    template <typename PointIN, typename PointOUT>
    struct PointOperators 
    {
      typedef PointIN PointIn;
      typedef PointOUT PointOut;
    };
    
    struct PointXYZItoIntensity : PointOperators <pcl::PointXYZI, float>
    {
      float
      operator () ()
      {
        return 0;
      }

      float
      operator () (pcl::PointXYZI& p)
      {
        return (p.intensity);
      }

      float 
      add (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        return (lhs.intensity + rhs.intensity);
      }

      float 
      dot (const float& scalar, const pcl::PointXYZI& p)
      {
        return (scalar * p.intensity);
      }

      float 
      dot (const pcl::PointXYZI& p, const float& scalar)
      {
        return dot (scalar, p);
      }
    };

    struct PointXYZRGBtoIntensity : PointOperators <pcl::PointXYZRGB, float>
    {
      float
      operator () ()
      {
        return 0;
      }

      float
      operator () (pcl::PointXYZRGB& p)
      {
        return ((299*p.r + 587*p.g + 114*p.b)/1000.0f);
      }

      float 
      add (const pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
      {
        return ((299*(lhs.r + rhs.r) + 587*(lhs.g + rhs.g) + 114*(lhs.b + rhs.b))/1000.0);
      }

      float 
      dot (const float& scalar, const pcl::PointXYZRGB& p)
      {
        return (scalar * (299*p.r + 587*p.g + 114*p.b)/1000.0);
      }
      
      float 
      dot (const pcl::PointXYZRGB& p, const float& scalar)
      {
        return (dot (p, scalar));
      }
    };

    struct PointXYZItoPointXYZI : PointOperators <pcl::PointXYZI, pcl::PointXYZI>
    {
      pcl::PointXYZI
      operator () ()
      {
        pcl::PointXYZI zero;
        zero.x = 0; zero.y = 0; zero.z = 0; zero.intensity = 0;
        return zero;
      }

      const pcl::PointXYZI&
      operator () (const pcl::PointXYZI& p)
      {
        return (p);
      }

      pcl::PointXYZI&
      operator () (pcl::PointXYZI& p)
      {
        return (p);
      }

      // pcl::PointXYZI&
      // operator= (pcl::PointXYZI& p, int 0)
      // {
      //   p.x = p.y = p.z = p.intensity = 0;
      //   return (p);
      // }

      pcl::PointXYZI 
      add (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs) 
      {
        pcl::PointXYZI result;
        result.x = lhs.x + rhs.x;
        result.y = lhs.y + rhs.y;
        result.z = lhs.z + rhs.z;
        result.intensity = lhs.intensity + rhs.intensity;
        return (result);
      }


      pcl::PointXYZI 
      minus (const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs) 
      {
        pcl::PointXYZI result;
        result.x = lhs.x - rhs.x;
        result.y = lhs.y - rhs.y;
        result.z = lhs.z - rhs.z;
        result.intensity = lhs.intensity - rhs.intensity;
        return (result);
      }

      pcl::PointXYZI 
      dot (const float& scalar, const pcl::PointXYZI& p) 
      {
        pcl::PointXYZI result;
        result.x = scalar * p.x;
        result.y = scalar * p.y;
        result.z = scalar * p.z;
        result.intensity = scalar * p.intensity;
        return (result);
      }

      pcl::PointXYZI 
      dot (const pcl::PointXYZI& p, const float& scalar)
      {
        return (dot (scalar, p));
      }

      pcl::PointXYZI&
      plus_assign (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        lhs.x+= rhs.x; lhs.y+= rhs.y; lhs.z+= rhs.z; lhs.intensity+= rhs.intensity;
        return (lhs);
      }
    };

    struct PointXYZRGBtoPointXYZRGB : PointOperators <pcl::PointXYZRGB, pcl::PointXYZRGB>
    {
      pcl::PointXYZRGB
      operator () ()
      {
        pcl::PointXYZRGB zero;
        zero.x = 0; zero.y = 0; zero.z = 0; zero.r = 0; zero.g = 0; zero.b = 0;
        return zero;
      }

      pcl::PointXYZRGB
      operator () (pcl::PointXYZRGB& p)
      {
        return (p);
      }

      pcl::PointXYZRGB 
      add (const pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs) 
      {
        pcl::PointXYZRGB result;
        result.x = lhs.x + rhs.x; result.y = lhs.y + rhs.y; result.z = lhs.z + rhs.z;
        result.r = lhs.r + rhs.r; result.g = lhs.g + rhs.g; result.b = lhs.b + rhs.b;
        return (result);
      }
            
      pcl::PointXYZRGB 
      dot (const float& scalar, const pcl::PointXYZRGB& p) 
      {
        pcl::PointXYZRGB result;
        result.x = scalar * p.x; result.y = scalar * p.y; result.z = scalar * p.z; 
        result.r = scalar * p.r; result.g = scalar * p.g; result.b = scalar * p.b;
        return (result);
      }

      pcl::PointXYZRGB&
      plus_assign (pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs)
      {
        lhs.x+= rhs.x; lhs.y+= rhs.y; lhs.z+= rhs.z; 
        lhs.r+= rhs.r; lhs.g+= rhs.g; lhs.b+= rhs.b;
        return (lhs);
      }
    };

    struct PointXYZRGBtoPointXYZI : PointOperators <pcl::PointXYZRGB, pcl::PointXYZI>
    {
      PointXYZI
      operator () (pcl::PointXYZRGB& p)
      {
        pcl::PointXYZI result;
        result.x = p.x; result.y = p.y; result.z = p.z;
        result.intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b)/1000.0f;
        return (result);
      }
      
      pcl::PointXYZI 
      add (const pcl::PointXYZRGB& lhs, const pcl::PointXYZRGB& rhs) 
      {
        pcl::PointXYZI result;
        result.x = lhs.x + rhs.x; result.y = lhs.y + rhs.y; result.z = lhs.z + rhs.z;
        result.intensity = (299*(lhs.r + rhs.r) + 587*(lhs.g + rhs.g) + 114*(lhs.b + rhs.b))/1000.0;
        return (result);
      }
      
      pcl::PointXYZI 
      dot (const float& scalar, const pcl::PointXYZRGB& p) 
      {
        pcl::PointXYZI result;
        result.x = scalar * p.x; result.y = scalar * p.y; result.z = scalar * p.z; 
        result.intensity = scalar * (299*p.r + 587*p.g + 114*p.b)/1000.0;
        return (result);
      }

      pcl::PointXYZI&
      plus_assign (pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
      {
        lhs.x+= rhs.x; lhs.y+= rhs.y; lhs.z+= rhs.z; lhs.intensity+= rhs.intensity;
        return (lhs);
      }
    };
  }
}

#endif
