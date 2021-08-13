/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#pragma once

#include <pcl/common/intensity.h>
#include <pcl/point_types.h>


namespace pcl
{
  namespace common
  {
    template<>
    struct IntensityFieldAccessor<pcl::PointNormal>
    {
      inline float
      operator () (const pcl::PointNormal &p) const
      {
        return (p.curvature);
      }

      inline void
      get (const pcl::PointNormal &p, float &intensity) const
      {
        intensity = p.curvature;
      }

      inline void
      set (pcl::PointNormal &p, float intensity) const
      {
        p.curvature = intensity;
      }

      inline void
      demean (pcl::PointNormal& p, float value) const
      {
        p.curvature -= value;
      }

      inline void
      add (pcl::PointNormal& p, float value) const
      {
        p.curvature += value;
      }
    };
    
    template<>
    struct IntensityFieldAccessor<pcl::PointXYZ>
    {
      inline float
      operator () (const pcl::PointXYZ &p) const
      {
        return (p.z);
      }

      inline void
      get (const pcl::PointXYZ &p, float &intensity) const
      {
        intensity = p.z;
      }

      inline void
      set (pcl::PointXYZ &p, float intensity) const
      {
        p.z = intensity;
      }

      inline void
      demean (pcl::PointXYZ& p, float value) const
      {
        p.z -= value;
      }

      inline void
      add (pcl::PointXYZ& p, float value) const
      {
        p.z += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZRGB>
    {
      inline float
      operator () (const pcl::PointXYZRGB &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }

      inline void
      get (const pcl::PointXYZRGB &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }
      
      inline void
      set (pcl::PointXYZRGB &p, float intensity) const
      {
        p.r = static_cast<std::uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<std::uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<std::uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl::PointXYZRGB& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl::PointXYZRGB& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZRGBA>
    {
      inline float
      operator () (const pcl::PointXYZRGBA &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }
      
      inline void
      get (const pcl::PointXYZRGBA &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }

      inline void
      set (pcl::PointXYZRGBA &p, float intensity) const
      {
        p.r = static_cast<std::uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<std::uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<std::uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl::PointXYZRGBA& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl::PointXYZRGBA& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZRGBNormal>
    {
      inline float
      operator () (const pcl::PointXYZRGBNormal &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }
      
      inline void
      get (const pcl::PointXYZRGBNormal &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }

      inline void
      set (pcl::PointXYZRGBNormal &p, float intensity) const
      {
        p.r = static_cast<std::uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<std::uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<std::uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl::PointXYZRGBNormal &p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl::PointXYZRGBNormal &p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZRGBL>
    {
      inline float
      operator () (const pcl::PointXYZRGBL &p) const
      {
        return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f);
      }

      inline void
      get (const pcl::PointXYZRGBL &p, float& intensity) const
      {
        intensity = static_cast<float> (299*p.r + 587*p.g + 114*p.b) * 0.001f;
      }
      
      inline void
      set (pcl::PointXYZRGBL &p, float intensity) const
      {
        p.r = static_cast<std::uint8_t> (intensity * 3.34448160535f); // 1000 / 299
        p.g = static_cast<std::uint8_t> (intensity * 1.70357751278f); // 1000 / 587
        p.b = static_cast<std::uint8_t> (intensity * 8.77192982456f); // 1000 / 114
      }
      
      inline void
      demean (pcl::PointXYZRGBL& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity -= value;
        set (p, intensity);
      }
      
      inline void
      add (pcl::PointXYZRGBL& p, float value) const
      {
        float intensity = this->operator () (p);
        intensity += value;
        set (p, intensity);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZLAB>
    {
      inline float
      operator () (const pcl::PointXYZLAB &p) const
      {
        return (p.L);
      }

      inline void
      get (const pcl::PointXYZLAB &p, float &intensity) const
      {
        intensity = p.L;
      }

      inline void
      set (pcl::PointXYZLAB &p, float intensity) const
      {
        p.L = intensity;
      }

      inline void
      demean (pcl::PointXYZLAB& p, float value) const
      {
        p.L -= value;
      }

      inline void
      add (pcl::PointXYZLAB& p, float value) const
      {
        p.L += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZHSV>
    {
      inline float
      operator () (const pcl::PointXYZHSV &p) const
      {
        return (p.v);
      }

      inline void
      get (const pcl::PointXYZHSV &p, float &intensity) const
      {
        intensity = p.v;
      }

      inline void
      set (pcl::PointXYZHSV &p, float intensity) const
      {
        p.v = intensity;
        p.s = 0.0f;
      }

      inline void
      demean (pcl::PointXYZHSV& p, float value) const
      {
        p.v -= value;
      }

      inline void
      add (pcl::PointXYZHSV& p, float value) const
      {
        p.v += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZL>
    {
      inline float
      operator () (const pcl::PointXYZL &p) const
      {
        return (static_cast<float>(p.label));
      }

      inline void
      get (const pcl::PointXYZL &p, float &intensity) const
      {
        intensity = static_cast<float>(p.label);
      }

      inline void
      set (pcl::PointXYZL &p, float intensity) const
      {
        p.label = static_cast<std::uint32_t>(intensity);
        
      }

      inline void
      demean (pcl::PointXYZL& p, float value) const
      {
        p.label -= static_cast<std::uint32_t>(value);
      }

      inline void
      add (pcl::PointXYZL& p, float value) const
      {
        p.label += static_cast<std::uint32_t>(value);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointXYZLNormal>
    {
      inline float
      operator () (const pcl::PointXYZLNormal &p) const
      {
        return (static_cast<float>(p.label));
      }

      inline void
      get (const pcl::PointXYZLNormal &p, float &intensity) const
      {
        intensity = static_cast<float>(p.label);
      }

      inline void
      set (pcl::PointXYZLNormal &p, float intensity) const
      {
        p.label = static_cast<std::uint32_t>(intensity);
        
      }

      inline void
      demean (pcl::PointXYZLNormal& p, float value) const
      {
        p.label -= static_cast<std::uint32_t>(value);
      }

      inline void
      add (pcl::PointXYZLNormal& p, float value) const
      {
        p.label += static_cast<std::uint32_t>(value);
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::InterestPoint>
    {
      inline float
      operator () (const pcl::InterestPoint &p) const
      {
        return (p.strength);
      }

      inline void
      get (const pcl::InterestPoint &p, float &intensity) const
      {
        intensity = p.strength;
      }

      inline void
      set (pcl::InterestPoint &p, float intensity) const
      {
        p.strength = intensity;
      }

      inline void
      demean (pcl::InterestPoint& p, float value) const
      {
        p.strength -= value;
      }

      inline void
      add (pcl::InterestPoint& p, float value) const
      {
        p.strength += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointWithRange>
    {
      inline float
      operator () (const pcl::PointWithRange &p) const
      {
        return (p.range);
      }

      inline void
      get (const pcl::PointWithRange &p, float &intensity) const
      {
        intensity = p.range;
      }

      inline void
      set (pcl::PointWithRange &p, float intensity) const
      {
        p.range = intensity;
      }

      inline void
      demean (pcl::PointWithRange& p, float value) const
      {
        p.range -= value;
      }

      inline void
      add (pcl::PointWithRange& p, float value) const
      {
        p.range += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointWithScale>
    {
      inline float
      operator () (const pcl::PointWithScale &p) const
      {
        return (p.scale);
      }

      inline void
      get (const pcl::PointWithScale &p, float &intensity) const
      {
        intensity = p.scale;
      }

      inline void
      set (pcl::PointWithScale &p, float intensity) const
      {
        p.scale = intensity;
      }

      inline void
      demean (pcl::PointWithScale& p, float value) const
      {
        p.scale -= value;
      }

      inline void
      add (pcl::PointWithScale& p, float value) const
      {
        p.scale += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointWithViewpoint>
    {
      inline float
      operator () (const pcl::PointWithViewpoint &p) const
      {
        return (p.z);
      }

      inline void
      get (const pcl::PointWithViewpoint &p, float &intensity) const
      {
        intensity = p.z;
      }

      inline void
      set (pcl::PointWithViewpoint &p, float intensity) const
      {
        p.z = intensity;
      }

      inline void
      demean (pcl::PointWithViewpoint& p, float value) const
      {
        p.z -= value;
      }

      inline void
      add (pcl::PointWithViewpoint& p, float value) const
      {
        p.z += value;
      }
    };

    template<>
    struct IntensityFieldAccessor<pcl::PointSurfel>
    {
      inline float
      operator () (const pcl::PointSurfel &p) const
      {
        return (p.curvature);
      }

      inline void
      get (const pcl::PointSurfel &p, float &intensity) const
      {
        intensity = p.curvature;
      }

      inline void
      set (pcl::PointSurfel &p, float intensity) const
      {
        p.curvature = intensity;
      }

      inline void
      demean (pcl::PointSurfel& p, float value) const
      {
        p.curvature -= value;
      }

      inline void
      add (pcl::PointSurfel& p, float value) const
      {
        p.curvature += value;
      }
    };
  }
}

