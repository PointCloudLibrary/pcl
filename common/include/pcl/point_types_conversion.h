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
 */

#ifndef PCL_TYPE_CONVERSIONS_H
#define PCL_TYPE_CONVERSIONS_H

namespace pcl
{
  // r,g,b, i values are from 0 to 1
  // h = [0,360]
  // s, v values are from 0 to 1
  // if s = 0 > h = -1 (undefined)

  /** \brief Convert a XYZRGB point type to a XYZI
    * \param[in] in the input XYZRGB point 
    * \param[out] out the output XYZI point
    */
  inline void 
  PointXYZRGBtoXYZI (PointXYZRGB&  in,
                     PointXYZI&    out)
  {
    out.x = in.x; out.y = in.y; out.z = in.z;
    out.intensity = 0.299f * in.r + 0.587f * in.g + 0.114f * in.b;
  }

  
  /** \brief Convert a XYZRGB point type to a XYZHSV
    * \param[in] in the input XYZRGB point 
    * \param[out] out the output XYZHSV point
    */
  inline void 
  PointXYZRGBtoXYZHSV (PointXYZRGB& in,
                       PointXYZHSV& out)
  {
    float min;

    out.x = in.x; out.y = in.y; out.z = in.z;

    out.v = std::max (in.r, std::max (in.g, in.b));
    min = std::min (in.r, std::min (in.g, in.b));

    if (out.v != 0)
      out.s = (out.v - min) / out.v;
    else
    {
      out.s = 0;
      out.h = -1;
      return;
    }

    if (in.r == out.v)
      out.h = static_cast<float> (in.g - in.b) / (out.v - min);
    else if (in.g == out.v)
      out.h = static_cast<float> (2 + (in.b - in.r) / (out.v - min));
    else 
      out.h = static_cast<float> (4 + (in.r - in.g) / (out.v - min));
    out.h *= 60;
    if (out.h < 0)
      out.h += 360;
  }


  /** \brief Convert a XYZHSV point type to a XYZRGB
    * \param[in] in the input XYZHSV point 
    * \param[out] out the output XYZRGB point
    */
  inline void 
  PointXYZHSVtoXYZRGB (PointXYZHSV&  in,
                       PointXYZRGB&  out)
  {
    if (in.s == 0)
    {
      out.r = out.g = out.b = static_cast<uint8_t> (in.v);
      return;
    } 
    float a = in.h / 60;
    int   i = static_cast<int> (floorf (a));
    float f = a - static_cast<float> (i);
    float p = in.v * (1 - in.s);
    float q = in.v * (1 - in.s * f);
    float t = in.v * (1 - in.s * (1 - f));

    switch (i)
    {
      case 0:
      {
        out.r = static_cast<uint8_t> (255 * in.v);
        out.g = static_cast<uint8_t> (255 * t);
        out.b = static_cast<uint8_t> (255 * p);
        break;
      }
      case 1:
      {
        out.r = static_cast<uint8_t> (255 * q); 
        out.g = static_cast<uint8_t> (255 * in.v); 
        out.b = static_cast<uint8_t> (255 * p); 
        break;
      }
      case 2:
      {
        out.r = static_cast<uint8_t> (255 * p);
        out.g = static_cast<uint8_t> (255 * in.v);
        out.b = static_cast<uint8_t> (255 * t);
        break;
      }
      case 3:
      {
        out.r = static_cast<uint8_t> (255 * p);
        out.g = static_cast<uint8_t> (255 * q);
        out.b = static_cast<uint8_t> (255 * in.v);
        break;
      }
      case 4:
      {
        out.r = static_cast<uint8_t> (255 * t);
        out.g = static_cast<uint8_t> (255 * p); 
        out.b = static_cast<uint8_t> (255 * in.v); 
        break;
      }
      default:
      {
        out.r = static_cast<uint8_t> (255 * in.v); 
        out.g = static_cast<uint8_t> (255 * p); 
        out.b = static_cast<uint8_t> (255 * q);
        break;
      }      
    }
  }


  /** \brief Convert a XYZRGB point cloud to a XYZHSV
    * \param[in] in the input XYZRGB point cloud
    * \param[out] out the output XYZHSV point cloud
    */
  inline void 
  PointCloudXYZRGBtoXYZHSV (PointCloud<PointXYZRGB>& in,
                            PointCloud<PointXYZHSV>& out)
  {
    out.width   = in.width;
    out.height  = in.height;
    for (size_t i = 0; i < in.points.size (); i++)
    {
      PointXYZHSV p;
      PointXYZRGBtoXYZHSV (in.points[i], p);
      out.points.push_back (p);
    }
  }
  /** \brief Convert a XYZRGB point cloud to a XYZI
    * \param[in] in the input XYZRGB point cloud
    * \param[out] out the output XYZI point cloud
    */
  inline void 
  PointCloudXYZRGBtoXYZI (PointCloud<PointXYZRGB>& in,
                          PointCloud<PointXYZI>& out)
  {
    out.width   = in.width;
    out.height  = in.height;
    for (size_t i = 0; i < in.points.size (); i++)
    {
      PointXYZI p;
      PointXYZRGBtoXYZI (in.points[i], p);
      out.points.push_back (p);
    }
  }
}

#endif //#ifndef PCL_TYPE_CONVERSIONS_H

