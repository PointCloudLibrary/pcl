/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_IN_HAND_SCANNER_COMMON_TYPES_HPP
#define PCL_IN_HAND_SCANNER_COMMON_TYPES_HPP

#include <limits>

namespace pcl
{
  namespace ihs
  {
    struct EIGEN_ALIGN16 PointModel
    {
      // NOTE: I rely on NaN in the default constructor!
      inline PointModel ()
        : x        (std::numeric_limits<float>::quiet_NaN ()),
          y        (std::numeric_limits<float>::quiet_NaN ()),
          z        (std::numeric_limits<float>::quiet_NaN ()),
          normal_x (std::numeric_limits<float>::quiet_NaN ()),
          normal_y (std::numeric_limits<float>::quiet_NaN ()),
          normal_z (std::numeric_limits<float>::quiet_NaN ()),
          b        (0),
          g        (0),
          r        (0),
          a        (255),
          weight   (0.f),
          age      (0),
          visconf  ()
      {
        data[3]   = 1.f;
        data_n[3] = 0.f;
      }

      inline PointModel (const float    x, const float    y, const float    z,
                         const float   nx, const float   ny, const float   nz,
                         const uint8_t  r, const uint8_t  g, const uint8_t  b,
                         const float weight)
        : x        ( x), y        ( y), z        ( z),
          normal_x (nx), normal_y (ny), normal_z (nz),
          b        ( b), g        ( g), r        ( r), a (255),
          weight   (weight),
          age      (0),
          visconf  ()
      {
        data[3]   = 1.f;
        data_n[3] = 0.f;
      }

      inline PointModel (const PointModel& other)
        : x        (other.x),        y        (other.y),        z        (other.z),
          normal_x (other.normal_x), normal_y (other.normal_y), normal_z (other.normal_z),
          rgba     (other.rgba),
          weight   (other.weight),
          age      (other.age),
          visconf  (other.visconf)
      {
        data[3]   = 1.f;
        data_n[3] = 0.f;
      }

      inline PointModel (const pcl::PointXYZRGBNormal& other, const float weight)
        : x        (other.x),        y        (other.y),        z        (other.z),
          normal_x (other.normal_x), normal_y (other.normal_y), normal_z (other.normal_z),
          rgba     (other.rgba),
          weight   (weight),
          age      (0)
      {
        data[3]   = 1.f;
        data_n[3] = 0.f;
      }

   // inline       Eigen::Vector3i getRGBVector3i ()       {return (Eigen::Vector3i (r, g, b));}
      inline const Eigen::Vector3i getRGBVector3i () const {return (Eigen::Vector3i (r, g, b));}
   // inline       Eigen::Vector4i getRGBVector4i ()       {return (Eigen::Vector4i (r, g, b, a));}
      inline const Eigen::Vector4i getRGBVector4i () const {return (Eigen::Vector4i (r, g, b, a));}

      PCL_ADD_POINT4D
      PCL_ADD_NORMAL4D
      PCL_ADD_RGB
      float weight;
      unsigned int age;
      pcl::ihs::VisibilityConfidence visconf;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_COMMON_TYPES_HPP
