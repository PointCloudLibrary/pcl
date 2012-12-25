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

#ifndef PCL_APPS_IN_HAND_SCANNER_IMPL_COMMON_TYPES_HPP
#define PCL_APPS_IN_HAND_SCANNER_IMPL_COMMON_TYPES_HPP

#include <limits>

namespace pcl
{
  namespace ihs
  {
    struct EIGEN_ALIGN16 _PointIHS
    {
      PCL_ADD_POINT4D
      PCL_ADD_NORMAL4D
      PCL_ADD_RGB
      float        weight;
      unsigned int age;
      uint32_t     directions;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct PointIHS : public pcl::ihs::_PointIHS
    {
      // NOTE: I rely on NaN in the default constructor!
      inline PointIHS ()
      {
        this->x = this->y = this->z = std::numeric_limits<float>::quiet_NaN ();
        this->data[3] = 1.f;

        this->normal_x = this->normal_y = this->normal_z = std::numeric_limits<float>::quiet_NaN ();
        this->data_n[3] = 0.f;

        this->b = this->g = this->r = 0; this->a = 255;

        this->weight     = 0.f;
        this->age        = 0;
        this->directions = 0;
      }

      inline PointIHS (const PointIHS& other)
      {
        this->x       = other.x;
        this->y       = other.y;
        this->z       = other.z;
        this->data[3] = other.data[3];

        this->normal_x  = other.normal_x;
        this->normal_y  = other.normal_y;
        this->normal_z  = other.normal_z;
        this->data_n[3] = other.data_n[3];

        this->rgba = other.rgba;

        this->weight     = other.weight;
        this->age        = other.age;
        this->directions = other.directions;
      }

      inline PointIHS (const pcl::PointXYZRGBNormal& other, const float weight)
      {
        this->x       = other.x;
        this->y       = other.y;
        this->z       = other.z;
        this->data[3] = other.data[3];

        this->normal_x  = other.normal_x;
        this->normal_y  = other.normal_y;
        this->normal_z  = other.normal_z;
        this->data_n[3] = other.data_n[3];

        this->rgba = other.rgba;

        this->weight     = weight;
        this->age        = 0;
        this->directions = 0;
      }

   // inline       Eigen::Vector3i getRGBVector3i ()       {return (Eigen::Vector3i (r, g, b));}
      inline const Eigen::Vector3i getRGBVector3i () const {return (Eigen::Vector3i (r, g, b));}
   // inline       Eigen::Vector4i getRGBVector4i ()       {return (Eigen::Vector4i (r, g, b, a));}
      inline const Eigen::Vector4i getRGBVector4i () const {return (Eigen::Vector4i (r, g, b, a));}
    };

  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_IMPL_COMMON_TYPES_HPP
