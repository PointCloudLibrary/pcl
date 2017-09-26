/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2017-, Southwest Research Institute
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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

#ifndef PCL_SURFACE_ADVANCING_FRONT_POINT_TYPE_H_
#define PCL_SURFACE_ADVANCING_FRONT_POINT_TYPE_H_

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl/impl/instantiate.hpp>
#include <ostream>

namespace pcl
{
  namespace afront
  {
    struct EIGEN_ALIGN16 _AfrontVertexPointType
    {
      PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
      PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])

      union {
        struct
        {
          float curvature;
          float max_step;
          float max_step_search_radius;
        };
        float data_c[4];
      };
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    struct AfrontVertexPointType : public _AfrontVertexPointType
    {
      inline AfrontVertexPointType (const AfrontVertexPointType &p)
      {
        x = p.x;
        y = p.y;
        z = p.z;
        data[3] = 1.0f;
        normal_x = p.normal_x;
        normal_y = p.normal_y;
        normal_z = p.normal_z;
        data_n[3] = 0.0f;
        curvature = p.curvature;
        max_step = p.max_step;
        max_step_search_radius = p.max_step_search_radius;
      }

      inline AfrontVertexPointType ()
      {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        normal_x = normal_y = normal_z = data_n[3] = 0.0f;
        curvature = 0.0f;
        max_step = 0.0f;
        max_step_search_radius = 0.0f;
      }

      friend std::ostream &
      operator<< (std::ostream &os, const AfrontVertexPointType &p)
      {
        os << p.x << "\t" << p.y << "\t" << p.z;
        return (os);
      }
    };

    struct EIGEN_ALIGN16 _AfrontGuidanceFieldPointType
    {
      PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
      PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])

      union {
        struct
        {
          float curvature;
          float ideal_edge_length;
        };
        float data_c[4];
      };
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    struct AfrontGuidanceFieldPointType : public _AfrontGuidanceFieldPointType
    {
      inline AfrontGuidanceFieldPointType (const AfrontGuidanceFieldPointType &p)
      {
        x = p.x;
        y = p.y;
        z = p.z;
        data[3] = 1.0f;
        normal_x = p.normal_x;
        normal_y = p.normal_y;
        normal_z = p.normal_z;
        data_n[3] = 0.0f;
        curvature = p.curvature;
        ideal_edge_length = p.ideal_edge_length;
      }

      inline AfrontGuidanceFieldPointType(const AfrontVertexPointType &p, const double rho)
      {
        x = p.x;
        y = p.y;
        z = p.z;
        data[3] = 1.0f;
        normal_x = p.normal_x;
        normal_y = p.normal_y;
        normal_z = p.normal_z;
        data_n[3] = 0.0f;
        curvature = p.curvature;
        ideal_edge_length = 2.0 * std::sin (rho / 2.0) / curvature;
      }

      inline AfrontGuidanceFieldPointType ()
      {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        normal_x = normal_y = normal_z = data_n[3] = 0.0f;
        curvature = 0.0f;
        ideal_edge_length = 0.0f;
      }

      friend std::ostream &
      operator<< (std::ostream &os, const AfrontGuidanceFieldPointType &p)
      {
        os << p.x << "\t" << p.y << "\t" << p.z;
        return (os);
      }
    };
  } // namespace afront
} // namespace pcl
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::afront::AfrontVertexPointType,
                                   (float, x, x) (float, y, y) (float, z, z) (float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature) (float, max_step, max_step) (float, max_step_search_radius, max_step_search_radius))

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::afront::AfrontGuidanceFieldPointType,
                                   (float, x, x) (float, y, y) (float, z, z) (float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature) (float, ideal_edge_length, ideal_edge_length))

#endif // PCL_SURFACE_ADVANCING_FRONT_POINT_TYPE_H_
