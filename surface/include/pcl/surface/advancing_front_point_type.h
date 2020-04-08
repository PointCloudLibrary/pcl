/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2017-, Southwest Research Institute
 * Copyright (c) 2017-, Open Perception, Inc.
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
#include <ostream>

namespace pcl
{
  struct EIGEN_ALIGN16 _AdvancingFrontVertexPointType
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

  struct AdvancingFrontVertexPointType : public _AdvancingFrontVertexPointType
  {
    inline AdvancingFrontVertexPointType ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      curvature = 0.0f;
      max_step = 0.0f;
      max_step_search_radius = 0.0f;
    }

    friend std::ostream &
    operator<< (std::ostream &os, const AdvancingFrontVertexPointType &p)
    {
      os << p.x << "\t" << p.y << "\t" << p.z << "\t"
         << p.normal_x << "\t" << p.normal_y << "\t" << p.normal_z << "\t"
         << p.curvature << "\t" << p.max_step << "\t" << p.max_step_search_radius;

      return (os);
    }
  };

  struct EIGEN_ALIGN16 _AdvancingFrontGuidanceFieldPointType
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

  struct AdvancingFrontGuidanceFieldPointType : public _AdvancingFrontGuidanceFieldPointType
  {
    inline AdvancingFrontGuidanceFieldPointType(const AdvancingFrontVertexPointType &p, const double rho)
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

    inline AdvancingFrontGuidanceFieldPointType ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      curvature = 0.0f;
      ideal_edge_length = 0.0f;
    }

    friend std::ostream &
    operator<< (std::ostream &os, const AdvancingFrontGuidanceFieldPointType &p)
    {
      os << p.x << "\t" << p.y << "\t" << p.z << "\t"
         << p.normal_x << "\t" << p.normal_y << "\t" << p.normal_z << "\t"
         << p.curvature << "\t" << p.ideal_edge_length;

      return (os);
    }
  };
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::AdvancingFrontVertexPointType,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float, max_step, max_step)
                                   (float, max_step_search_radius, max_step_search_radius))

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::AdvancingFrontGuidanceFieldPointType,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float, ideal_edge_length, ideal_edge_length))

#endif // PCL_SURFACE_ADVANCING_FRONT_POINT_TYPE_H_
