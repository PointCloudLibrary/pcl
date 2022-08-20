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

#include <bitset>


/**
  * \file pcl/point_types.h
  * Defines all the PCL implemented PointT point type structures
  * \ingroup common
  */

// Allow nameless structs/unions
#if defined _MSC_VER
  #pragma warning(disable: 4201)
#endif

/** @{*/
namespace pcl
{
  /** \brief Members: float x, y, z
    * \ingroup common
    */
  struct PointXYZ;

  /** \brief Members: rgba
    * \ingroup common
    */
  struct RGB;

  /** \brief Members: intensity (float)
    * \ingroup common
    */
  struct Intensity;

  /** \brief Members: intensity (std::uint8_t)
    * \ingroup common
    */
  struct Intensity8u;

  /** \brief Members: intensity (std::uint32_t)
    * \ingroup common
    */
  struct Intensity32u;

  /** \brief Members: float x, y, z, intensity
    * \ingroup common
    */
  struct PointXYZI;

  /** \brief Members: float x, y, z, uin32_t label
    * \ingroup common
    */
  struct PointXYZL;

  /** \brief Members: std::uint32_t label
    * \ingroup common
    */
  struct Label;

  /** \brief Members: float x, y, z; std::uint32_t rgba
    * \ingroup common
    */
  struct PointXYZRGBA;

  /** \brief Members: float x, y, z, rgb
    * \ingroup common
    */
  struct PointXYZRGB;

  /** \brief Members: float x, y, z, rgb, std::uint32_t label
    * \ingroup common
    */
  struct PointXYZRGBL;

  /** \brief Members: float x, y, z, L, a, b
    * \ingroup common
    */
  struct PointXYZLAB;

  /** \brief Members: float x, y, z, h, s, v
    * \ingroup common
    */
  struct PointXYZHSV;

  /** \brief Members: float x, y
    * \ingroup common
    */
  struct PointXY;

  /** \brief Members: float u, v
    * \ingroup common
    */
  struct PointUV;

  /** \brief Members: float x, y, z, strength
    * \ingroup common
    */
  struct InterestPoint;

  /** \brief Members: float normal[3], curvature
    * \ingroup common
    */
  struct Normal;

  /** \brief Members: float normal[3]
    * \ingroup common
    */
  struct Axis;

  /** \brief Members: float x, y, z; float normal[3], curvature
    * \ingroup common
    */
  struct PointNormal;

  /** \brief Members: float x, y, z, rgb, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZRGBNormal;

  /** \brief Members: float x, y, z, intensity, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZINormal;

  /** \brief Members: float x, y, z, label, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZLNormal;

  /** \brief Members: float x, y, z (union with float point[4]), range
    * \ingroup common
    */
  struct PointWithRange;

  /** \brief Members: float x, y, z, vp_x, vp_y, vp_z
    * \ingroup common
    */
  struct PointWithViewpoint;

  /** \brief Members: float j1, j2, j3
    * \ingroup common
    */
  struct MomentInvariants;

  /** \brief Members: float r_min, r_max
    * \ingroup common
    */
  struct PrincipalRadiiRSD;

  /** \brief Members: std::uint8_t boundary_point
    * \ingroup common
    */
  struct Boundary;

  /** \brief Members: float principal_curvature[3], pc1, pc2
    * \ingroup common
    */
  struct PrincipalCurvatures;

  /** \brief Members: float descriptor[352], rf[9]
    * \ingroup common
    */
  struct SHOT352;

  /** \brief Members: float descriptor[1344], rf[9]
    * \ingroup common
    */
  struct SHOT1344;

  /** \brief Members: Axis x_axis, y_axis, z_axis
    * \ingroup common
    */
  struct ReferenceFrame;

  /** \brief Members: float descriptor[1980], rf[9]
    * \ingroup common
    */
  struct ShapeContext1980;

  /** \brief Members: float descriptor[1960], rf[9]
    * \ingroup common
    */
  struct UniqueShapeContext1960;

  /** \brief Members: float pfh[125]
    * \ingroup common
    */
  struct PFHSignature125;

  /** \brief Members: float pfhrgb[250]
    * \ingroup common
    */
  struct PFHRGBSignature250;

  /** \brief Members: float f1, f2, f3, f4, alpha_m
    * \ingroup common
    */
  struct PPFSignature;

  /** \brief Members: float f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, alpha_m
    * \ingroup common
    */
  struct CPPFSignature;

  /** \brief Members: float f1, f2, f3, f4, r_ratio, g_ratio, b_ratio, alpha_m
    * \ingroup common
    */
  struct PPFRGBSignature;

  /** \brief Members: float values[12]
    * \ingroup common
    */
  struct NormalBasedSignature12;

  /** \brief Members: float fpfh[33]
    * \ingroup common
    */
  struct FPFHSignature33;

  /** \brief Members: float vfh[308]
    * \ingroup common
    */
  struct VFHSignature308;

  /** \brief Members: float grsd[21]
    * \ingroup common
    */
  struct GRSDSignature21;

  /** \brief Members: float esf[640]
    * \ingroup common
    */
  struct ESFSignature640;

  /** \brief Members: float gasd[512]
  * \ingroup common
  */
  struct GASDSignature512;

  /** \brief Members: float gasd[984]
  * \ingroup common
  */
  struct GASDSignature984;

  /** \brief Members: float gasd[7992]
  * \ingroup common
  */
  struct GASDSignature7992;

  /** \brief Members: float histogram[16]
    * \ingroup common
    */
  struct GFPFHSignature16;

  /** \brief Members: float scale; float orientation; std::uint8_t descriptor[64]
    * \ingroup common
    */
  struct BRISKSignature512;

   /** \brief Members: float x, y, z, roll, pitch, yaw; float descriptor[36]
     * \ingroup common
     */
  struct Narf36;

  /** \brief Data type to store extended information about a transition from foreground to backgroundSpecification of the fields for BorderDescription::traits.
    * \ingroup common
    */
  using BorderTraits = std::bitset<32>;

  /** \brief Specification of the fields for BorderDescription::traits.
    * \ingroup common
    */
  enum BorderTrait
  {
    BORDER_TRAIT__OBSTACLE_BORDER, BORDER_TRAIT__SHADOW_BORDER, BORDER_TRAIT__VEIL_POINT,
    BORDER_TRAIT__SHADOW_BORDER_TOP, BORDER_TRAIT__SHADOW_BORDER_RIGHT, BORDER_TRAIT__SHADOW_BORDER_BOTTOM,
    BORDER_TRAIT__SHADOW_BORDER_LEFT, BORDER_TRAIT__OBSTACLE_BORDER_TOP, BORDER_TRAIT__OBSTACLE_BORDER_RIGHT,
    BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM, BORDER_TRAIT__OBSTACLE_BORDER_LEFT, BORDER_TRAIT__VEIL_POINT_TOP,
    BORDER_TRAIT__VEIL_POINT_RIGHT, BORDER_TRAIT__VEIL_POINT_BOTTOM, BORDER_TRAIT__VEIL_POINT_LEFT
  };

  /** \brief Members: int x, y; BorderTraits traits
    * \ingroup common
    */
  struct BorderDescription;

  /** \brief Members: float gradient[3]
    * \ingroup common
    */
  struct IntensityGradient;

  /** \brief Members: float histogram[N]
    * \ingroup common
    */
  template<int N>
  struct Histogram;

  /** \brief Members: float x, y, z, scale, angle, response, octave
    * \ingroup common
    */
  struct PointWithScale;

  /** \brief Members: float x, y, z, normal[3], rgba, radius, confidence, curvature
    * \ingroup common
    */
  struct PointSurfel;

  /** \brief Members: float x, y, z, intensity, intensity_variance, height_variance
    * \ingroup common
    */
  struct PointDEM;
} // namespace pcl
/** @} */

#include <pcl/impl/point_types.hpp>
