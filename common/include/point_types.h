/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: point_types.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */
#ifndef PCL_DATA_TYPES_H_
#define PCL_DATA_TYPES_H_

#include <Eigen/Core>
#include <bitset>
#include <vector>
#include "pcl/ros/register_point_struct.h"
#include "pcl/win32_macros.h"
#include <math.h>

namespace pcl
{
  /** \brief A point structure representing Euclidean xyz coordinates. (SSE friendly) */
  struct PointXYZ;
  // Members: float x, y, z;

  /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.*/ 
  struct PointXYZI;
  // Members: float x, y, z, intensity; 

  /** \brief A point structure representing Euclidean xyz coordinates, and the RGBA color. */
  struct PointXYZRGBA;
  // Members: float x, y, z; uint32_t rgba;

  /** \brief A point structure representing Euclidean xyz coordinates, and the RGB color. */
  struct PointXYZRGB;
  // Members: float x, y, z, rgb;

  /** \brief A 2D point structure representing Euclidean xy coordinates. */
  struct PointXY;
  // Members: float x, y;

  /** \brief A point structure representing an interest point with Euclidean xyz coordinates, and an interest value. */
  struct InterestPoint;
  // Members: float x, y, z, strength;

  /** \brief A point structure representing normal coordinates and the surface curvature estimate. (SSE friendly) */
  struct Normal;
  // Members: float normal[3], curvature;

  /** \brief A point structure representing Euclidean xyz coordinates, together with normal coordinates and the surface curvature estimate. (SSE friendly) */
  struct PointNormal;
  // Members: float x, y, z; float normal[3], curvature;

  /** \brief A point structure representing Euclidean xyz coordinates, and the RGB color, together with normal coordinates and the surface curvature estimate. */
  struct PointXYZRGBNormal;
  // Members: float x, y, z, rgb, normal[3], curvature;

  /** \brief A point structure representing Euclidean xyz coordinates, intensity, together with normal coordinates and the surface curvature estimate. */
  struct PointXYZINormal;
  // Members: float x, y, z, intensity, normal[3], curvature;

  /** \brief A point structure representing Euclidean xyz coordinates, padded with an extra range float. */
  struct PointWithRange;
  // Members: float x, y, z (union with float point[4]), range;

  /** \brief A point structure representing Euclidean xyz coordinates together with the viewpoint from which it was seen. */
  struct PointWithViewpoint;
  // Members: float x, y, z, vp_x, vp_y, vp_z;

  /** \brief A point structure representing the three moment invariants. */
  struct MomentInvariants;
  // Members: float j1, j2, j3;

  // TODO add point type for Radius-based Surface Descriptor (RSD) histograms, and let pcl::RSDEstimation return it if needed

  /** \brief A point structure representing the minimum and maximum surface radii (in meters) computed using RSD. */
  struct PrincipalRadiiRSD;
  // Members: float r_min, r_max;

  /** \brief A point structure representing a description of whether a point is lying on a surface boundary or not. */
  struct Boundary;
  // Members: uint8_t boundary_point;

  /** \brief A point structure representing the principal curvatures and their magnitudes. */
  struct PrincipalCurvatures;
  // Members: float principal_curvature[3], pc1, pc2;

  /** \brief A point structure representing the Point Feature Histogram (PFH). */
  struct PFHSignature125;
  // Members: float pfh[125];

  /** \brief A point structure representing the Fast Point Feature Histogram (FPFH). */
  struct FPFHSignature33;
  // Members: float fpfh[33];

  /** \brief A point structure representing the Viewpoint Feature Histogram (VFH). */
  struct VFHSignature308;
  // Members: float vfh[308];
  
  /** \brief A point structure representing the Narf descriptor. */
  struct Narf36;
  // Members: float x, y, z, roll, pitch, yaw; float descriptor[36];


  /** \brief Data type to store extended information about a transition from foreground to backgroundSpecification of the fields for BorderDescription::traits. */
  typedef std::bitset<32> BorderTraits;

  /** \brief Specification of the fields for BorderDescription::traits. */
  enum BorderTrait 
  {
    BORDER_TRAIT__OBSTACLE_BORDER,
    BORDER_TRAIT__SHADOW_BORDER,
    BORDER_TRAIT__VEIL_POINT,
    BORDER_TRAIT__SHADOW_BORDER_TOP,
    BORDER_TRAIT__SHADOW_BORDER_RIGHT,
    BORDER_TRAIT__SHADOW_BORDER_BOTTOM,
    BORDER_TRAIT__SHADOW_BORDER_LEFT,
    BORDER_TRAIT__OBSTACLE_BORDER_TOP,
    BORDER_TRAIT__OBSTACLE_BORDER_RIGHT,
    BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM,
    BORDER_TRAIT__OBSTACLE_BORDER_LEFT,
    BORDER_TRAIT__VEIL_POINT_TOP,
    BORDER_TRAIT__VEIL_POINT_RIGHT,
    BORDER_TRAIT__VEIL_POINT_BOTTOM,
    BORDER_TRAIT__VEIL_POINT_LEFT,
  };
  
  /** \brief A structure to store if a point in a range image lies on a border between an obstacle and the background. */
  struct BorderDescription;
  // Members: int x, y; BorderTraits traits;

  /** \brief A point structure representing the intensity gradient of an XYZI point cloud. */
  struct IntensityGradient;
  // Members: float gradient[3];

  /** \brief A point structure representing an N-D histogram. */
  template <int N>
  struct Histogram;
  // Members: float histogram[N];

  /** \brief A point structure representing a 3-D position and scale. */
  struct PointWithScale;
  // Members: float x, y, z, scale;
  
  /** \brief A surfel, that is, a point structure representing Euclidean xyz coordinates, together with normal coordinates, a RGBA color, a radius, a confidence value and the surface curvature estimate. */
  struct PointSurfel;
  // Members: float x, y, z, normal[3], rgba, radius, confidence, curvature;

  // -----Functions on points-----
  //! Calculate the euclidean distance between the two given points.
  template <typename PointType1, typename PointType2>
  inline float euclideanDistance(const PointType1& p1, const PointType2& p2);
  
  //! Calculate the squared euclidean distance between the two given points.
  template <typename PointType1, typename PointType2>
  inline float squaredEuclideanDistance(const PointType1& p1, const PointType2& p2);

  //! Checks if x,y,z are finite numbers.
  template <typename PointType>
  inline bool hasValidXYZ(const PointType& p);
}


#include "pcl/impl/point_types.hpp"  // Include struct definitions


// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZ,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
);

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZ, pcl::_PointXYZ)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGBA,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint32_t, rgba, rgba)
);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGB,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::InterestPoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, strength, strength)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZI, 
                                   (float, x, x) 
                                   (float, y, y) 
                                   (float, z, z) 
                                   (float, intensity, intensity) 
); 

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Normal,
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointNormal,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGBNormal,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZINormal,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointWithRange,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, range, range)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointWithViewpoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, vp_x, vp_x)
                                   (float, vp_y, vp_y)
                                   (float, vp_z, vp_z)
);

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointWithViewpoint, pcl::_PointWithViewpoint);


POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::MomentInvariants,
                                   (float, j1, j1)
                                   (float, j2, j2)
                                   (float, j3, j3)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PrincipalRadiiRSD,
                                   (float, r_min, r_min)
                                   (float, r_max, r_max)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Boundary,
                                   (uint8_t, boundary_point, boundary_point)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PrincipalCurvatures,
                                   (float, principal_curvature_x, principal_curvature_x)
                                   (float, principal_curvature_y, principal_curvature_y)
                                   (float, principal_curvature_z, principal_curvature_z)
                                   (float, pc1, pc1)
                                   (float, pc2, pc2)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PFHSignature125,
                                   (float[125], histogram, pfh)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::FPFHSignature33,
                                   (float[33], histogram, fpfh)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::VFHSignature308,
                                   (float[308], histogram, vfh)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Narf36,
                                   (float[36], descriptor, descriptor)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::IntensityGradient,
                                   (float, gradient_x, gradient_x)
                                   (float, gradient_y, gradient_y)
                                   (float, gradient_z, gradient_z)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointWithScale,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, scale, scale)
);

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointSurfel,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (uint32_t, rgba, rgba)
                                   (float, radius, radius)
                                   (float, confidence, confidence)
                                   (float, curvature, curvature)
);

//POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::BorderDescription,
                                   //(int, x, x)
                                   //(int, y, y)
                                   //(uint32_t, traits_int, traits)
//);


#endif  //#ifndef PCL_DATA_TYPES_H_
