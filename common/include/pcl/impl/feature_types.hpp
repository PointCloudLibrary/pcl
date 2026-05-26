/*
 * SPDX-License-Identifier: BSD-3-Clause
 *  *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *  *  All rights reserved
 */

#pragma once

#include <pcl/descriptor_size.h>        // for descriptorSize_v
#include <pcl/feature_types.h>          // implementee
#include <pcl/pcl_exports.h>            // for PCL_EXPORTS
#include <pcl/register_point_struct.h>  // for POINT_CLOUD_REGISTER_POINT_STRUCT
#include <pcl/type_traits.h>            // for asEnum which is used internally in POINT_CLOUD_REGISTER_POINT_STRUCT

#include <ostream>                      // for ostream, operator<<

namespace pcl
{
  namespace detail
  {
    namespace traits
    {
    }
  }
  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const MomentInvariants& p);
  /** \brief A point structure representing the three moment invariants.
    * \ingroup common
    */
  struct MomentInvariants
  {
    float j1 = 0.f, j2 = 0.f, j3 = 0.f;

    inline constexpr MomentInvariants () = default;

    inline constexpr MomentInvariants (float _j1, float _j2, float _j3): j1 (_j1), j2 (_j2), j3 (_j3) {}

    friend std::ostream& operator << (std::ostream& os, const MomentInvariants& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PrincipalRadiiRSD& p);
  /** \brief A point structure representing the minimum and maximum surface radii (in meters) computed using RSD.
    * \ingroup common
    */
  struct PrincipalRadiiRSD
  {
    float r_min = 0.f, r_max = 0.f;

    inline constexpr PrincipalRadiiRSD () = default;

    inline constexpr PrincipalRadiiRSD (float _r_min, float _r_max): r_min (_r_min), r_max (_r_max) {}

    friend std::ostream& operator << (std::ostream& os, const PrincipalRadiiRSD& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Boundary& p);
  /** \brief A point structure representing a description of whether a point is lying on a surface boundary or not.
    * \ingroup common
    */
  struct Boundary
  {
    std::uint8_t boundary_point = 0;

#if defined(_LIBCPP_VERSION) && _LIBCPP_VERSION <= 1101
    constexpr operator unsigned char() const
    {
      return boundary_point;
    }
#endif

    inline constexpr Boundary (std::uint8_t _boundary = 0): boundary_point (_boundary) {}

    friend std::ostream& operator << (std::ostream& os, const Boundary& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PrincipalCurvatures& p);
  /** \brief A point structure representing the principal curvatures and their magnitudes.
    * \ingroup common
    */
  struct PrincipalCurvatures
  {
    union
    {
      float principal_curvature[3];
      struct
      {
        float principal_curvature_x;
        float principal_curvature_y;
        float principal_curvature_z;
      };
    };
    float pc1 = 0.f;
    float pc2 = 0.f;

    inline constexpr PrincipalCurvatures (): PrincipalCurvatures (0.f, 0.f) {}

    inline constexpr PrincipalCurvatures (float _pc1, float _pc2): PrincipalCurvatures (0.f, 0.f, 0.f, _pc1, _pc2) {}

    inline constexpr PrincipalCurvatures (float _x, float _y, float _z): PrincipalCurvatures (_x, _y, _z, 0.f, 0.f) {}

    inline constexpr PrincipalCurvatures (float _x, float _y, float _z, float _pc1, float _pc2):
      principal_curvature_x (_x), principal_curvature_y (_y), principal_curvature_z (_z), pc1 (_pc1), pc2 (_pc2) {}

    friend std::ostream& operator << (std::ostream& os, const PrincipalCurvatures& p);
  };
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::MomentInvariants,
    (float, j1, j1)
    (float, j2, j2)
    (float, j3, j3)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PrincipalRadiiRSD,
    (float, r_min, r_min)
    (float, r_max, r_max)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Boundary,
    (std::uint8_t, boundary_point, boundary_point)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PrincipalCurvatures,
    (float, principal_curvature_x, principal_curvature_x)
    (float, principal_curvature_y, principal_curvature_y)
    (float, principal_curvature_z, principal_curvature_z)
    (float, pc1, pc1)
    (float, pc2, pc2)
)
