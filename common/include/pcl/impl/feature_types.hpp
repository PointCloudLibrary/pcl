/*
 * SPDX-License-Identifier: BSD-3-Clause
 *  *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *  *  All rights reserved
 */

#pragma once

#include <pcl/feature_types.h>          // implementee
#include <pcl/pcl_exports.h>            // for PCL_EXPORTS
#include <pcl/register_point_struct.h>  // for POINT_CLOUD_REGISTER_POINT_STRUCT
#include <pcl/type_traits.h>            // for asEnum which is used internally in POINT_CLOUD_REGISTER_POINT_STRUCT

#include <ostream>                      // for ostream, operator<<

namespace pcl
{
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
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::MomentInvariants,
    (float, j1, j1)
    (float, j2, j2)
    (float, j3, j3)
)
