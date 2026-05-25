/*
 * SPDX-License-Identifier: BSD-3-Clause
 *  *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *  *  All rights reserved
 */

#include <pcl/feature_types.h>

namespace pcl
{
  std::ostream&
  operator << (std::ostream& os, const MomentInvariants& p)
  {
    os << "(" << p.j1 << "," << p.j2 << "," << p.j3 << ")";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const PrincipalRadiiRSD& p)
  {
    os << "(" << p.r_min << "," << p.r_max << ")";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const Boundary& p)
  {
    os << p.boundary_point;
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const PrincipalCurvatures& p)
  {
    os << "(" << p.principal_curvature[0] << "," << p.principal_curvature[1] << "," << p.principal_curvature[2] << " - " << p.pc1 << "," << p.pc2 << ")";
    return (os);
  }
} // namespace pcl
