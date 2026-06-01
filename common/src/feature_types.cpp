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

  std::ostream&
  operator << (std::ostream& os, const SHOT352& p)
  {
    for (int i = 0; i < 9; ++i)
    os << (i == 0 ? "(" : "") << p.rf[i] << (i < 8 ? ", " : ")");
    for (std::size_t i = 0; i < 352; ++i)
    os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 351 ? ", " : ")");
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const SHOT1344& p)
  {
    for (int i = 0; i < 9; ++i)
    os << (i == 0 ? "(" : "") << p.rf[i] << (i < 8 ? ", " : ")");
    for (std::size_t i = 0; i < 1344; ++i)
    os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 1343 ? ", " : ")");
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const ReferenceFrame& p)
  {
    os << "("
       << p.x_axis[0] << " " << p.x_axis[1] << " " << p.x_axis[2] << ","
       << p.y_axis[0] << " " << p.y_axis[1] << " " << p.y_axis[2] << ","
       << p.z_axis[0] << " " << p.z_axis[1] << " " << p.z_axis[2] << ")";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const ShapeContext1980& p)
  {
    for (int i = 0; i < 9; ++i)
    os << (i == 0 ? "(" : "") << p.rf[i] << (i < 8 ? ", " : ")");
    for (std::size_t i = 0; i < 1980; ++i)
      os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 1979 ? ", " : ")");
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const UniqueShapeContext1960& p)
  {
    for (int i = 0; i < 9; ++i)
    os << (i == 0 ? "(" : "") << p.rf[i] << (i < 8 ? ", " : ")");
    for (std::size_t i = 0; i < 1960; ++i)
      os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 1959 ? ", " : ")");
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const PFHSignature125& p)
  {
    for (int i = 0; i < 125; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 124 ? ", " : ")");
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const PFHRGBSignature250& p)
  {
    for (int i = 0; i < 250; ++i)
    os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 249 ? ", " : ")");
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const PPFSignature& p)
  {
    os << "(" << p.f1 << ", " << p.f2 << ", " << p.f3 << ", " << p.f4 << ", " << p.alpha_m << ")";
    return (os);
  }

  std::ostream&
  operator << (std::ostream& os, const CPPFSignature& p)
  {
    os << "(" << p.f1 << ", " << p.f2 << ", " << p.f3 << ", " << p.f4 << ", " << p.f5 << ", " << p.f6 << ", " << p.f7 << ", " << p.f8 << ", " << p.f9 << ", " << p.f10 << ", " << p.alpha_m << ")";
    return (os);
  }
} // namespace pcl
