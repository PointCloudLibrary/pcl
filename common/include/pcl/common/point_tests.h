/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: point_types.hpp 6415 2012-07-16 20:11:47Z rusu $
 *
 */

#pragma once

#include <pcl/point_types.h>

#ifdef _MSC_VER
#include <Eigen/src/StlSupport/details.h>
#endif

namespace pcl
{
  /** Tests if the 3D components of a point are all finite
    * param[in] pt point to be tested
    * return true if finite, false otherwise
    */
  template <typename PointT> inline bool
  isFinite (const PointT &pt)
  {
    return (std::isfinite (pt.x) && std::isfinite (pt.y) && std::isfinite (pt.z));
  }

#ifdef _MSC_VER
  template <typename PointT> inline bool
  isFinite (const Eigen::internal::workaround_msvc_stl_support<PointT> &pt)
  {
    return isFinite<PointT> (static_cast<const PointT&> (pt));
  }
#endif

  template<> inline bool isFinite<pcl::Axis>(const pcl::Axis&) { return (true); }
  template<> inline bool isFinite<pcl::BRISKSignature512>(const pcl::BRISKSignature512&) { return (true); }
  template<> inline bool isFinite<pcl::BorderDescription>(const pcl::BorderDescription &) { return true; }
  template<> inline bool isFinite<pcl::Boundary>(const pcl::Boundary&) { return (true); }
  template<> inline bool isFinite<pcl::CPPFSignature>(const pcl::CPPFSignature&) { return (true); }
  template<> inline bool isFinite<pcl::ESFSignature640>(const pcl::ESFSignature640&) { return (true); }
  template<> inline bool isFinite<pcl::FPFHSignature33>(const pcl::FPFHSignature33&) { return (true); }
  template<> inline bool isFinite<pcl::GASDSignature512>(const pcl::GASDSignature512&) { return (true); }
  template<> inline bool isFinite<pcl::GASDSignature984>(const pcl::GASDSignature984&) { return (true); }
  template<> inline bool isFinite<pcl::GASDSignature7992>(const pcl::GASDSignature7992&) { return (true); }
  template<> inline bool isFinite<pcl::GRSDSignature21>(const pcl::GRSDSignature21&) { return (true); }
  template<> inline bool isFinite<pcl::Intensity>(const pcl::Intensity&) { return (true); }
  template<> inline bool isFinite<pcl::IntensityGradient>(const pcl::IntensityGradient&) { return (true); }
  template<> inline bool isFinite<pcl::Label>(const pcl::Label&) { return (true); }
  template<> inline bool isFinite<pcl::MomentInvariants>(const pcl::MomentInvariants&) { return (true); }
  template<> inline bool isFinite<pcl::NormalBasedSignature12>(const pcl::NormalBasedSignature12&) { return (true); }
  template<> inline bool isFinite<pcl::PFHRGBSignature250>(const pcl::PFHRGBSignature250&) { return (true); }
  template<> inline bool isFinite<pcl::PFHSignature125>(const pcl::PFHSignature125&) { return (true); }
  template<> inline bool isFinite<pcl::PPFRGBSignature>(const pcl::PPFRGBSignature&) { return (true); }
  
  template<> inline bool isFinite<pcl::PPFSignature>(const pcl::PPFSignature& pt)
  {
    return std::isfinite(pt.f1) && std::isfinite(pt.f2) && std::isfinite(pt.f3) && std::isfinite(pt.f4) && std::isfinite(pt.alpha_m);
  }

  template<> inline bool isFinite<pcl::PrincipalCurvatures>(const pcl::PrincipalCurvatures&) { return (true); }
  template<> inline bool isFinite<pcl::PrincipalRadiiRSD>(const pcl::PrincipalRadiiRSD&) { return (true); }
  template<> inline bool isFinite<pcl::RGB>(const pcl::RGB&) { return (true); }
  template<> inline bool isFinite<pcl::ReferenceFrame>(const pcl::ReferenceFrame&) { return (true); }
  template<> inline bool isFinite<pcl::SHOT1344>(const pcl::SHOT1344&) { return (true); }
  template<> inline bool isFinite<pcl::SHOT352>(const pcl::SHOT352&) { return (true); }
  template<> inline bool isFinite<pcl::ShapeContext1980>(const pcl::ShapeContext1980&) { return (true); }
  template<> inline bool isFinite<pcl::UniqueShapeContext1960>(const pcl::UniqueShapeContext1960&) { return (true); }
  template<> inline bool isFinite<pcl::VFHSignature308>(const pcl::VFHSignature308&) { return (true); }

  // specification for pcl::PointXY
  template <> inline bool
  isFinite<pcl::PointXY> (const pcl::PointXY &p)
  {
    return (std::isfinite (p.x) && std::isfinite (p.y));
  }

  // specification for pcl::Normal
  template <> inline bool
  isFinite<pcl::Normal> (const pcl::Normal &n)
  {
    return (std::isfinite (n.normal_x) && std::isfinite (n.normal_y) && std::isfinite (n.normal_z));
  }

  // generic fallback cases
  template <typename PointT, traits::HasNoXY<PointT> = true> constexpr inline bool
  isXYFinite (const PointT&) noexcept
  {
    return true;
  }

  template <typename PointT, traits::HasNoXYZ<PointT> = true> constexpr inline bool
  isXYZFinite (const PointT&) noexcept
  {
    return true;
  }

  template <typename PointT, traits::HasNoNormal<PointT> = true> constexpr inline bool
  isNormalFinite (const PointT&) noexcept
  {
    return true;
  }

  // special cases for checks
  template <typename PointT, traits::HasXY<PointT> = true> inline bool
  isXYFinite (const PointT& pt) noexcept
  {
    return std::isfinite(pt.x) && std::isfinite(pt.y);
  }

  template <typename PointT, traits::HasXYZ<PointT> = true> inline bool
  isXYZFinite (const PointT& pt) noexcept
  {
    return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
  }

  template <typename PointT, traits::HasNormal<PointT> = true> inline bool
  isNormalFinite (const PointT& pt) noexcept
  {
    return std::isfinite(pt.normal_x) && std::isfinite(pt.normal_y) && std::isfinite(pt.normal_z);
  }
}
