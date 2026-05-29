/*
 * SPDX-License-Identifier: BSD-3-Clause
 *  *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *  *  All rights reserved
 */

#pragma once

#include <pcl/descriptor_size.h>        // for descriptorSize_v
#include <pcl/memory.h>                 // for PCL_MAKE_ALIGNED_OPERATOR_NEW
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
      template<> struct descriptorSize<SHOT352> { static constexpr const int value = 352; };
      template<> struct descriptorSize<SHOT1344> { static constexpr const int value = 1344; };
      template<> struct descriptorSize<ShapeContext1980> { static constexpr const int value = 1980; };
      template<> struct descriptorSize<UniqueShapeContext1960> { static constexpr const int value = 1960; };
      template<> struct descriptorSize<PFHSignature125> { static constexpr const int value = 125; };
      template<> struct descriptorSize<PFHRGBSignature250> { static constexpr const int value = 250; };
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

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const SHOT352& p);
  /** \brief A point structure representing the generic Signature of Histograms of OrienTations (SHOT) - shape only.
    * \ingroup common
    */
  struct SHOT352
  {
    float descriptor[352] = {0.f};
    float rf[9] = {0.f};
    static constexpr int descriptorSize () { return detail::traits::descriptorSize_v<SHOT352>; }

    inline constexpr SHOT352 () = default;

    friend std::ostream& operator << (std::ostream& os, const SHOT352& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const SHOT1344& p);
  /** \brief A point structure representing the generic Signature of Histograms of OrienTations (SHOT) - shape+color.
    * \ingroup common
    */
  struct SHOT1344
  {
    float descriptor[1344] = {0.f};
    float rf[9] = {0.f};
    static constexpr int descriptorSize () { return detail::traits::descriptorSize_v<SHOT1344>; }

    inline constexpr SHOT1344 () = default;

    friend std::ostream& operator << (std::ostream& os, const SHOT1344& p);
  };

  /** \brief A structure representing the Local Reference Frame of a point.
    *  \ingroup common
    */
  struct EIGEN_ALIGN16 _ReferenceFrame
  {
    union
    {
      float rf[9];
      struct
      {
        float x_axis[3];
        float y_axis[3];
        float z_axis[3];
      };
    };

    inline Eigen::Map<Eigen::Vector3f> getXAxisVector3fMap () { return (Eigen::Vector3f::Map (x_axis)); }
    inline const Eigen::Map<const Eigen::Vector3f> getXAxisVector3fMap () const { return (Eigen::Vector3f::Map (x_axis)); }
    inline Eigen::Map<Eigen::Vector3f> getYAxisVector3fMap () { return (Eigen::Vector3f::Map (y_axis)); }
    inline const Eigen::Map<const Eigen::Vector3f> getYAxisVector3fMap () const { return (Eigen::Vector3f::Map (y_axis)); }
    inline Eigen::Map<Eigen::Vector3f> getZAxisVector3fMap () { return (Eigen::Vector3f::Map (z_axis)); }
    inline const Eigen::Map<const Eigen::Vector3f> getZAxisVector3fMap () const { return (Eigen::Vector3f::Map (z_axis)); }
    inline Eigen::Map<Eigen::Matrix3f> getMatrix3fMap () { return (Eigen::Matrix3f::Map (rf)); }
    inline const Eigen::Map<const Eigen::Matrix3f> getMatrix3fMap () const { return (Eigen::Matrix3f::Map (rf)); }

    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ReferenceFrame& p);
  struct EIGEN_ALIGN16 ReferenceFrame : public _ReferenceFrame
  {
    inline constexpr ReferenceFrame (const _ReferenceFrame &p) :
      ReferenceFrame{p.rf}
    {
      //std::copy_n(p.rf, 9, rf); // this algorithm is constexpr starting from C++20
    }

    inline constexpr ReferenceFrame () :
      _ReferenceFrame{ {{0.0f}} }
    {
      // this algorithm is constexpr starting from C++20
      /*std::fill_n(x_axis, 3, 0.f);
      std::fill_n(y_axis, 3, 0.f);
      std::fill_n(z_axis, 3, 0.f);*/
    }

    inline constexpr ReferenceFrame (const float (&_rf)[9]) :
      _ReferenceFrame{ {{_rf[0], _rf[1], _rf[2], _rf[3], _rf[4], _rf[5], _rf[6], _rf[7], _rf[8]}} } {}

    friend std::ostream& operator << (std::ostream& os, const ReferenceFrame& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ShapeContext1980& p);
  /** \brief A point structure representing a Shape Context.
    * \ingroup common
    */
  struct ShapeContext1980
  {
    float descriptor[1980] = {0.f};
    float rf[9] = {0.f};
    static constexpr int descriptorSize () { return detail::traits::descriptorSize_v<ShapeContext1980>; }

    inline constexpr ShapeContext1980 () = default;

    friend std::ostream& operator << (std::ostream& os, const ShapeContext1980& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const UniqueShapeContext1960& p);
  /** \brief A point structure representing a Unique Shape Context.
    * \ingroup common
    */
  struct UniqueShapeContext1960
  {
    float descriptor[1960] = {0.f};
    float rf[9] = {0.f};
    static constexpr int descriptorSize () { return detail::traits::descriptorSize_v<UniqueShapeContext1960>; }

    inline constexpr UniqueShapeContext1960 () = default;

    friend std::ostream& operator << (std::ostream& os, const UniqueShapeContext1960& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PFHSignature125& p);
  /** \brief A point structure representing the Point Feature Histogram (PFH).
    * \ingroup common
    */
  struct PFHSignature125
  {
    float histogram[125] = {0.f};
    static constexpr int descriptorSize () { return detail::traits::descriptorSize_v<PFHSignature125>; }

    inline constexpr PFHSignature125 () = default;

    friend std::ostream& operator << (std::ostream& os, const PFHSignature125& p);
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PFHRGBSignature250& p);
  /** \brief A point structure representing the Point Feature Histogram with colors (PFHRGB).
    * \ingroup common
    */
  struct PFHRGBSignature250
  {
    float histogram[250] = {0.f};
    static constexpr int descriptorSize () { return detail::traits::descriptorSize_v<PFHRGBSignature250>; }

    inline constexpr PFHRGBSignature250 () = default;

    friend std::ostream& operator << (std::ostream& os, const PFHRGBSignature250& p);
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

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::SHOT352,
    (float[352], descriptor, shot)
    (float[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::SHOT1344,
    (float[1344], descriptor, shot)
    (float[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_ReferenceFrame,
    (float[3], x_axis, x_axis)
    (float[3], y_axis, y_axis)
    (float[3], z_axis, z_axis)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::ReferenceFrame, pcl::_ReferenceFrame)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ShapeContext1980,
    (float[1980], descriptor, shape_context)
    (float[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::UniqueShapeContext1960,
    (float[1960], descriptor, shape_context)
    (float[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PFHSignature125,
    (float[125], histogram, pfh)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PFHRGBSignature250,
    (float[250], histogram, pfhrgb)
)
