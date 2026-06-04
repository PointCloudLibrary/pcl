/*
 * SPDX-License-Identifier: BSD-3-Clause
 *  *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *  *  All rights reserved
 */

#pragma once


/**
 * \file pcl/feature_types.h
 * Defines all the PCL implemented PointT feature type structures
 * \ingroup common
 */

/** @{*/
namespace pcl
{
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
} // namespace pcl
/** @} */

#include <pcl/impl/feature_types.hpp>
