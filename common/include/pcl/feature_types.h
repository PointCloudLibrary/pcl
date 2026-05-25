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
} // namespace pcl
/** @} */

#include <pcl/impl/feature_types.hpp>
