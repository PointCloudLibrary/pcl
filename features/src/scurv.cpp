/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, OpenPerception
 *
 *  \authors Antonio J Rodríguez-Sánchez, University of Innsbruck
 *           Tomáš Tureček, Tomas Bata University in Zlín
 *           Alex Melniciuc, University of Innsbruck
 *
 *  All rights reserved.
 */

#include <pcl/features/impl/scurv.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE_PRODUCT(SCurVEstimation, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal))((pcl::Normal)(pcl::PointNormal)))
#else
  PCL_INSTANTIATE_PRODUCT(SCurVEstimation, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES))
#endif
#endif    // PCL_NO_PRECOMPILE

