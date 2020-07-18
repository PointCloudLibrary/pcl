/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 */

#include <pcl/features/impl/fpfh.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE_PRODUCT(FPFHEstimation, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal))((pcl::Normal)(pcl::PointNormal))((pcl::FPFHSignature33)))
#else
  PCL_INSTANTIATE_PRODUCT(FPFHEstimation, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES)((pcl::FPFHSignature33)))
#endif
#endif    // PCL_NO_PRECOMPILE

