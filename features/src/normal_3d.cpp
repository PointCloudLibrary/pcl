/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 */

#include <pcl/features/impl/normal_3d.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointSurfel)(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal))((pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal)))
#else
  PCL_INSTANTIATE_PRODUCT(NormalEstimation, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES))
#endif
#endif    // PCL_NO_PRECOMPILE

