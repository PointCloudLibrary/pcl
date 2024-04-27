/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/impl/instantiate.hpp>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/impl/extract_polygonal_prism_data.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
PCL_INSTANTIATE (
    ExtractPolygonalPrismData,
    (pcl::PointXYZ) (pcl::PointXYZI) (pcl::PointXYZRGBA) (pcl::PointXYZRGB))
PCL_INSTANTIATE (
    isPointIn2DPolygon,
    (pcl::PointXYZ) (pcl::PointXYZI) (pcl::PointXYZRGBA) (pcl::PointXYZRGB))
PCL_INSTANTIATE (
    isXYPointIn2DXYPolygon,
    (pcl::PointXYZ) (pcl::PointXYZI) (pcl::PointXYZRGBA) (pcl::PointXYZRGB))
#else
PCL_INSTANTIATE (ExtractPolygonalPrismData, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE (isPointIn2DPolygon, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE (isXYPointIn2DXYPolygon, PCL_XYZ_POINT_TYPES)
#endif
