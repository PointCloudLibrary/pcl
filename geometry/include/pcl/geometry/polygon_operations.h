/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: point_operators.h 4389 2012-02-10 10:10:26Z nizar $
 *
 */

#ifndef PCL_GEOMETRY_POLYGON_OPERATORS_H
#define PCL_GEOMETRY_POLYGON_OPERATORS_H

#include "planar_polygon.h"
#include <pcl/point_cloud.h>

namespace pcl
{
  /** \brief see approximatePolygon2D
    * \author Suat Gedikli <gedikli@willowgarage.com>
    */
  template <typename PointT>
  void approximatePolygon (const PlanarPolygon<PointT>& polygon, PlanarPolygon<PointT>& approx_polygon, float threshold, bool refine = false, bool closed = true);
  
  /** \brief returns an approximate polygon to given 2D contour. Uses just X and Y values.
    * \note  if refinement is not turned on, the resulting polygon will contain points from the original contour with their original z values (if any)
    * \note  if refinement is turned on, the z values of the refined polygon are not valid and should be set to 0 if point contains z attribute. 
    * \param [in] polygon input polygon
    * \param [out] approx_polygon approximate polygon
    * \param [in] threshold maximum allowed distance of an input vertex to an output edge
    * \param refine
    * \param [in] closed whether it is a closed polygon or a polyline
    * \author Suat Gedikli <gedikli@willowgarage.com>
    */
  template <typename PointT>
  void approximatePolygon2D (const typename PointCloud<PointT>::VectorType &polygon, 
                             typename PointCloud<PointT>::VectorType &approx_polygon, 
                             float threshold, bool refine = false, bool closed = true);

} // namespace pcl

#include "impl/polygon_operations.hpp"
#endif // PCL_GEOMETRY_POLYGON_OPERATORS_H=======
