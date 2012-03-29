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

#ifndef PCL_COMMON_POLYGON_OPERATORS_H
#define PCL_COMMON_POLYGON_OPERATORS_H

#include "planar_polygon.h"

namespace pcl
{
  /** \brief fuses two planar polygons (only X and Y coordinates will be used!), where fusing means that the outer
    * possible non-convex - contour of the polygon union will be determined
    * \param[in] planar_polygon1 first input polygon
    * \param[in] planar_polygon2 second input polygon
    * \param[out] fused_polygon the outer contour of the polygon-union
    * \return true if merging was successful - this means basically if the polygons were intersecting or nor
    */
  template <typename PointT>
  bool fusePlanarPolygons (const PlanarPolygon<PointT>& planar_polygon1, const PlanarPolygon<PointT>& planar_polygon2, PlanarPolygon<PointT>& fused_polygon);

  /** \brief computes the convex hull of a polygon by exploiting the polygon structure
    * \param[in] polygon input polygon
    * \param[out] convex_hull indices of the convex polygon that contains the input polygon completely
    * \attention input polygon must be planar AND counter clockwise => normal direction is important
    */
//  template <typename PointT> void
//  computeConvexHull2D (const PlanarPolygon<PointT>& polygon, std::vector<unsigned>& convex_hull);

  /** \brief computes the bounding box with smallest area that completely encloses the input polygon
    * \param[in] polygon input polygon
    * \param[out] bounding_box the bounding box
    */
//  template <typename PointT> void
//  getMinimumBoundingRectangle (const PlanarPolygon<PointT>& polygon, Box2D& bounding_box );
}

