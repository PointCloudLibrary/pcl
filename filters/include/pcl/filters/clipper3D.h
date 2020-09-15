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
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <Eigen/StdVector>

namespace pcl
{
  /**
    * \brief Base class for 3D clipper objects
    * \author Suat Gedikli <gedikli@willowgarage.com>
    * \ingroup filters
    */
  template<typename PointT>
  class Clipper3D
  {
    public:
      using Ptr = shared_ptr<Clipper3D<PointT> >;
      using ConstPtr = shared_ptr<const Clipper3D<PointT> >;
 
      /**
        * \brief virtual destructor. Never throws an exception.
        */
      virtual ~Clipper3D () noexcept {}

      /**
        * \brief interface to clip a single point
        * \param[in] point the point to check against
        * \return true, it point still exists, false if its clipped
        */
      virtual bool
      clipPoint3D (const PointT& point) const = 0;

      /**
        * \brief interface to clip a line segment given by two end points. The order of the end points is unimportant and will sty the same after clipping.
        * This means basically, that the direction of the line will not flip after clipping.
        * \param[in,out] pt1 start point of the line
        * \param[in,out] pt2 end point of the line
        * \return true if the clipped line is not empty, thus the parameters are still valid, false if line completely outside clipping space
        */
      virtual bool
      clipLineSegment3D (PointT& pt1, PointT& pt2) const = 0;

      /**
        * \brief interface to clip a planar polygon given by an ordered list of points
        * \param[in,out] polygon the polygon in any direction (ccw or cw) but ordered, thus two neighboring points define an edge of the polygon
        */
      virtual void
      clipPlanarPolygon3D (std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon) const = 0;

      /**
        * \brief interface to clip a planar polygon given by an ordered list of points
        * \param[in] polygon the polygon in any direction (ccw or cw) but ordered, thus two neighboring points define an edge of the polygon
        * \param[out] clipped_polygon the clipped polygon
        */
      virtual void
      clipPlanarPolygon3D (const std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon, std::vector<PointT, Eigen::aligned_allocator<PointT> >& clipped_polygon) const = 0;

      /**
        * \brief interface to clip a point cloud
        * \param[in] cloud_in input point cloud
        * \param[out] clipped indices of points that remain after clipping the input cloud
        * \param[in] indices the indices of points in the point cloud to be clipped.
        * \return list of indices of remaining points after clipping.
        */
      virtual void
      clipPointCloud3D (const pcl::PointCloud<PointT> &cloud_in, Indices& clipped, const Indices& indices = Indices ()) const = 0;

      /**
        * \brief polymorphic method to clone the underlying clipper with its parameters.
        * \return the new clipper object from the specific subclass with all its parameters.
        */
      virtual Clipper3D<PointT>*
      clone () const = 0;
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}
