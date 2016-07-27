/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_GEOMETRY_H_
#define PCL_GEOMETRY_H_

#if defined __GNUC__
#  pragma GCC system_header
#endif

#include <Eigen/Core>

/**
  * \file common/geometry.h
  * Defines some geometrical functions and utility functions
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  namespace geometry
  {
    /** @return the euclidean distance between 2 points */
    template <typename PointT> inline float 
    distance (const PointT& p1, const PointT& p2)
    {
      Eigen::Vector3f diff = p1.getVector3fMap () - p2.getVector3fMap ();
      return (diff.norm ());
    }

    /** @return the squared euclidean distance between 2 points */
    template<typename PointT> inline float 
    squaredDistance (const PointT& p1, const PointT& p2)
    {
      Eigen::Vector3f diff = p1.getVector3fMap () - p2.getVector3fMap ();
      return (diff.squaredNorm ());
    }

    /** @return the point projection on a plane defined by its origin and normal vector 
      * \param[in] point Point to be projected
      * \param[in] plane_origin The plane origin
      * \param[in] plane_normal The plane normal 
      * \param[out] projected The returned projected point
      */
    template<typename PointT, typename NormalT> inline void 
    project (const PointT& point, const PointT &plane_origin, 
             const NormalT& plane_normal, PointT& projected)
    {
      Eigen::Vector3f po = point - plane_origin;
      const Eigen::Vector3f normal = plane_normal.getVector3fMapConst ();
      float lambda = normal.dot(po);
      projected.getVector3fMap () = point.getVector3fMapConst () - (lambda * normal);
    }

    /** @return the point projection on a plane defined by its origin and normal vector 
      * \param[in] point Point to be projected
      * \param[in] plane_origin The plane origin
      * \param[in] plane_normal The plane normal 
      * \param[out] projected The returned projected point
      */
    inline void 
    project (const Eigen::Vector3f& point, const Eigen::Vector3f &plane_origin, 
             const Eigen::Vector3f& plane_normal, Eigen::Vector3f& projected)
    {
      Eigen::Vector3f po = point - plane_origin;
      float lambda = plane_normal.dot(po);
      projected = point - (lambda * plane_normal);
    }
  }
}

/*@}*/
#endif  //#ifndef PCL_GEOMETRY_H_
