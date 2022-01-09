/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#pragma once

#include <limits>

#include <pcl/pcl_base.h>

namespace pcl
{
  /** \brief General purpose method for checking if a 3D point is inside or
    * outside a given 2D polygon. 
    * \note this method accepts any general 3D point that is projected onto the
    * 2D polygon, but performs an internal XY projection of both the polygon and the point. 
    * \param point a 3D point projected onto the same plane as the polygon
    * \param polygon a polygon
    * \ingroup segmentation
    */
  template <typename PointT> bool 
  isPointIn2DPolygon (const PointT &point, const pcl::PointCloud<PointT> &polygon);

  /** \brief Check if a 2d point (X and Y coordinates considered only!) is
    * inside or outside a given polygon. This method assumes that both the point
    * and the polygon are projected onto the XY plane.
    *
    * \note (This is highly optimized code taken from http://www.visibone.com/inpoly/)
    *       Copyright (c) 1995-1996 Galacticomm, Inc.  Freeware source code.
    * \param point a 2D point projected onto the same plane as the polygon
    * \param polygon a polygon
    * \ingroup segmentation
    */
  template <typename PointT> bool 
  isXYPointIn2DXYPolygon (const PointT &point, const pcl::PointCloud<PointT> &polygon);

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ExtractPolygonalPrismData uses a set of point indices that
    * represent a planar model, and together with a given height, generates a 3D
    * polygonal prism. The polygonal prism is then used to segment all points
    * lying inside it.
    *
    * An example of its usage is to extract the data lying within a set of 3D
    * boundaries (e.g., objects supported by a plane).
    *
    * Example usage:
    * \code{.cpp}
    * double z_min = 0., z_max = 0.05; // we want the points above the plane, no farther than 5 cm from the surface
    * pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZ> ());
    * pcl::ConvexHull<pcl::PointXYZ> hull;
    * // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
    * hull.setInputCloud (cloud);
    * hull.reconstruct (hull_points);
    * if (hull.getDimension () == 2)
    * {
    *   pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    *   prism.setInputCloud (point_cloud);
    *   prism.setInputPlanarHull (hull_points);
    *   prism.setHeightLimits (z_min, z_max);
    *   prism.segment (cloud_indices);
    * }
    * else
    *  PCL_ERROR ("The input cloud does not represent a planar surface.\n");
    * \endcode
    * \author Radu Bogdan Rusu
    * \ingroup segmentation
    */
  template <typename PointT>
  class ExtractPolygonalPrismData : public PCLBase<PointT>
  {
    using PCLBase<PointT>::input_;
    using PCLBase<PointT>::indices_;
    using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

    public:
      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      using PointIndicesPtr = PointIndices::Ptr;
      using PointIndicesConstPtr = PointIndices::ConstPtr;

      /** \brief Empty constructor. */
      ExtractPolygonalPrismData () : planar_hull_ (), min_pts_hull_ (3), 
                                     height_limit_min_ (0),
                                     height_limit_max_(std::numeric_limits<float>::max()),
                                     vpx_ (0), vpy_ (0), vpz_ (0)
      {};

      /** \brief Provide a pointer to the input planar hull dataset.
        * \note Please see the example in the class description for how to obtain this.
        * \param[in] hull the input planar hull dataset
        */
      inline void 
      setInputPlanarHull (const PointCloudConstPtr &hull) { planar_hull_ = hull; }

      /** \brief Get a pointer the input planar hull dataset. */
      inline PointCloudConstPtr 
      getInputPlanarHull () const { return (planar_hull_); }

      /** \brief Set the height limits. All points having distances to the
        * model outside this interval will be discarded.
        *
        * \param[in] height_min the minimum allowed distance to the plane model value
        * \param[in] height_max the maximum allowed distance to the plane model value
        */
      inline void
      setHeightLimits (double height_min, double height_max)
      {
        height_limit_min_ = height_min;
        height_limit_max_ = height_max;
      }

      /** \brief Get the height limits (min/max) as set by the user. The
        * default values are 0, std::numeric_limits<float>::max().
        * \param[out] height_min the resultant min height limit
        * \param[out] height_max the resultant max height limit
        */
      inline void
      getHeightLimits (double &height_min, double &height_max) const
      {
        height_min = height_limit_min_;
        height_max = height_limit_max_;
      }

      /** \brief Set the viewpoint.
        * \param[in] vpx the X coordinate of the viewpoint
        * \param[in] vpy the Y coordinate of the viewpoint
        * \param[in] vpz the Z coordinate of the viewpoint
        */
      inline void
      setViewPoint (float vpx, float vpy, float vpz)
      {
        vpx_ = vpx;
        vpy_ = vpy;
        vpz_ = vpz;
      }

      /** \brief Get the viewpoint. */
      inline void
      getViewPoint (float &vpx, float &vpy, float &vpz) const
      {
        vpx = vpx_;
        vpy = vpy_;
        vpz = vpz_;
      }

      /** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
        * \param[out] output the resultant point indices that support the model found (inliers)
        */
      void 
      segment (PointIndices &output);

    protected:
      /** \brief A pointer to the input planar hull dataset. */
      PointCloudConstPtr planar_hull_;

      /** \brief The minimum number of points needed on the convex hull. */
      int min_pts_hull_;

      /** \brief The minimum allowed height (distance to the model) a point
        * will be considered from. 
        */
      double height_limit_min_;

      /** \brief The maximum allowed height (distance to the model) a point
        * will be considered from. 
        */
      double height_limit_max_;

      /** \brief Values describing the data acquisition viewpoint. Default: 0,0,0. */
      float vpx_, vpy_, vpz_;

      /** \brief Class getName method. */
      virtual std::string 
      getClassName () const { return ("ExtractPolygonalPrismData"); }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/extract_polygonal_prism_data.hpp>
#endif
