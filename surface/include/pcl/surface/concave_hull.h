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
 * $Id: convex_hull.h 36101 2011-02-21 19:39:30Z aaldoma $
 *
 */

#ifndef PCL_CONCAVE_HULL_H
#define PCL_CONCAVE_HULL_H

#include "pcl/surface/convex_hull.h"

namespace pcl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ConcaveHull (alpha shapes) using libqhull library.
   * \author Aitor Aldoma
   */
  template<typename PointInT>
  class ConcaveHull : public PCLBase<PointInT>
  {
    using PCLBase<PointInT>::input_;
    using PCLBase<PointInT>::indices_;
    using PCLBase<PointInT>::initCompute;
    using PCLBase<PointInT>::deinitCompute;

    public:
      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \brief Empty constructor. */
      ConcaveHull () {};

      /** \brief Compute a concave hull for all points given 
        * \param points the resultant concave hull vertices
        * \param polygons the resultant concave hull polygons (vertex indices)
        */
      void
      reconstruct (PointCloud &points, std::vector<pcl::Vertices> &polygons);

      /** \brief Compute a concave hull for all points given 
        * \param output the resultant concave hull vertices
        */
      void
      reconstruct (PointCloud &output);

      /** \brief describe method here.. 
        * \param alpha describe alpha here...
        */
      inline void 
      setAlpha (double alpha)
      {
        alpha_ = alpha;
      }

      /** \brief describe method here. */
      inline double
      getAlpha ()
      {
        return (alpha_);
      }

      /** \brief describe method here.
        * \param voronoi_centers
        */
      inline void 
      setVoronoiCenters (PointCloud &voronoi_centers)
      {
        voronoi_centers_ = voronoi_centers;
      }

    protected:
      /** \brief Class get name method. */
      std::string
      getClassName () const { return ("ConcaveHull"); }

    private:
      /** \brief describe alpha here... */
      double alpha_;
      
      /** \brief describe voronoi_centers here.. */
      pcl::PointCloud<PointInT> voronoi_centers_;

      /** \brief describe method here...
        * \param v
        * \param center
        */
      inline double 
      computeDistVertexCenter2D (vertexT *v, coordT *center)
      {
        return (sqrt ((v->point[0] - center[0]) * (v->point[0] - center[0]) + 
                      (v->point[1] - center[1]) * (v->point[1] - center[1])));
      }

      /** \brief The actual reconstruction method. 
        * \param points the resultant concave hull vertices
        * \param polygons the resultant concave hull polygons (vertex indices)
        */
      void
      performReconstruction (PointCloud &points, std::vector<pcl::Vertices> &polygons);
  };

}

#endif  //#ifndef PCL_CONCAVE_HULL
