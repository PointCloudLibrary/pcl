/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/pcl_config.h>
#ifdef HAVE_QHULL

#ifndef PCL_CONCAVE_HULL_H
#define PCL_CONCAVE_HULL_H

#include <pcl/surface/convex_hull.h>

namespace pcl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ConcaveHull (alpha shapes) using libqhull library.
    * \author Aitor Aldoma
    * \ingroup surface
    */
  template<typename PointInT>
  class ConcaveHull : public MeshConstruction<PointInT>
  {
    protected:
      typedef boost::shared_ptr<ConcaveHull<PointInT> > Ptr;
      typedef boost::shared_ptr<const ConcaveHull<PointInT> > ConstPtr;

      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

    public:
      using MeshConstruction<PointInT>::reconstruct;

      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \brief Empty constructor. */
      ConcaveHull () : alpha_ (0), keep_information_ (false), voronoi_centers_ (), dim_(0)
      {
      };
      
      /** \brief Empty destructor */
      virtual ~ConcaveHull () {}

      /** \brief Compute a concave hull for all points given 
        *
        * \param points the resultant points lying on the concave hull 
        * \param polygons the resultant concave hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        */
      void
      reconstruct (PointCloud &points, 
                   std::vector<pcl::Vertices> &polygons);

      /** \brief Compute a concave hull for all points given 
       * \param output the resultant concave hull vertices
       */
      void
      reconstruct (PointCloud &output);

      /** \brief Set the alpha value, which limits the size of the resultant
        * hull segments (the smaller the more detailed the hull).  
        *
        * \param alpha positive, non-zero value, defining the maximum length
        * from a vertex to the facet center (center of the voronoi cell).
        */
      inline void
      setAlpha (double alpha)
      {
        alpha_ = alpha;
      }

      /** \brief Returns the alpha parameter, see setAlpha(). */
      inline double
      getAlpha ()
      {
        return (alpha_);
      }

      /** \brief If set, the voronoi cells center will be saved in _voronoi_centers_
        * \param voronoi_centers
        */
      inline void
      setVoronoiCenters (PointCloudPtr voronoi_centers)
      {
        voronoi_centers_ = voronoi_centers;
      }

      /** \brief If keep_information_is set to true the convex hull
        * points keep other information like rgb, normals, ...
        * \param value where to keep the information or not, default is false
        */
      void
      setKeepInformation (bool value)
      {
        keep_information_ = value;
      }

      /** \brief Returns the dimensionality (2 or 3) of the calculated hull. */
      PCL_DEPRECATED ("[pcl::ConcaveHull::getDim] This method is deprecated. Please use getDimension () instead.")
      int
      getDim () const;
      
      /** \brief Returns the dimensionality (2 or 3) of the calculated hull. */
      inline int
      getDimension () const
      {
        return (dim_);
      }

      /** \brief Sets the dimension on the input data, 2D or 3D.
        * \param[in] dimension The dimension of the input data.  If not set, this will be determined automatically.
        */
      void 
      setDimension (int dimension)
      {
        if ((dimension == 2) || (dimension == 3))
          dim_ = dimension;
        else
          PCL_ERROR ("[pcl::%s::setDimension] Invalid input dimension specified!\n", getClassName ().c_str ());
      }

      /** \brief Retrieve the indices of the input point cloud that for the convex hull.
        *
        * \note Should only be called after reconstruction was performed and if the ConcaveHull is
        * set to preserve information via setKeepInformation ().
        *
        * \param[out] hull_point_indices The indices of the points forming the point cloud
        */
      void
      getHullPointIndices (pcl::PointIndices &hull_point_indices) const;

    protected:
      /** \brief Class get name method. */
      std::string
      getClassName () const
      {
        return ("ConcaveHull");
      }

    protected:
      /** \brief The actual reconstruction method.
        * 
        * \param points the resultant points lying on the concave hull 
        * \param polygons the resultant concave hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        */
      void
      performReconstruction (PointCloud &points, 
                             std::vector<pcl::Vertices> &polygons);

      virtual void
      performReconstruction (PolygonMesh &output);

      virtual void
      performReconstruction (std::vector<pcl::Vertices> &polygons);

      /** \brief The method accepts facets only if the distance from any vertex to the facet->center 
        * (center of the voronoi cell) is smaller than alpha 
        */
      double alpha_;

      /** \brief If set to true, the reconstructed point cloud describing the hull is obtained from 
        * the original input cloud by performing a nearest neighbor search from Qhull output. 
        */
      bool keep_information_;

      /** \brief the centers of the voronoi cells */
      PointCloudPtr voronoi_centers_;
      
      /** \brief the dimensionality of the concave hull */
      int dim_;

      /** \brief vector containing the point cloud indices of the convex hull points. */
      pcl::PointIndices hull_indices_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/concave_hull.hpp>
#endif

#endif  //#ifndef PCL_CONCAVE_HULL
#endif
