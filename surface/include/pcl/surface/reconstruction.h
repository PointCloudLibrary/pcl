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
 * $Id$
 *
 */

#ifndef PCL_SURFACE_RECONSTRUCTION_H_
#define PCL_SURFACE_RECONSTRUCTION_H_

#include <pcl/pcl_base.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/pcl_search.h>
#include <pcl/ros/conversions.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace pcl
{
  /** \brief SurfaceReconstruction represents a base surface reconstruction
    * class. All \b surface reconstruction methods take in a point cloud and
    * generate a new surface from it, by either resampling the data or
    * generating new data altogether. These methods are thus \b not preserving
    * the topology of the original data.
    *
    * \note Reconstruction methods that always preserve the original input
    * point cloud data as the surface vertices and simply construct the mesh on
    * top should inherit from \ref MeshConstruction.
    *
    * \author Radu B. Rusu, Michael Dixon, Alex Ichim
    * \ingroup surface
    */
  template <typename PointInT>
  class SurfaceReconstruction: public PCLBase<PointInT>
  {
    public:
      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

      typedef typename pcl::search::Search<PointInT> KdTree;
      typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;

      /** \brief Constructor. */
      SurfaceReconstruction () : tree_ (), check_tree_ (true) {}

      /** \brief Destructor. */
      virtual ~SurfaceReconstruction () {}

       /** \brief Base method for surface reconstruction for all points given in
        * <setInputCloud (), setIndices ()> 
        * \param[out] output the resultant reconstructed surface model
        */
      void 
      reconstruct (pcl::PolygonMesh &output);

      /** \brief Base method for surface reconstruction for all points given in
        * <setInputCloud (), setIndices ()> 
        * \param[out] points the resultant points lying on the new surface
        * \param[out] polygons the resultant polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        */
      void 
      reconstruct (pcl::PointCloud<PointInT> &points,
                   std::vector<pcl::Vertices> &polygons);

      /** \brief Provide an optional pointer to a search object.
        * \param[in] tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree)
      {
        tree_ = tree;
      }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr 
      getSearchMethod () { return (tree_); }

    protected:
      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief A flag specifying whether or not the derived reconstruction
        * algorithm needs the search object \a tree.*/
      bool check_tree_;

      /** \brief Abstract surface reconstruction method. 
        * \param[out] output the output polygonal mesh 
        */
      virtual void 
      performReconstruction (pcl::PolygonMesh &output) = 0;

      /** \brief Abstract surface reconstruction method. 
        * \param[out] points the resultant points lying on the surface
        * \param[out] polygons the resultant polygons, as a set of vertices. The Vertices structure contains an array of point indices.
        */
      virtual void 
      performReconstruction (pcl::PointCloud<PointInT> &points, 
                             std::vector<pcl::Vertices> &polygons) = 0;

      /** \brief Abstract class get name method. */
      virtual std::string 
      getClassName () const { return (""); }
  };

  /** \brief MeshConstruction represents a base surface reconstruction
    * class. All \b mesh constructing methods that take in a point cloud and
    * generate a surface that uses the original data as vertices should inherit
    * from this class.
    *
    * \note Reconstruction methods that generate a new surface or create new
    * vertices in locations different than the input data should inherit from
    * \ref SurfaceReconstruction.
    *
    * \author Radu B. Rusu, Michael Dixon, Alex Ichim
    * \ingroup surface
    */
  template <typename PointInT>
  class MeshConstruction: public PCLBase<PointInT>
  {
    protected:
      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

    public:
      typedef typename pcl::search::Search<PointInT> KdTree;
      typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;

      /** \brief Constructor. */
      MeshConstruction () : tree_ (), check_tree_ (true) {}

      /** \brief Destructor. */
      virtual ~MeshConstruction () {}

      /** \brief Base method for surface reconstruction for all points given in
        * <setInputCloud (), setIndices ()> 
        * \param[out] output the resultant reconstructed surface model
        *
        * \note This method copies the input point cloud data from
        * PointCloud<T> to PointCloud2, and is implemented here for backwards
        * compatibility only!
        *
        */
      void 
      reconstruct (pcl::PolygonMesh &output);

      /** \brief Base method for mesh construction for all points given in
        * <setInputCloud (), setIndices ()> 
        * \param[out] polygons the resultant polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        */
      void 
      reconstruct (std::vector<pcl::Vertices> &polygons);

      /** \brief Provide an optional pointer to the search object.
        * \param[in] tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree)
      {
        tree_ = tree;
      }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr 
      getSearchMethod () { return (tree_); }

    protected:
      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief A flag specifying whether or not the derived reconstruction
        * algorithm needs the search object \a tree.*/
      bool check_tree_;

      /** \brief Abstract surface reconstruction method. 
        * \param[out] output the output polygonal mesh 
        */
      virtual void 
      performReconstruction (pcl::PolygonMesh &output) = 0;

      /** \brief Abstract surface reconstruction method. 
        * \param[out] polygons the resultant polygons, as a set of vertices. The Vertices structure contains an array of point indices.
        */
      virtual void 
      performReconstruction (std::vector<pcl::Vertices> &polygons) = 0;

      /** \brief Abstract class get name method. */
      virtual std::string 
      getClassName () const { return (""); }
  };
}

#include "pcl/surface/impl/reconstruction.hpp"

#endif  // PCL_SURFACE_RECONSTRUCTION_H_

