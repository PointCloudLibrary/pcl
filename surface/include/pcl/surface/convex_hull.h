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

#ifndef PCL_CONVEX_HULL_2D_H_
#define PCL_CONVEX_HULL_2D_H_

// PCL includes
#include <pcl/surface/reconstruction.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>

namespace pcl
{
  /** \brief Sort 2D points in a vector structure
    * \param p1 the first point
    * \param p2 the second point
    * \ingroup surface
    */
  inline bool
  comparePoints2D (const std::pair<int, Eigen::Vector4f> & p1, const std::pair<int, Eigen::Vector4f> & p2)
  {
    double angle1 = atan2 (p1.second[1], p1.second[0]) + M_PI;
    double angle2 = atan2 (p2.second[1], p2.second[0]) + M_PI;
    return (angle1 > angle2);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ConvexHull using libqhull library.
    * \author Aitor Aldoma, Alex Trevor
    * \ingroup surface
    */
  template<typename PointInT>
  class ConvexHull : public MeshConstruction<PointInT>
  {
    protected:
      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

    public:
      typedef boost::shared_ptr<ConvexHull<PointInT> > Ptr;
      typedef boost::shared_ptr<const ConvexHull<PointInT> > ConstPtr;

      using MeshConstruction<PointInT>::reconstruct;

      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \brief Empty constructor. */
      ConvexHull () : compute_area_ (false), total_area_ (0), total_volume_ (0), dimension_ (0), 
                      projection_angle_thresh_ (cos (0.174532925) ), qhull_flags ("qhull "),
                      x_axis_ (1.0, 0.0, 0.0), y_axis_ (0.0, 1.0, 0.0), z_axis_ (0.0, 0.0, 1.0)
      {
      };
      
      /** \brief Empty destructor */
      virtual ~ConvexHull () {}

      /** \brief Compute a convex hull for all points given.
        *
        * \note In 2D case (i.e. if the input points belong to one plane)
        * the \a polygons vector will have a single item, whereas in 3D
        * case it will contain one item for each hull facet.
        *
        * \param[out] points the resultant points lying on the convex hull.
        * \param[out] polygons the resultant convex hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        */
      void
      reconstruct (PointCloud &points,
                   std::vector<pcl::Vertices> &polygons);

      /** \brief Compute a convex hull for all points given.
        * \param[out] points the resultant points lying on the convex hull.
        */
      void
      reconstruct (PointCloud &points);

      /** \brief If set to true, the qhull library is called to compute the total area and volume of the convex hull.
        * NOTE: When this option is activated, the qhull library produces output to the console.
        * \param[in] value whether to compute the area and the volume, default is false
        */
      void
      setComputeAreaVolume (bool value)
      {
        compute_area_ = value;
        if (compute_area_)
          qhull_flags = std::string ("qhull FA");
        else
          qhull_flags = std::string ("qhull ");
      }

      /** \brief Returns the total area of the convex hull. */
      double
      getTotalArea () const
      {
        return (total_area_);
      }

      /** \brief Returns the total volume of the convex hull. Only valid for 3-dimensional sets.
        *  For 2D-sets volume is zero. 
        */
      double
      getTotalVolume () const
      {
        return (total_volume_);
      }

      /** \brief Sets the dimension on the input data, 2D or 3D.
        * \param[in] dimension The dimension of the input data.  If not set, this will be determined automatically.
        */
      void 
      setDimension (int dimension)
      {
        if ((dimension == 2) || (dimension == 3))
          dimension_ = dimension;
        else
          PCL_ERROR ("[pcl::%s::setDimension] Invalid input dimension specified!\n", getClassName ().c_str ());
      }

      /** \brief Returns the dimensionality (2 or 3) of the calculated hull. */
      inline int
      getDimension () const
      {
        return (dimension_);
      }

      /** \brief Retrieve the indices of the input point cloud that for the convex hull.
        *
        * \note Should only be called after reconstruction was performed.
        * \param[out] hull_point_indices The indices of the points forming the point cloud
        */
      void
      getHullPointIndices (pcl::PointIndices &hull_point_indices) const;

    protected:
      /** \brief The actual reconstruction method. 
        * 
        * \param[out] points the resultant points lying on the convex hull 
        * \param[out] polygons the resultant convex hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        * \param[in] fill_polygon_data true if polygons should be filled, false otherwise
        */
      void
      performReconstruction (PointCloud &points, 
                             std::vector<pcl::Vertices> &polygons, 
                             bool fill_polygon_data = false);
      
      /** \brief The reconstruction method for 2D data.  Does not require dimension to be set. 
        * 
        * \param[out] points the resultant points lying on the convex hull 
        * \param[out] polygons the resultant convex hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        * \param[in] fill_polygon_data true if polygons should be filled, false otherwise
        */
      void
      performReconstruction2D (PointCloud &points, 
                               std::vector<pcl::Vertices> &polygons, 
                               bool fill_polygon_data = false);
      
      /** \brief The reconstruction method for 3D data.  Does not require dimension to be set. 
        * 
        * \param[out] points the resultant points lying on the convex hull 
        * \param[out] polygons the resultant convex hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        * \param[in] fill_polygon_data true if polygons should be filled, false otherwise
        */
      void
      performReconstruction3D (PointCloud &points, 
                               std::vector<pcl::Vertices> &polygons, 
                               bool fill_polygon_data = false);
      
      /** \brief A reconstruction method that returns a polygonmesh.
        *
        * \param[out] output a PolygonMesh representing the convex hull of the input data.
        */
      virtual void
      performReconstruction (PolygonMesh &output);
      
      /** \brief A reconstruction method that returns the polygon of the convex hull.
        *
        * \param[out] polygons the polygon(s) representing the convex hull of the input data.
        */
      virtual void
      performReconstruction (std::vector<pcl::Vertices> &polygons);

      /** \brief Automatically determines the dimension of input data - 2D or 3D. */
      void 
      calculateInputDimension ();

      /** \brief Class get name method. */
      std::string
      getClassName () const
      {
        return ("ConvexHull");
      }

      /* \brief True if we should compute the area and volume of the convex hull. */
      bool compute_area_;

      /* \brief The area of the convex hull. */
      double total_area_;

      /* \brief The volume of the convex hull (only for 3D hulls, zero for 2D). */
      double total_volume_;
      
      /** \brief The dimensionality of the concave hull (2D or 3D). */
      int dimension_;

      /** \brief How close can a 2D plane's normal be to an axis to make projection problematic. */
      double projection_angle_thresh_;

      /** \brief Option flag string to be used calling qhull. */
      std::string qhull_flags;

      /* \brief x-axis - for checking valid projections. */
      const Eigen::Vector3d x_axis_;

      /* \brief y-axis - for checking valid projections. */
      const Eigen::Vector3d y_axis_;

      /* \brief z-axis - for checking valid projections. */
      const Eigen::Vector3d z_axis_;

      /* \brief vector containing the point cloud indices of the convex hull points. */
      pcl::PointIndices hull_indices_;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/convex_hull.hpp>
#endif

#endif  //#ifndef PCL_CONVEX_HULL_2D_H_
#endif
