/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: rsd.h 35534 2011-01-26 20:45:54Z marton $
 *
 */

#ifndef PCL_RSD_H_
#define PCL_RSD_H_

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief Estimate the Radius-based Surface Descriptor (RSD) for a given point based on its spatial neighborhood of 3D points with normals
    * \param surface the dataset containing the XYZ points
    * \param normals the dataset containing the surface normals at each point in the dataset
    * \param indices the neighborhood point indices in the dataset
    * \param max_dist the upper bound for the considered distance interval
    * \param nr_subdiv the number of subdivisions for the considered distance interval
    * \param plane_radius document me
    * \param radii the output point of a type that should have r_min and r_max fields
    */
  template <typename PointInT, typename PointNT, typename PointOutT> void
     computeRSD (const pcl::PointCloud<PointInT> &surface, const pcl::PointCloud<PointNT> &normals,
                 const std::vector<int> &indices, double max_dist,
                 int nr_subdiv, double plane_radius, PointOutT &radii);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b RSDEstimation estimates the Radius-based Surface Descriptor (minimal and maximal radius of the local surface's curves)
    * for a given point cloud dataset containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> Z.C. Marton , D. Pangercic , N. Blodow , J. Kleinehellefort, M. Beetz
    *      General 3D Modelling of Novel Objects from a Single View
    *      In Proceedings of the 2010 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
    *      Taipei, Taiwan, October 18-22, 2010
    * </li>
    * </ul>
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized.
    * \author Zoltan-Csaba Marton
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class RSDEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;

      /** \brief Empty constructor. */
      RSDEstimation () : nr_subdiv_ (5), plane_radius_ (0.2)
      {
        feature_name_ = "RadiusSurfaceDescriptor";
      };

      /** \brief Set the number of subdivisions for the considered distance interval.
        * \param nr_subdiv the number of subdivisions
        */
      inline void setNrSubdivisions (int nr_subdiv) { nr_subdiv_ = nr_subdiv; }

      /** \brief Get the number of subdivisions for the considered distance interval. */
      inline int getNrSubdivisions () { return (nr_subdiv_); }

      /** \brief Set the maximum radius, above which everything can be considered planar.
        * \note the order of magnitude should be around 10-20 times the search radius (0.2 works well for typical datasets).
        * \note on accurate 3D data (e.g. kinect) a search radius as low as 0.01 still gives good results.
        * \param plane_radius the new plane radius
        */
      inline void setPlaneRadius (double plane_radius) { plane_radius_ = plane_radius; }

      /** \brief Get the maximum radius, above which everything can be considered planar. */
      inline double getPlaneRadius () { return (plane_radius_); }

      /** \brief Disables the setting of the number of k nearest neighbors to use for the feature estimation. */
      inline void setKSearch (int) {  ROS_ERROR ("[pcl::%s::computeFeature] RSD does not work with k nearest neighbor search. Use setRadiusSearch() instead!", getClassName ().c_str ()); }

    protected:

      /** \brief Estimate the estimates the Radius-based Surface Descriptor (RSD) at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the RSD feature estimates (r_min and r_max values)
        */
      void computeFeature (PointCloudOut &output);

    private:

      /** \brief The upper bound for the considered distance interval. */
      // TODO double max_dist_;

      /** \brief The number of subdivisions for the considered distance interval. */
      int nr_subdiv_;

      /** \brief The maximum radius, above which everything can be considered planar. */
      double plane_radius_;
  };
}

#endif  //#ifndef PCL_RSD_H_


