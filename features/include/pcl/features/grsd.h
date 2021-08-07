/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>

namespace pcl
{
  /** \brief @b GRSDEstimation estimates the Global Radius-based Surface Descriptor (GRSD) for a given point cloud dataset
    * containing points and normals.
    *
    * @note If you use this code in any academic work, please cite (first for the ray-casting and second for the additive version):
    *
    * <ul>
    * <li> Z.C. Marton, D. Pangercic, N. Blodow, Michael Beetz.
    *      Combined 2D-3D Categorization and Classification for Multimodal Perception Systems.
    *      In The International Journal of Robotics Research, Sage Publications
    *      pages 1378--1402, Volume 30, Number 11, September 2011.
    * </li>
    * <li> A. Kanezaki, Z.C. Marton, D. Pangercic, T. Harada, Y. Kuniyoshi, M. Beetz.
    *      Voxelized Shape and Color Histograms for RGB-D
    *      In the Workshop on Active Semantic Perception and Object Search in the Real World, in conjunction with the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
    *      San Francisco, California, September 25-30, 2011.
    * </li>
    * </ul>
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref FPFHEstimationOMP for examples on parallel implementations of the FPFH (Fast Point Feature Histogram).
    * \author Zoltan Csaba Marton
    * \ingroup features
    */
  
  template <typename PointInT, typename PointNT, typename PointOutT>
  class GRSDEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::input_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using Feature<PointInT, PointOutT>::setSearchSurface;
      //using Feature<PointInT, PointOutT>::computeFeature;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;
      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;
      using PointCloudInPtr = typename Feature<PointInT, PointOutT>::PointCloudInPtr;

      /** \brief Constructor. */
      GRSDEstimation () : additive_ (true)
      {
        feature_name_ = "GRSDEstimation";
        relative_coordinates_all_ = getAllNeighborCellIndices ();
      };

      /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the feature
        * estimation. Same value will be used for the internal voxel grid leaf size.
        * \param[in] radius the sphere radius used as the maximum distance to consider a point a neighbor
        */
      inline void 
      setRadiusSearch (double radius) { width_ = search_radius_ = radius; }

      /** \brief Get the sphere radius used for determining the neighbors. 
        * \return the sphere radius used as the maximum distance to consider a point a neighbor 
        */
      inline double
      getRadiusSearch () const { return (search_radius_); }
      
      /** \brief Get the type of the local surface based on the min and max radius computed. 
        * \return the integer that represents the type of the local surface with values as
        * Plane (1), Cylinder (2), Noise or corner (0), Sphere (3) and Edge (4) 
        */
      static inline int
      getSimpleType (float min_radius, float max_radius,
                     double min_radius_plane = 0.100,
                     double max_radius_noise = 0.015,
                     double min_radius_cylinder = 0.175,
                     double max_min_radius_diff = 0.050);

    protected:

      /** \brief Estimate the Global Radius-based Surface Descriptor (GRSD) for a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud that contains the GRSD feature
        */
      void
      computeFeature (PointCloudOut &output) override;

    private:

      /** \brief Defines if an additive feature is computed or ray-casting is used to get a more descriptive feature. */
      bool additive_;

      /** \brief Defines the voxel size to be used. */
      double width_;

      /** \brief Pre-computed the relative cell indices of all the 26 neighbors. */
      Eigen::MatrixXi relative_coordinates_all_;

  };

}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/grsd.hpp>
#endif
