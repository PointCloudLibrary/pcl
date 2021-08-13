/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id$
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief Transform a list of 2D matrices into a point cloud containing the values in a vector (Histogram<N>).
    * Can be used to transform the 2D histograms obtained in \ref RSDEstimation into a point cloud.
    * @note The template parameter N should be (greater or) equal to the product of the number of rows and columns.
    * \param[in] histograms2D the list of neighborhood 2D histograms
    * \param[out] histogramsPC the dataset containing the linearized matrices
    * \ingroup features
    */
  template <int N> void
  getFeaturePointCloud (const std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> > &histograms2D, PointCloud<Histogram<N> > &histogramsPC)
  {
    histogramsPC.resize (histograms2D.size ());
    histogramsPC.width    = histograms2D.size ();
    histogramsPC.height   = 1;
    histogramsPC.is_dense = true;

    const int rows  = histograms2D.at(0).rows();
    const int cols = histograms2D.at(0).cols();

    typename PointCloud<Histogram<N> >::VectorType::iterator it = histogramsPC.begin ();
    for (const Eigen::MatrixXf& h : histograms2D)
    {
      Eigen::Map<Eigen::MatrixXf> histogram (&(it->histogram[0]), rows, cols);
      histogram = h;
      ++it;
    }
  }

  /** \brief Estimate the Radius-based Surface Descriptor (RSD) for a given point based on its spatial neighborhood of 3D points with normals
    * \param[in] surface the dataset containing the XYZ points
    * \param[in] normals the dataset containing the surface normals at each point in the dataset
    * \param[in] indices the neighborhood point indices in the dataset (first point is used as the reference)
    * \param[in] max_dist the upper bound for the considered distance interval
    * \param[in] nr_subdiv the number of subdivisions for the considered distance interval
    * \param[in] plane_radius maximum radius, above which everything can be considered planar
    * \param[in] radii the output point of a type that should have r_min and r_max fields
    * \param[in] compute_histogram if not false, the full neighborhood histogram is provided, usable as a point signature
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT> Eigen::MatrixXf
  computeRSD (const pcl::PointCloud<PointInT> &surface, const pcl::PointCloud<PointNT> &normals,
              const pcl::Indices &indices, double max_dist,
              int nr_subdiv, double plane_radius, PointOutT &radii, bool compute_histogram = false);

  /** \brief Estimate the Radius-based Surface Descriptor (RSD) for a given point based on its spatial neighborhood of 3D points with normals
    * \param[in] normals the dataset containing the surface normals at each point in the dataset
    * \param[in] indices the neighborhood point indices in the dataset (first point is used as the reference)
    * \param[in] sqr_dists the squared distances from the first to all points in the neighborhood
    * \param[in] max_dist the upper bound for the considered distance interval
    * \param[in] nr_subdiv the number of subdivisions for the considered distance interval
    * \param[in] plane_radius maximum radius, above which everything can be considered planar
    * \param[in] radii the output point of a type that should have r_min and r_max fields
    * \param[in] compute_histogram if not false, the full neighborhood histogram is provided, usable as a point signature
    * \ingroup features
    */
  template <typename PointNT, typename PointOutT> Eigen::MatrixXf
  computeRSD (const pcl::PointCloud<PointNT> &normals,
              const pcl::Indices &indices, const std::vector<float> &sqr_dists, double max_dist,
              int nr_subdiv, double plane_radius, PointOutT &radii, bool compute_histogram = false);

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
    * <li> Z.C. Marton, D. Pangercic, N. Blodow, Michael Beetz.
    *      Combined 2D-3D Categorization and Classification for Multimodal Perception Systems.
    *      In The International Journal of Robotics Research, Sage Publications
    *      pages 1378--1402, Volume 30, Number 11, September 2011.
    * </li>
    * </ul>
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized.
    * \author Zoltan-Csaba Marton
    * \ingroup features
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

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;
      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;

      using Ptr = shared_ptr<RSDEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const RSDEstimation<PointInT, PointNT, PointOutT> >;


      /** \brief Empty constructor. */
      RSDEstimation () : nr_subdiv_ (5), plane_radius_ (0.2), save_histograms_ (false)
      {
        feature_name_ = "RadiusSurfaceDescriptor";
      };

      /** \brief Set the number of subdivisions for the considered distance interval.
        * \param[in] nr_subdiv the number of subdivisions
        */
      inline void 
      setNrSubdivisions (int nr_subdiv) { nr_subdiv_ = nr_subdiv; }

      /** \brief Get the number of subdivisions for the considered distance interval. 
        * \return the number of subdivisions
	*/
      inline int 
      getNrSubdivisions () const { return (nr_subdiv_); }

      /** \brief Set the maximum radius, above which everything can be considered planar.
        * \note the order of magnitude should be around 10-20 times the search radius (0.2 works well for typical datasets).
        * \note on accurate 3D data (e.g. openni sernsors) a search radius as low as 0.01 still gives good results.
        * \param[in] plane_radius the new plane radius
        */
      inline void 
      setPlaneRadius (double plane_radius) { plane_radius_ = plane_radius; }

      /** \brief Get the maximum radius, above which everything can be considered planar.
        * \return the plane_radius used
	*/
      inline double 
      getPlaneRadius () const { return (plane_radius_); }

      /** \brief Disables the setting of the number of k nearest neighbors to use for the feature estimation. */
      inline void 
      setKSearch (int) 
      {
        PCL_ERROR ("[pcl::%s::setKSearch] RSD does not work with k nearest neighbor search. Use setRadiusSearch() instead!\n", getClassName ().c_str ());
      }

      /** \brief Set whether the full distance-angle histograms should be saved.
        * @note Obtain the list of histograms by getHistograms ()
        * \param[in] save set to true if histograms should be saved
        */
      inline void
      setSaveHistograms (bool save) { save_histograms_ = save; }

      /** \brief Returns whether the full distance-angle histograms are being saved. 
        * \return true if the histograms are being be saved
	*/
      inline bool
      getSaveHistograms () const { return (save_histograms_); }

      /** \brief Returns a pointer to the list of full distance-angle histograms for all points.
        * \return the histogram being saved when computing RSD
	*/
      inline shared_ptr<std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> > >
      getHistograms () const { return (histograms_); }

    protected:

      /** \brief Estimate the estimates the Radius-based Surface Descriptor (RSD) at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the RSD feature estimates (r_min and r_max values)
        */
      void 
      computeFeature (PointCloudOut &output) override;

      /** \brief The list of full distance-angle histograms for all points. */
      shared_ptr<std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> > > histograms_;

    private:
      /** \brief The number of subdivisions for the considered distance interval. */
      int nr_subdiv_;

      /** \brief The maximum radius, above which everything can be considered planar. */
      double plane_radius_;

      /** \brief Signals whether the full distance-angle histograms are being saved. */
      bool save_histograms_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/rsd.hpp>
#endif
