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

#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>

namespace pcl
{
  /** \brief FPFHEstimationOMP estimates the Fast Point Feature Histogram (FPFH) descriptor for a given point cloud
    * dataset containing points and normals, in parallel, using the OpenMP standard.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - R.B. Rusu, N. Blodow, M. Beetz.
    *     Fast Point Feature Histograms (FPFH) for 3D Registration.
    *     In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA),
    *     Kobe, Japan, May 12-17 2009.
    *   - R.B. Rusu, A. Holzbach, N. Blodow, M. Beetz.
    *     Fast Geometric Point Labeling using Conditional Random Fields.
    *     In Proceedings of the 22nd IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
    *     St. Louis, MO, USA, October 11-15 2009.
    *
    * \attention
    * The convention for FPFH features is:
    *   - if a query point's nearest neighbors cannot be estimated, the FPFH feature will be set to NaN
    *     (not a number)
    *   - it is impossible to estimate a FPFH descriptor for a point that
    *     doesn't have finite 3D coordinates. Therefore, any point that contains
    *     NaN data on x, y, or z, will have its FPFH feature property set to NaN.
    *
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class FPFHEstimationOMP : public FPFHEstimation<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<FPFHEstimationOMP<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const FPFHEstimationOMP<PointInT, PointNT, PointOutT> >;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::hist_f1_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::hist_f2_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::hist_f3_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::weightPointSPFHSignature;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param[in] nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      FPFHEstimationOMP (unsigned int nr_threads = 0) : nr_bins_f1_ (11), nr_bins_f2_ (11), nr_bins_f3_ (11)
      {
        feature_name_ = "FPFHEstimationOMP";

        setNumberOfThreads(nr_threads);
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param[in] nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      void
      setNumberOfThreads (unsigned int nr_threads = 0);

    private:
      /** \brief Estimate the Fast Point Feature Histograms (FPFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the FPFH feature estimates
        */
      void
      computeFeature (PointCloudOut &output) override;

    public:
      /** \brief The number of subdivisions for each angular feature interval. */
      int nr_bins_f1_, nr_bins_f2_, nr_bins_f3_;
    private:
      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/fpfh_omp.hpp>
#endif
