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
 * $Id: fpfh_omp.h 34737 2010-12-14 09:07:35Z rusu $
 *
 */

#ifndef PCL_FPFH_OMP_H_
#define PCL_FPFH_OMP_H_

#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b FPFHEstimationOMP estimates the Fast Point Feature Histogram (FPFH) descriptor for a given point cloud
    * dataset containing points and normals, in parallel, using the OpenMP standard.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> R.B. Rusu, N. Blodow, M. Beetz.
    *      Fast Point Feature Histograms (FPFH) for 3D Registration.
    *      In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA),
    *      Kobe, Japan, May 12-17 2009.
    * </li>
    * <li> R.B. Rusu, A. Holzbach, N. Blodow, M. Beetz.
    *      Fast Geometric Point Labeling using Conditional Random Fields.
    *      In Proceedings of the 22nd IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
    *      St. Louis, MO, USA, October 11-15 2009.
    * </li>
    * </ul>
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class FPFHEstimationOMP : public FPFHEstimation<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::hist_f1_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::hist_f2_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::hist_f3_;
      using FPFHEstimation<PointInT, PointNT, PointOutT>::weightPointSPFHSignature;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Empty constructor. */
      FPFHEstimationOMP () : nr_bins_f1_ (11), nr_bins_f2_ (11), nr_bins_f3_ (11), threads_ (-1) 
      {
        feature_name_ = "FPFHEstimationOMP";
      };

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      FPFHEstimationOMP (int nr_threads) : nr_bins_f1_ (11), nr_bins_f2_ (11), nr_bins_f3_ (11)
      {
        setNumberOfThreads (nr_threads);
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      inline void setNumberOfThreads (int nr_threads) { threads_ = nr_threads; }

    private:

      /** \brief Estimate the Fast Point Feature Histograms (FPFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the FPFH feature estimates
        */
      void computeFeature (PointCloudOut &output);

    public:
      /** \brief The number of subdivisions for each angular feature interval. */
      int nr_bins_f1_, nr_bins_f2_, nr_bins_f3_;
    private:
      /** \brief The number of threads the scheduler should use. */
      int threads_;
  };
}

#endif  //#ifndef PCL_FPFH_OMP_H_


