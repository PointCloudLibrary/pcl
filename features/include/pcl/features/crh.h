/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: cvfh.h 4936 2012-03-07 11:12:45Z aaldoma $
 *
 */

#pragma once

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief CRHEstimation estimates the Camera Roll Histogram (CRH) descriptor for a given
   * point cloud dataset containing XYZ data and normals, as presented in:
   *   - CAD-Model Recognition and 6 DOF Pose Estimation
   *     A. Aldoma, N. Blodow, D. Gossow, S. Gedikli, R.B. Rusu, M. Vincze and G. Bradski
   *     ICCV 2011, 3D Representation and Recognition (3dRR11) workshop
   *     Barcelona, Spain, (2011)
   *
   * The suggested PointOutT is pcl::Histogram<90>. //dc (real) + 44 complex numbers (real, imaginary) + nyquist (real)
   *
   * \author Aitor Aldoma
   * \ingroup features
   */
  template<typename PointInT, typename PointNT, typename PointOutT = pcl::Histogram<90> >
  class CRHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<CRHEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const CRHEstimation<PointInT, PointNT, PointOutT> >;

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

      /** \brief Constructor. */
      CRHEstimation () :
        vpx_ (0), vpy_ (0), vpz_ (0), nbins_ (90)
      {
        k_ = 1;
        feature_name_ = "CRHEstimation";
      }
      ;

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

      /** \brief Get the viewpoint. 
       * \param[out] vpx the X coordinate of the viewpoint
       * \param[out] vpy the Y coordinate of the viewpoint
       * \param[out] vpz the Z coordinate of the viewpoint
       */
      inline void
      getViewPoint (float &vpx, float &vpy, float &vpz)
      {
        vpx = vpx_;
        vpy = vpy_;
        vpz = vpz_;
      }

      inline void
      setCentroid (Eigen::Vector4f & centroid)
      {
        centroid_ = centroid;
      }

    private:
      /** \brief Values describing the viewpoint ("pinhole" camera model assumed). 
       * By default, the viewpoint is set to 0,0,0.
       */
      float vpx_, vpy_, vpz_;

      /** \brief Number of bins, this should match the Output type */
      int nbins_;

      /** \brief Centroid to be used */
      Eigen::Vector4f centroid_;

      /** \brief Estimate the CRH histogram at
       * a set of points given by <setInputCloud (), setIndices ()> using the surface in
       * setSearchSurface ()
       *
       * \param[out] output the resultant point cloud with a CRH histogram
       */
      void
      computeFeature (PointCloudOut &output) override;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/crh.hpp>
#endif
