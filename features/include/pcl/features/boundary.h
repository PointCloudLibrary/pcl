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

#ifndef PCL_BOUNDARY_H_
#define PCL_BOUNDARY_H_

#include <pcl/features/feature.h>
#include <Eigen/Geometry>

namespace pcl
{
  /** \brief @b BoundaryEstimation estimates whether a set of points is lying on surface boundaries using an angle
    * criterion. The code makes use of the estimated surface normals at each point in the input dataset.
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \a NormalEstimationOpenMP and \a NormalEstimationTBB for examples on how to extend this to parallel implementations.
    * \author Radu Bogdan Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class BoundaryEstimation: public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    public:
      /** \brief Empty constructor. */
      BoundaryEstimation () : angle_threshold_ (M_PI/2.0) 
      {
        feature_name_ = "BoundaryEstimation";
      };

     /** \brief Check whether a point is a boundary point in a planar patch of projected points given by indices.
        * \note A coordinate system u-v-n must be computed a-priori using \a getCoordinateSystemOnPlane
        * \param[in] cloud a pointer to the input point cloud
        * \param[in] q_idx the index of the query point in \a cloud
        * \param[in] indices the estimated point neighbors of the query point
        * \param[in] u the u direction
        * \param[in] v the v direction
        * \param[in] angle_threshold the threshold angle (default \f$\pi / 2.0\f$)
        */
      bool 
      isBoundaryPoint (const pcl::PointCloud<PointInT> &cloud, 
                       int q_idx, const std::vector<int> &indices, 
                       const Eigen::Vector4f &u, const Eigen::Vector4f &v, const float angle_threshold);

      /** \brief Check whether a point is a boundary point in a planar patch of projected points given by indices.
        * \note A coordinate system u-v-n must be computed a-priori using \a getCoordinateSystemOnPlane
        * \param[in] cloud a pointer to the input point cloud
        * \param[in] q_point a pointer to the querry point
        * \param[in] indices the estimated point neighbors of the query point
        * \param[in] u the u direction
        * \param[in] v the v direction
        * \param[in] angle_threshold the threshold angle (default \f$\pi / 2.0\f$)
        */
      bool 
      isBoundaryPoint (const pcl::PointCloud<PointInT> &cloud, 
                       const PointInT &q_point, 
                       const std::vector<int> &indices, 
                       const Eigen::Vector4f &u, const Eigen::Vector4f &v, const float angle_threshold);

      /** \brief Set the decision boundary (angle threshold) that marks points as boundary or regular. 
        * (default \f$\pi / 2.0\f$) 
        * \param[in] angle the angle threshold
        */
      inline void
      setAngleThreshold (float angle)
      {
        angle_threshold_ = angle;
      }

      /** \brief Get the decision boundary (angle threshold) as set by the user. */
      float
      getAngleThreshold ()
      {
        return (angle_threshold_);
      }

      /** \brief Get a u-v-n coordinate system that lies on a plane defined by its normal
        * \param[in] p_coeff the plane coefficients (containing the plane normal)
        * \param[out] u the resultant u direction
        * \param[out] v the resultant v direction
        */
      inline void 
      getCoordinateSystemOnPlane (const PointNT &p_coeff, 
                                  Eigen::Vector4f &u, Eigen::Vector4f &v)
      {
        pcl::Vector4fMapConst p_coeff_v = p_coeff.getNormalVector4fMap ();
        v = p_coeff_v.unitOrthogonal ();
        u = p_coeff_v.cross3 (v);
      }

    protected:
 
      /** \brief Estimate whether a set of points is lying on surface boundaries using an angle criterion for all points
        * given in <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains boundary point estimates
        */
      void 
      computeFeature (PointCloudOut &output);

      /** \brief The decision boundary (angle threshold) that marks points as boundary or regular. (default \f$\pi / 2.0\f$) */
      float angle_threshold_;
  };
}

#endif  //#ifndef PCL_BOUNDARY_H_


