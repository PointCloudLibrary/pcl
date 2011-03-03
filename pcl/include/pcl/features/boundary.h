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
 * $Id: boundary.h 35826 2011-02-08 00:59:32Z rusu $
 *
 */

#ifndef PCL_BOUNDARY_H_
#define PCL_BOUNDARY_H_

#include <pcl/features/feature.h>
#include <Eigen/Geometry>

namespace pcl
{
  /** \brief Compute the angle in the [ 0, 2*PI ) interval of a point (direction) with a reference (0, 0) in 2D.
    * \param point a 2D point
    */
  inline float 
  getAngle2D (const float point[2])
  {
    float rad = atan2(point[1], point[0]);
    if (rad < 0)
      rad += 2 * M_PI;
    return (rad);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b BoundaryEstimation estimates whether a set of points is lying on surface boundaries using an angle
    * criterion. The code makes use of the estimated surface normals at each point in the input dataset.
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \a NormalEstimationOpenMP and \a NormalEstimationTBB for examples on how to extend this to parallel implementations.
    * \author Radu Bogdan Rusu
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

      /** \brief Get a u-v-n coordinate system that lies on a plane defined by its normal
        * \param p_coeff the plane coefficients (containing the plane normal)
        * \param u the resultant u direction
        * \param v the resultant v direction
        */
      inline void 
      getCoordinateSystemOnPlane (const PointNT &p_coeff, 
                                  Eigen::Vector3f &u, Eigen::Vector3f &v)
      {
        pcl::Vector3fMapConst p_coeff_v = p_coeff.getNormalVector3fMap ();
        v = p_coeff_v.unitOrthogonal ();
        u = p_coeff_v.cross (v);
      }

      /** \brief Check whether a point is a boundary point in a planar patch of projected points given by indices.
        * \note A coordinate system u-v-n must be computed a-priori using \a getCoordinateSystemOnPlane
        * \param cloud a pointer to the input point cloud
        * \param q_idx the index of the query point in \a cloud
        * \param indices the estimated point neighbors of the query point
        * \param u the u direction
        * \param v the v direction
        * \param angle_threshold the threshold angle (default \f$\pi / 2.0\f$)
        */
      bool isBoundaryPoint (const pcl::PointCloud<PointInT> &cloud, int q_idx, const std::vector<int> &indices, const Eigen::Vector3f &u, const Eigen::Vector3f &v, float angle_threshold);

      /** \brief Check whether a point is a boundary point in a planar patch of projected points given by indices.
        * \note A coordinate system u-v-n must be computed a-priori using \a getCoordinateSystemOnPlane
        * \param cloud a pointer to the input point cloud
        * \param q_point a pointer to the querry point
        * \param indices the estimated point neighbors of the query point
        * \param u the u direction
        * \param v the v direction
        * \param angle_threshold the threshold angle (default \f$\pi / 2.0\f$)
        */
      bool isBoundaryPoint (const pcl::PointCloud<PointInT> &cloud, const PointInT &q_point, const std::vector<int> &indices, const Eigen::Vector3f &u, const Eigen::Vector3f &v, float angle_threshold);

      /** \brief The decision boundary (angle threshold) that marks points as boundary or regular. (default \f$\pi / 2.0\f$) */
      float angle_threshold_;

    protected:

      /** \brief Estimate whether a set of points is lying on surface boundaries using an angle criterion for all points
        * given in <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains boundary point estimates
        */
      void computeFeature (PointCloudOut &output);
  };
}

#endif  //#ifndef PCL_BOUNDARY_H_


