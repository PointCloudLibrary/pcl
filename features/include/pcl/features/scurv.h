/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2018, University of Innsbruck (Antonio J Rodríguez-Sánchez, Tomas Turecek, Alex Melniciuc)
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
#ifndef PCL_SCURV_H_
#define PCL_SCURV_H_

#include <pcl/features/feature.h>
#include <vector>

namespace pcl
{
  /** \brief @b SCurVEstimation compute and incorporate surface curvatures and distributions of local
    * surface point projections that represent flatness, concavity and convexity in a 3D object-centered
    * and view-dependent descriptor for a given point cloud dataset containing points.
    * For more information about the SCurV descriptor, see:
    * Antonio J Rodríguez-Sánchez, Sandor Szedmak and Justus Piater, "SCurV: A 3D descriptor for object classification",
    * IEEE/RSJ International Conference on Intelligento Robot Systems (IROS) 2015
    * \author Antonio J Rodríguez-Sánchez
    * \ingroup features
    */

  template <typename PointInT, typename PointNT>
  class SCurVEstimation: public FeatureFromNormals<PointInT, PointNT, SCurVSignature210>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      typedef typename Feature<PointInT, SCurVSignature210>::PointCloudIn PointCloudIn;

      typedef boost::shared_ptr<SCurVEstimation<PointInT, PointNT> > Ptr;
      typedef boost::shared_ptr<const SCurVEstimation<PointInT, PointNT> > ConstPtr;

      /** \brief Default constructor. */
      SCurVEstimation ()
      {
        feature_name_ = "SCurVEstimation";
        k_ = 19;
      }

      /** \brief Compute SCurV signature.
        * \param[out] output the resultant point cloud model dataset containing the estimated feature
        */
      void
      compute (PointCloud<SCurVSignature210> &output);

    protected:
      using Feature<PointInT, SCurVSignature210>::feature_name_;
      using Feature<PointInT, SCurVSignature210>::getClassName;
      using Feature<PointInT, SCurVSignature210>::input_;
      using Feature<PointInT, SCurVSignature210>::k_;
      using FeatureFromNormals<PointInT, PointNT, SCurVSignature210>::normals_;

      /** \brief Control point to be used for Piecewise Cubic Hermite Interpolating Polynomial (PCHIP)
        * \param[in] x the x value of control point
        * \param[in] f the functional value in control point
        * \param[in] d the derivative value in control point
        */
      struct 
      HermitePoint {
        double x;
        double f;
        double d;
      };

      /** \brief Estimate the SCurV descriptor at a set of points given by <setInputCloud (), setInputNormals ()>
        * \param[out] output the resultant point cloud model dataset that contains the SCurV feature estimates
        */
      void 
      computeFeature (pcl::PointCloud<pcl::SCurVSignature210> &output);

      /** \brief Scale normalization of cloud based on range values
        * \param[out] output the resultant scale normalized point cloud
        * \param[in] min_range the minimum value of range to be scaled to
        * \param[in] max_range the maximum value of range to be scaled to
        */
      void
      normalizeScale (pcl::PointCloud<pcl::PointNormal> &cloud, int min_range, int max_range);
      
      /** \brief return scale normalized value of x
        * \param[in] x the value to be normalized
        * \param[in] low the minimum of range
        * \param[in] hihg the maximum of range
        */
      double
      getNormalizedValue (double x, double low, double high);

      /** \brief Return the value of Piecewise Cubic Hermite Interpolating Polynomial (PCHIP) in xi between control points point1 and point2
        * \param[in] point1 the left control point
        * \param[in] point2 the right control point
        * \param[in] xi the x value in which to compute the function value
        */
      double
      getHermiteDerivativeInterpolation (HermitePoint point1, HermitePoint point2, double xi);

      /** \brief Piecewise Cubic Hermite Interpolating Polynomial (PCHIP) sign-testing routine.
        * This routine essentially computes the sign of arg1 * arg2.
	* The object is to do this without multiplying arg1 * arg2, to avoid possible over/underflow problems.
        * \param[in] arg1
        * \param[in] arg2
        */
      double
      signMultiplied (double arg1, double arg2);

      /** \brief Set derivatives of Piecewise Cubic Hermite Interpolating Polynomial (PCHIP) control points
        * \param[in] n the count of control points
        * \param[in] x the vector of x values of all control points
        * \param[in] f the vector of functional values of all control points
        * \param[out] d the vector of derivative values of all control points
        */
      void
      setSplinePchip (int n, std::vector<double> x, std::vector<double> f, std::vector<double> &d);

  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/scurv.hpp>
#endif

#endif // #
