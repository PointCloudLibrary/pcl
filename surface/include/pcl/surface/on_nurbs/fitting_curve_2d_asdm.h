/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * 
 *
 */

#ifndef NURBS_FITTING_CURVE_2D_ASDM_H
#define NURBS_FITTING_CURVE_2D_ASDM_H

#include <pcl/pcl_exports.h>
#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Fitting a 2D B-Spline curve to 2D point-clouds using asymmetric squared-distance-minimization
      *  Based on paper: TODO
      * \author Thomas Mörwald
      * \ingroup surface
      */
    class PCL_EXPORTS FittingCurve2dASDM : public FittingCurve2dAPDM
    {
      public:
        /** \brief Constructor initializing B-Spline curve using initNurbsCurve2D(...).
          * \param[in] order the polynomial order of the B-Spline curve.
          * \param[in] data pointer to the 2D point-cloud data to be fit.
          */
        FittingCurve2dASDM (int order, NurbsDataCurve2d *data);

        /** \brief Constructor initializing with the B-Spline curve given in argument 2.
          * \param[in] data pointer to the 2D point-cloud data to be fit.
          * \param[in] nc B-Spline curve used for fitting.
          */
        FittingCurve2dASDM (NurbsDataCurve2d *data, const ON_NurbsCurve &nc);

        /** \brief Assemble the system of equations for fitting
          * - for large point-clouds this is time consuming.
          * - should be done once before refinement to initialize the starting points for point inversion. 
          */
        virtual void
        assemble (const Parameter &parameter);

        /** \brief Solve system of equations using Eigen or UmfPack (can be defined in on_nurbs.cmake),
          * and updates B-Spline curve if a solution can be obtained.
          */
        virtual double
        solve (double damp = 1.0);

        /** \brief Update curve according to the current system of equations.
          * \param[in] damp damping factor from one iteration to the other.
          */
        virtual double
        updateCurve (double damp);

      protected:

        /** \brief Add minimization constraint: point-to-surface distance (squared-distance-minimization). */
        virtual void
        addPointConstraint (const double &param, const Eigen::Vector2d &point, const Eigen::Vector2d &normal,
                            const Eigen::Vector2d &tangent, double rho, double d, double weight, unsigned &row);

        /** \brief Add minimization constraint: smoothness by control point regularisation. */
        virtual void
        addCageRegularisation (double weight, unsigned &row, const std::vector<double> &elements, double wConcav = 0.0);

        /** \brief Assemble point-to-surface constraints. */
        virtual void
        assembleInterior (double wInt, double sigma2, double rScale, unsigned &row);

        /** \brief Assemble closest points constraints. At each midpoint of the curve elements the closest data points
          * are computed and point-to-surface constraints are added. 
          */
        virtual void
        assembleClosestPoints (const std::vector<double> &elements, double weight, double sigma2, unsigned &row);

    };
  }
}

#endif
