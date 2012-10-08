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

#ifndef NURBS_FITTING_PATCH_TDM_H
#define NURBS_FITTING_PATCH_TDM_H

#include <pcl/surface/on_nurbs/fitting_surface_pdm.h>

namespace pcl
{
  namespace on_nurbs
  {
    /** \brief Fitting a B-Spline surface to 3D point-clouds using tangent-distance-minimization
     *  Based on paper: TODO
     * \author Thomas Mörwald
     * \ingroup surface     */
    class FittingSurfaceTDM : public FittingSurface
    {
    public:

      /** \brief Parameters with TDM extensions for fitting */
      struct ParameterTDM : public FittingSurface::Parameter
      {
        double interior_tangent_weight;
        double boundary_tangent_weight;

        ParameterTDM (double intW = 1.0, double intS = 0.000001, double intR = 0.0, double intTW = 0.1,
                      double bndW = 1.0, double bndS = 0.000001, double bndR = 0.0, double bndTW = 0.1,
                      unsigned regU = 0, unsigned regV = 0) :
          Parameter (intW, intS, intR, bndW, bndS, bndR, regU, regV), interior_tangent_weight (intTW),
              boundary_tangent_weight (bndTW)
        {
        }
      };

      /** \brief Constructor initializing with the B-Spline surface given in argument 2.
       * \param[in] data pointer to the 3D point-cloud data to be fit.
       * \param[in] ns B-Spline surface used for fitting.        */
      FittingSurfaceTDM (NurbsDataSurface *data, const ON_NurbsSurface &ns);

      /** \brief Constructor initializing B-Spline surface using initNurbsPCA(...).
       * \param[in] order the polynomial order of the B-Spline surface.
       * \param[in] data pointer to the 2D point-cloud data to be fit.
       * \param[in] z vector defining front face of surface.        */
      FittingSurfaceTDM (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));

      /** \brief Assemble the system of equations for fitting
       * - for large point-clouds this is time consuming.
       * - should be done once before refinement to initialize the starting points for point inversion. */
      virtual void
      assemble (ParameterTDM param = ParameterTDM ());

      /** \brief Solve system of equations using Eigen or UmfPack (can be defined in on_nurbs.cmake),
       *  and updates B-Spline surface if a solution can be obtained. */
      virtual void
      solve (double damp = 1.0);

      /** \brief Update surface according to the current system of equations.
       *  \param[in] damp damping factor from one iteration to the other. */
      virtual void
      updateSurf (double damp);

    protected:

      /** \brief Assemble point-to-surface constraints for interior points. */
      virtual void
      assembleInterior (double wInt, double wTangent, unsigned &row);

      /** \brief Assemble point-to-surface constraints for boundary points. */
      virtual void
      assembleBoundary (double wBnd, double wTangent, unsigned &row);

      /** \brief Add minimization constraint: point-to-surface distance (point-distance-minimization). */
      virtual void
      addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, const Eigen::Vector3d &normal,
                          const Eigen::Vector3d &tu, const Eigen::Vector3d &tv, double tangent_weight, double weight,
                          unsigned &row);

      /** \brief Add minimization constraint: interior smoothness by control point regularisation. */
      virtual void
      addCageInteriorRegularisation (double weight, unsigned &row);

      /** \brief Add minimization constraint: boundary smoothness by control point regularisation. */
      virtual void
      addCageBoundaryRegularisation (double weight, int side, unsigned &row);

      /** \brief Add minimization constraint: corner smoothness by control point regularisation. */
      virtual void
      addCageCornerRegularisation (double weight, unsigned &row);

      virtual void
      addInteriorRegularisation (int, int, int, double, unsigned &)
      {
      }
      virtual void
      addBoundaryRegularisation (int, int, int, double, unsigned &)
      {
      }
    };

  }
}

#endif /* NURBS_FITTING_PATCH_TDM_H */
