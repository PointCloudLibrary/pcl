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

#pragma once

#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Fitting and optimizing multiple B-Spline surfaces
     *  to 3D point-clouds using point-distance-minimization in a single system of equations (global).
     *  Based on paper: TODO
     * \author Thomas Mörwald
     * \ingroup surface     */
    class GlobalOptimization
    {
    public:

      /** \brief Parameters for fitting */
      struct Parameter
      {
        double interior_weight;
        double interior_smoothness;

        double boundary_weight;
        double boundary_smoothness;

        double closing_weight;
        double closing_sigma;
        unsigned closing_samples;

        double common_weight;

        Parameter (double intW = 1.0, double intS = 1e-6, double bndW = 0.0, double bndS = 1e-6, double cloW = 0.0,
                   double cloSig = 0.0, unsigned cloSam = 0, double comW = 0.0) :
          interior_weight (intW), interior_smoothness (intS), boundary_weight (bndW), boundary_smoothness (bndS),
              closing_weight (cloW), closing_sigma (cloSig), closing_samples (cloSam), common_weight (comW)
        {
        }
      };

      /** \brief Constructor with a set of data and a set of B-Spline surfaces.
       * \param[in] data set of 3D point-cloud data to be fit.
       * \param[in] nurbs set of B-Spline surface used for fitting.        */
      GlobalOptimization (const std::vector<NurbsDataSurface*> &data, const std::vector<ON_NurbsSurface*> &nurbs);

      /** \brief Default virtual destructor. */
      virtual
      ~GlobalOptimization() = default;

      /** \brief Set common boundary points two NURBS should lie on
       *  \param[in] boundary vector of boundary points.
       *  \param[in] nurbs_indices vector of 2 NURBS indices sharing the boundary point. */
      void
      setCommonBoundary (const vector_vec3d &boundary, const vector_vec2i &nurbs_indices);

      /** \brief Assemble the system of equations for fitting
       * - for large point-clouds this is time consuming.
       * - should be done once before refinement to initialize the starting points for point inversion. */
      virtual void
      assemble (Parameter params = Parameter ());

      /** \brief Solve system of equations using Eigen or UmfPack (can be defined in on_nurbs.cmake),
       *  and updates B-Spline surface if a solution can be obtained. */
      virtual void
      solve (double damp = 1.0);

      /** \brief Update surface according to the current system of equations.
       *  \param[in] damp damping factor from one iteration to the other. */
      virtual void
      updateSurf (double damp);

      /** \brief Set parameters for inverse mapping
       *  \param[in] in_max_steps maximum number of iterations.
       *  \param[in] in_accuracy stops iteration if specified accuracy is reached. */
      void
      setInvMapParams (double im_max_steps, double im_accuracy);

      /** \brief Refines specified surface by inserting a knot in the middle of each element.
       *  \param[in] id the index of the surface to be refined.
       *  \param[in] dim dimension of refinement (0,1)  */
      void
      refine (unsigned id, int dim);

      inline void
      setQuiet (bool val)
      {
        m_quiet = val;
        m_solver.setQuiet (val);
      }

    protected:

      std::vector<NurbsDataSurface*> m_data;
      std::vector<ON_NurbsSurface*> m_nurbs;

      void
      assembleCommonParams (unsigned id1, double weight, unsigned &row);

      /** \brief Assemble closing-constraint of boundaries using data.boundary for getting closest points */
      virtual void
      assembleCommonBoundaries (unsigned id1, double weight, unsigned &row);

      /** \brief Assemble closing-constraint of boundaries by sampling from nurbs boundary and find closest point on closest nurbs */
      virtual void
      assembleClosingBoundaries (unsigned id, unsigned samples, double sigma, double weight, unsigned &row);

      /** \brief Assemble point-to-surface constraints for interior points. */
      virtual void
      assembleInteriorPoints (unsigned id, int ncps, double weight, unsigned &row);

      /** \brief Assemble point-to-surface constraints for boundary points. */
      virtual void
      assembleBoundaryPoints (unsigned id, int ncps, double weight, unsigned &row);

      /** \brief Assemble smoothness constraints. */
      virtual void
      assembleRegularisation (unsigned id, int ncps, double wCageRegInt, double wCageRegBnd, unsigned &row);

      /** \brief Add minimization constraint: two points in parametric domain of two surfaces should lie on each other. */
      virtual void
      addParamConstraint (const Eigen::Vector2i &id, const Eigen::Vector2d &params1, const Eigen::Vector2d &params2,
                          double weight, unsigned &row);

      /** \brief Add minimization constraint: point-to-surface distance (point-distance-minimization). */
      virtual void
      addPointConstraint (unsigned id, int ncps, const Eigen::Vector2d &params, const Eigen::Vector3d &point,
                          double weight, unsigned &row);

      /** \brief Add minimization constraint: interior smoothness by control point regularisation. */
      virtual void
      addCageInteriorRegularisation (unsigned id, int ncps, double weight, unsigned &row);

      /** \brief Add minimization constraint: boundary smoothness by control point regularisation. */
      virtual void
      addCageBoundaryRegularisation (unsigned id, int ncps, double weight, int side, unsigned &row);

      /** \brief Add minimization constraint: corner smoothness by control point regularisation. */
      virtual void
      addCageCornerRegularisation (unsigned id, int ncps, double weight, unsigned &row);

    protected:
      NurbsSolve m_solver;
      bool m_quiet;
      unsigned m_ncols, m_nrows;
      int im_max_steps;
      double im_accuracy;

      // index routines
      int
      grc2gl (const ON_NurbsSurface &nurbs, int I, int J)
      {
        return nurbs.CVCount (1) * I + J;
      } // global row/col index to global lexicographic index
      int
      lrc2gl (const ON_NurbsSurface &nurbs, int E, int F, int i, int j)
      {
        return grc2gl (nurbs, E + i, F + j);
      } // local row/col index to global lexicographic index
      int
      gl2gr (const ON_NurbsSurface &nurbs, int A)
      {
        return (static_cast<int> (A / nurbs.CVCount (1)));
      } // global lexicographic in global row index
      int
      gl2gc (const ON_NurbsSurface &nurbs, int A)
      {
        return (static_cast<int> (A % nurbs.CVCount (1)));
      } // global lexicographic in global col index
    };

  }
}
