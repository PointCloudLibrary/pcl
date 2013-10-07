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

#ifndef NURBS_FITTING_CYLINDER_H
#define NURBS_FITTING_CYLINDER_H

#include <pcl/pcl_exports.h>
#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Fitting a cylindric (dim 0 clamped, dim 1 periodic) B-Spline surface
     *  to 3D point-clouds using point-distance-minimization
     *  Based on paper: TODO
     * \author Thomas Mörwald
     * \ingroup surface     */
    class PCL_EXPORTS FittingCylinder
    {
    public:

      ON_TextLog m_out;
      ON_NurbsSurface m_nurbs;
      NurbsDataSurface *m_data;

      /** \brief Constructor initializing B-Spline surface using initNurbsPCACylinder(...).
       * \param[in] order the polynomial order of the B-Spline surface.
       * \param[in] data pointer to the 2D point-cloud data to be fit.
       * \param[in] z vector defining front face of surface.        */
      FittingCylinder (int order, NurbsDataSurface *data);

      /** \brief Constructor initializing with the B-Spline surface given in argument 2.
       * \param[in] data pointer to the 3D point-cloud data to be fit.
       * \param[in] ns B-Spline surface used for fitting.        */
      FittingCylinder (NurbsDataSurface *data, const ON_NurbsSurface &ns);

      /** \brief Refines surface by inserting a knot in the middle of each element.
       *  \param[in] dim dimension of refinement (0,1)  */
      void
      refine (int dim);

      /** \brief Refines surface by inserting a knot in the middle of the element belonging to param.
       *  \param[in] dim dimension of refinement (0,1)
       *  \param[in] param parameter defining the element to be refined. */
      void
      refine (int dim, double param);

      /** \brief Refines surface by inserting a knot in the middle of the element specified.
       *  \param[in] dim dimension of refinement (0,1)
       *  \param[in] span_index the index of the element of refinement. */
      void
      refine (int dim, unsigned span_index);

      /** \brief Assemble the system of equations for fitting
       * - for large point-clouds this is time consuming.
       * - should be done once before refinement to initialize the starting points for point inversion. */
      void
      assemble (double smoothness = 0.000001f);

      /** \brief Solve system of equations using Eigen or UmfPack (can be defined in on_nurbs.cmake),
       *  and updates B-Spline surface if a solution can be obtained. */
      void
      solve (double damp = 1.0);

      /** \brief Update surface according to the current system of equations.
       *  \param[in] damp damping factor from one iteration to the other. */
      void
      updateSurf (double damp);

      /** \brief Set parameters for inverse mapping
       *  \param[in] in_max_steps maximum number of iterations.
       *  \param[in] in_accuracy stops iteration if specified accuracy is reached. */
      void
      setInvMapParams (int in_max_steps, double invMapInt_accuracy);

      inline void
      setQuiet (bool val)
      {
        m_quiet = val;
        m_solver.setQuiet(val);
      }

      /** \brief Initializing a cylindric B-Spline surface using principal-component-analysis and eigen values */
      static ON_NurbsSurface
      initNurbsPCACylinder (int order, NurbsDataSurface *data);

      /** \brief Initializing a cylindric B-Spline surface using given axes. First axis provided becomes cylinder axis */
      static ON_NurbsSurface
        initNurbsCylinderWithAxes (int order, NurbsDataSurface *data, Eigen::Matrix3d &axes);

      /** \brief Get the elements of a cylindric B-Spline surface.*/
      static std::vector<double>
      getElementVector (const ON_NurbsSurface &nurbs, int dim);

      /** \brief Inverse mapping / point inversion: Given a point pt, this function finds the closest
       * point on the B-Spline surface using Newtons method and
       * point-distance (L2-, Euclidean norm).
       *  \param[in] nurbs the B-Spline surface.
       *  \param[in] pt the point to which the closest point on the surface will be computed.
       *  \param[in] hint the starting point in parametric domain (warning: may lead to convergence at local minima).
       *  \param[in] error the distance between the point pt and p after convergence.
       *  \param[in] p closest boundary point on surface.
       *  \param[in] tu the tangent vector at point p in u-direction.
       *  \param[in] tv the tangent vector at point p in v-direction.
       *  \param[in] maxSteps maximum number of iterations.
       *  \param[in] accuracy convergence criteria: if error is lower than accuracy the function returns
       *  \return closest point on surface in parametric domain.*/
      static Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, const Eigen::Vector2d &hint,
                      double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);

      /** \brief Given a point pt, the function finds the closest midpoint of the elements of the surface.
       *  \param[in] nurbs the B-Spline surface.
       *  \param[in] pt the point to which the closest midpoint of the elements will be computed.
       *  return closest midpoint in parametric domain. */
      static Eigen::Vector2d
      findClosestElementMidPoint (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt);

    protected:

      /** \brief Initialisation of member variables */
      void
      init ();

      /** \brief Assemble point-to-surface constraints for interior points. */
      void
      assembleInterior (double wInt, unsigned &row);

      /** \brief Add minimization constraint: point-to-surface distance (point-distance-minimization). */
      void
      addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, double weight, unsigned &row);

      /** \brief Add minimization constraint: interior smoothness by control point regularisation. */
      void
      addCageInteriorRegularisation (double weight, unsigned &row);

      /** \brief Add minimization constraint: corner smoothness by control point regularisation. */
      void
      addCageBoundaryRegularisation (double weight, int side, unsigned &row);

    private:
      NurbsSolve m_solver;
      bool m_quiet;

      int in_max_steps;
      double in_accuracy;

      // index routines
      int
      grc2gl (int I, int J)
      {
        int cp_red = (m_nurbs.m_order[1] - 2);
        int ncpj = (m_nurbs.m_cv_count[1] - 2 * cp_red);
        return ncpj * I + (J % ncpj);
      } // global row/col index to global lexicographic index
      int
      lrc2gl (int E, int F, int i, int j)
      {
        return grc2gl (E + i, F + j);
      } // local row/col index to global lexicographic index
      int
      gl2gr (int A)
      {
        return (A / m_nurbs.CVCount (1));
      } // global lexicographic in global row index
      int
      gl2gc (int A)
      {
        return (A % m_nurbs.CVCount (1));
      } // global lexicographic in global col index

    };

  }
}

#endif /* NURBS_FITTING_CYLINDER_H */
