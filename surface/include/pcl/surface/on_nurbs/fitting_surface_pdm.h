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

#ifndef NURBS_FITTING_PATCH_H
#define NURBS_FITTING_PATCH_H

#include <pcl/pcl_exports.h>
#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>

namespace pcl
{
  namespace on_nurbs
  {
    /** \brief Fitting a B-Spline surface to 3D point-clouds using point-distance-minimization
     *  Based on paper: TODO
     * \author Thomas Mörwald
     * \ingroup surface
     */
    class PCL_EXPORTS FittingSurface
    {
    public:
      ON_NurbsSurface m_nurbs;
      NurbsDataSurface *m_data;

      class myvec
      {
      public:
        int side;
        double hint;

        myvec (int side, double hint)
        {
          this->side = side;
          this->hint = hint;
        }
      };

      /** \brief Parameters for fitting */
      struct Parameter
      {
        double interior_weight;
        double interior_smoothness;
        double interior_regularisation;

        double boundary_weight;
        double boundary_smoothness;
        double boundary_regularisation;

        unsigned regularisation_resU;
        unsigned regularisation_resV;

        Parameter (double intW = 1.0, double intS = 0.000001, double intR = 0.0, double bndW = 1.0,
                   double bndS = 0.000001, double bndR = 0.0, unsigned regU = 0, unsigned regV = 0) :
          interior_weight (intW), interior_smoothness (intS), interior_regularisation (intR), boundary_weight (bndW),
              boundary_smoothness (bndS), boundary_regularisation (bndR), regularisation_resU (regU),
              regularisation_resV (regV)
        {
        }
      };

      /** \brief Constructor initializing with the B-Spline surface given in argument 2.
       * \param[in] data pointer to the 3D point-cloud data to be fit.
       * \param[in] ns B-Spline surface used for fitting.
       */
      FittingSurface (NurbsDataSurface *data, const ON_NurbsSurface &ns);

      /** \brief Constructor initializing B-Spline surface using initNurbsPCA(...).
       * \param[in] order the polynomial order of the B-Spline surface.
       * \param[in] data pointer to the 2D point-cloud data to be fit.
       * \param[in] z vector defining front face of surface.
       */
      FittingSurface (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));

      /** \brief Refines surface by inserting a knot in the middle of each element.
       * \param[in] dim dimension of refinement (0,1)
       */
      void
      refine (int dim);

      static void
      refine (ON_NurbsSurface &nurbs, int dim);

      /** \brief Assemble the system of equations for fitting
       * - for large point-clouds this is time consuming.
       * - should be done once before refinement to initialize the starting points for point inversion.
       */
      virtual void
      assemble (Parameter param = Parameter ());

      /** \brief Solve system of equations using Eigen or UmfPack (can be defined in on_nurbs.cmake),
       * and updates B-Spline surface if a solution can be obtained.
       */
      virtual void
      solve (double damp = 1.0);

      /** \brief Update surface according to the current system of equations.
       * \param[in] damp damping factor from one iteration to the other.
       */
      virtual void
      updateSurf (double damp);

      /** \brief Set parameters for inverse mapping
       * \param[in] in_max_steps maximum number of iterations.
       * \param[in] in_accuracy stops iteration if specified accuracy is reached.
       */
      void
      setInvMapParams (unsigned in_max_steps, double in_accuracy);

      /** \brief Get the elements of a B-Spline surface.*/
      static std::vector<double>
      getElementVector (const ON_NurbsSurface &nurbs, int dim);

      /** \brief Inverse mapping / point inversion: Given a point pt, this function finds the closest
       * point on the B-Spline surface using Newtons method and point-distance (L2-, Euclidean norm).
       * \param[in] nurbs the B-Spline surface.
       * \param[in] pt the point to which the closest point on the surface will be computed.
       * \param[in] hint the starting point in parametric domain (warning: may lead to convergence at local minima).
       * \param[in] error the distance between the point pt and p after convergence.
       * \param[in] p closest point on surface.
       * \param[in] tu the tangent vector at point p in u-direction.
       * \param[in] tv the tangent vector at point p in v-direction.
       * \param[in] maxSteps maximum number of iterations.
       * \param[in] accuracy convergence criteria: if error is lower than accuracy the function returns
       * \return closest point on surface in parametric domain.
       */
      static Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, const Eigen::Vector2d &hint,
                      double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);

      static Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, const Eigen::Vector2d &hint,
                      Eigen::Vector3d &p, int maxSteps, double accuracy, bool quiet);

      /** \brief Given a point pt, the function finds the closest midpoint of the elements of the surface.
       * \param[in] nurbs the B-Spline surface.
       * \param[in] pt the point to which the closest midpoint of the elements will be computed.
       * return closest midpoint in parametric domain.
       */
      static Eigen::Vector2d
      findClosestElementMidPoint (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt);

      /** \brief Inverse mapping / point inversion: Given a point pt, this function finds the closest
       * point on the boundary of the B-Spline surface using Newtons method and point-distance (L2-, Euclidean norm).
       * \param[in] nurbs the B-Spline surface.
       * \param[in] pt the point to which the closest point on the surface will be computed.
       * \param[in] error the distance between the point pt and p after convergence.
       * \param[in] p closest boundary point on surface.
       * \param[in] tu the tangent vector at point p in u-direction.
       * \param[in] tv the tangent vector at point p in v-direction.
       * \param[in] maxSteps maximum number of iterations.
       * \param[in] accuracy convergence criteria: if error is lower than accuracy the function returns
       * \return closest point on surface in parametric domain.
       */
      static Eigen::Vector2d
      inverseMappingBoundary (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, double &error,
                              Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                              double accuracy = 1e-6, bool quiet = true);

      /** \brief Inverse mapping / point inversion: Given a point pt, this function finds the closest
       * point on one side of the boundary of the B-Spline surface using Newtons method and
       * point-distance (L2-, Euclidean norm).
       * \param[in] nurbs the B-Spline surface.
       * \param[in] pt the point to which the closest point on the surface will be computed.
       * \param[in] side the side of the boundary (NORTH, SOUTH, EAST, WEST)
       * \param[in] hint the starting point in parametric domain (warning: may lead to convergence at local minima).
       * \param[in] error the distance between the point pt and p after convergence.
       * \param[in] p closest boundary point on surface.
       * \param[in] tu the tangent vector at point p in u-direction.
       * \param[in] tv the tangent vector at point p in v-direction.
       * \param[in] maxSteps maximum number of iterations.
       * \param[in] accuracy convergence criteria: if error is lower than accuracy the function returns
       * \return closest point on surface in parametric domain.
       */
      static Eigen::Vector2d
      inverseMappingBoundary (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, int side, double hint,
                              double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv,
                              int maxSteps = 100, double accuracy = 1e-6, bool quiet = true);

      /** \brief Initializing a B-Spline surface using 4 corners */
      static ON_NurbsSurface
      initNurbs4Corners (int order, ON_3dPoint ll, ON_3dPoint lr, ON_3dPoint ur, ON_3dPoint ul);

      /** \brief Initializing a B-Spline surface using principal-component-analysis and eigen values */
      static ON_NurbsSurface
      initNurbsPCA (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));

      /** \brief Initializing a B-Spline surface using principal-component-analysis and bounding box of points */
      static ON_NurbsSurface
      initNurbsPCABoundingBox (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));

      /** \brief Enable/Disable debug outputs in console. */
      inline void
      setQuiet (bool val)
      {
        m_quiet = val;
        m_solver.setQuiet (val);
      }

    protected:

      /** \brief Initialisation of member variables */
      void
      init ();

      /** \brief Assemble point-to-surface constraints for interior points. */
      virtual void
      assembleInterior (double wInt, unsigned &row);

      /** \brief Assemble point-to-surface constraints for boundary points. */
      virtual void
      assembleBoundary (double wBnd, unsigned &row);

      /** \brief Add minimization constraint: point-to-surface distance (point-distance-minimization). */
      virtual void
      addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, double weight, unsigned &row);
      //  void addBoundaryPointConstraint(double paramU, double paramV, double weight, unsigned &row);

      /** \brief Add minimization constraint: interior smoothness by control point regularisation. */
      virtual void
      addCageInteriorRegularisation (double weight, unsigned &row);

      /** \brief Add minimization constraint: boundary smoothness by control point regularisation. */
      virtual void
      addCageBoundaryRegularisation (double weight, int side, unsigned &row);

      /** \brief Add minimization constraint: corner smoothness by control point regularisation. */
      virtual void
      addCageCornerRegularisation (double weight, unsigned &row);

      /** \brief Add minimization constraint: interior smoothness by derivatives regularisation. */
      virtual void
      addInteriorRegularisation (int order, int resU, int resV, double weight, unsigned &row);

      /** \brief Add minimization constraint: boundary smoothness by derivatives regularisation. */
      virtual void
      addBoundaryRegularisation (int order, int resU, int resV, double weight, unsigned &row);

      NurbsSolve m_solver;

      bool m_quiet;

      std::vector<double> m_elementsU;
      std::vector<double> m_elementsV;

      double m_minU;
      double m_minV;
      double m_maxU;
      double m_maxV;

      int in_max_steps;
      double in_accuracy;

      // index routines
      int
      grc2gl (int I, int J)
      {
        return m_nurbs.CVCount (1) * I + J;
      } // global row/col index to global lexicographic index
      int
      lrc2gl (int E, int F, int i, int j)
      {
        return grc2gl (E + i, F + j);
      } // local row/col index to global lexicographic index
      int
      gl2gr (int A)
      {
        return (static_cast<int> (A / m_nurbs.CVCount (1)));
      } // global lexicographic in global row index
      int
      gl2gc (int A)
      {
        return (static_cast<int> (A % m_nurbs.CVCount (1)));
      } // global lexicographic in global col index
    };

  } // namespace on_nurbs
} // namespace pcl

#endif    // PATCHFITTING_H_
