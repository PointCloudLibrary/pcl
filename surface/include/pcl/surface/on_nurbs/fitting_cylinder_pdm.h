/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 * @author thomas.moerwald
 *
 */

#ifndef NURBS_FITTING_CYLINDER_H
#define NURBS_FITTING_CYLINDER_H

#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>

namespace pcl
{
  namespace on_nurbs
  {

    class FittingCylinder
    {
    public:

      ON_TextLog m_out;
      ON_NurbsSurface m_nurbs;
      NurbsDataSurface *m_data;

      FittingCylinder (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));
      FittingCylinder (NurbsDataSurface *data, const ON_NurbsSurface &ns);

      void
      refine (int dim);
      void
      refine (int dim, double param);
      void
      refine (int dim, unsigned span_index);

      void
      assemble (double smoothness = 0.000001f);

      void
      solve (double damp = 1.0);

      void
      updateSurf (double damp);

      void
      setInvMapParams (int in_max_steps, double invMapInt_accuracy);

      inline void
      setQuiet (bool val)
      {
        m_quiet = val;
      }

      static ON_NurbsSurface
      initNurbsPCACylinder (int order, NurbsDataSurface *data);

      void
      PrintK ();
      void
      Printf ();
      void
      Printx ();

      static std::vector<double>
      getElementVector (const ON_NurbsSurface &nurbs, int dim);

      void
      inverseMappingInterior ();
      static Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, const Eigen::Vector2d &hint,
                      double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);
      static Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, Eigen::Vector2d* phint, double &error,
                      Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);

    private:
      void
      init ();

      void
      addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, double weight, unsigned &row);

      void
      addCageInteriorRegularisation (double weight, unsigned &row);

      void
      addCageBoundaryRegularisation (double weight, int side, unsigned &row);

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
