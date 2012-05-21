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

#ifndef NURBS_FITTING_PATCH_H
#define NURBS_FITTING_PATCH_H

#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>

namespace pcl
{
  namespace on_nurbs
  {

    class FittingPatch
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

      FittingPatch (NurbsDataSurface *data, const ON_NurbsSurface &ns);
      FittingPatch (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));

      void
      refine (int dim);

      virtual void
      assemble (Parameter param = Parameter ());

      virtual void
      solve (double damp = 1.0);

      virtual void
      updateSurf (double damp);

      void
      setInvMapParams (unsigned in_max_steps, double in_accuracy);

      inline void
      setQuiet (bool val)
      {
        m_quiet = val;
      }

      static std::vector<double>
      getElementVector (const ON_NurbsSurface &nurbs, int dim);

      static Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, const Eigen::Vector2d &hint,
                      double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);
      static Eigen::Vector2d
      inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, Eigen::Vector2d* phint, double &error,
                      Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                      double accuracy = 1e-6, bool quiet = true);
      static Eigen::Vector2d
      inverseMappingBoundary (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, double &error,
                              Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv, int maxSteps = 100,
                              double accuracy = 1e-6, bool quiet = true);
      static Eigen::Vector2d
      inverseMappingBoundary (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, int side, double hint,
                              double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv,
                              int maxSteps = 100, double accuracy = 1e-6, bool quiet = true);

      static ON_NurbsSurface
      initNurbs4Corners (int order, ON_3dPoint ll, ON_3dPoint lr, ON_3dPoint ur, ON_3dPoint ul);
      static ON_NurbsSurface
      initNurbsPCA (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));
      static ON_NurbsSurface
      initNurbsPCABoundingBox (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));

    protected:

      void
      init ();

      virtual void
      assembleInterior (double wInt, unsigned &row);
      virtual void
      assembleBoundary (double wBnd, unsigned &row);

      virtual void
      addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, double weight, unsigned &row);
      //  void addBoundaryPointConstraint(double paramU, double paramV, double weight, unsigned &row);

      virtual void
      addCageInteriorRegularisation (double weight, unsigned &row);
      virtual void
      addCageBoundaryRegularisation (double weight, int side, unsigned &row);
      virtual void
      addCageCornerRegularisation (double weight, unsigned &row);

      virtual void
      addInteriorRegularisation (int order, int resU, int resV, double weight, unsigned &row);
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
        return (int)(A / m_nurbs.CVCount (1));
      } // global lexicographic in global row index
      int
      gl2gc (int A)
      {
        return (int)(A % m_nurbs.CVCount (1));
      } // global lexicographic in global col index
    };

  }
}

#endif /* PATCHFITTING_H_ */
