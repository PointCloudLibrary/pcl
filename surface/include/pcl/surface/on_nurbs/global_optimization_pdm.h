/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
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
 *   * Neither the name of Thomas Mörwald nor the names of its
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

#ifndef NURBS_OPTIMIZATION_H
#define NURBS_OPTIMIZATION_H

#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_solve.h>

namespace pcl
{
  namespace on_nurbs
  {

    class GlobalOptimization
    {
    public:

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

      GlobalOptimization (const std::vector<NurbsDataSurface*> &data, const std::vector<ON_NurbsSurface*> &nurbs);

      void
      setCommonBoundary (const vector_vec3d &boundary, const vector_vec2i &nurbs_indices);

      virtual void
      assemble (Parameter params = Parameter ());

      virtual void
      solve (double damp = 1.0);
      virtual void
      updateSurf (double damp);

      void
      setInvMapParams (double im_max_steps, double im_accuracy);

      void
      refine (unsigned id, int dim);

      inline void
      setQuiet (bool val)
      {
        m_quiet = val;
      }

    protected:

      void
      init ();

      std::vector<NurbsDataSurface*> m_data;
      std::vector<ON_NurbsSurface*> m_nurbs;
      std::vector<NurbsTools> m_ntools;

      /** @brief assemble closing of boundaries using data.boundary for getting closest points */
      virtual void
      assembleCommonBoundaries (unsigned id1, double weight, unsigned &row);
      /** @brief assemble closing of boundaries by sampling from nurbs boundary and find closest point on closest nurbs */
      virtual void
      assembleClosingBoundaries (unsigned id, unsigned samples, double sigma, double weight, unsigned &row);

      virtual void
      assembleInteriorPoints (unsigned id, int ncps, double weight, unsigned &row);

      virtual void
      assembleBoundaryPoints (unsigned id, int ncps, double weight, unsigned &row);

      virtual void
      assembleRegularisation (unsigned id, int ncps, double wCageRegInt, double wCageRegBnd, unsigned &row);

      virtual void
      addParamConstraint (const Eigen::Vector2i &id, const Eigen::Vector2d &params1, const Eigen::Vector2d &params2,
                          double weight, unsigned &row);
      virtual void
      addPointConstraint (unsigned id, int ncps, const Eigen::Vector2d &params, const Eigen::Vector3d &point,
                          double weight, unsigned &row);

      virtual void
      addCageInteriorRegularisation (unsigned id, int ncps, double weight, unsigned &row);
      virtual void
      addCageBoundaryRegularisation (unsigned id, int ncps, double weight, int side, unsigned &row);
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
        return (int)(A / nurbs.CVCount (1));
      } // global lexicographic in global row index
      int
      gl2gc (const ON_NurbsSurface &nurbs, int A)
      {
        return (int)(A % nurbs.CVCount (1));
      } // global lexicographic in global col index
    };

  }
}
#endif

