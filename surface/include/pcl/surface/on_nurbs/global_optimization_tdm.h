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

#ifndef NURBS_OPTIMIZATION_TDM_H
#define NURBS_OPTIMIZATION_TDM_H

#include <pcl/surface/on_nurbs/global_optimization_pdm.h>

namespace pcl
{
  namespace on_nurbs
  {

    class GlobalOptimizationTDM : public GlobalOptimization
    {
    public:
      //TomGine::tgTomGineThread *m_dbgWin;

      struct ParameterTDM : public GlobalOptimization::Parameter
      {
        double interior_tangent_weight;
        double boundary_tangent_weight;
        double closing_tangent_weight;

        ParameterTDM (double intW = 1.0, double intS = 1e-6, double intTW = 0.01, double bndW = 0.0,
                      double bndS = 1e-6, double bndTW = 0.01, double cloW = 0.0, double cloSi = 0.0,
                      unsigned cloSa = 0, double cloTW = 0.0, double comW = 0.0) :
          Parameter (intW, intS, bndW, bndS, cloW, cloSi, cloSa, comW), interior_tangent_weight (intTW),
              boundary_tangent_weight (bndTW), closing_tangent_weight (cloTW)
        {
        }
        ParameterTDM (GlobalOptimization::Parameter params, double intTW = 0.01, double bndTW = 0.0, double cloTW = 0.0) :
          Parameter (params), interior_tangent_weight (intTW), boundary_tangent_weight (bndTW),
              closing_tangent_weight (cloTW)
        {
        }
      };

    public:
      GlobalOptimizationTDM (const std::vector<NurbsDataSurface*> &data, const std::vector<ON_NurbsSurface*> &nurbs);

      virtual void
      assemble (Parameter params);
      void
      assemble (ParameterTDM params = ParameterTDM ());

      virtual void
      solve (double damp = 1.0);
      virtual void
      updateSurf (double damp);

    private:
      /** @brief assemble closing of boundaries using data.boundary for getting closest points */
      virtual void
      assembleCommonBoundaries (unsigned id1, double weight, unsigned &row);

      void
      assembleCommonBoundariesTD (unsigned id1, double wTangent, double weight, unsigned &row);

      /** @brief assemble closing of boundaries by sampling from nurbs boundary and find closest point on closest nurbs */
      virtual void
      assembleClosingBoundaries (unsigned id, unsigned samples, double sigma, double weight, unsigned &row);

      void
      assembleClosingBoundariesTD (unsigned id, unsigned samples, double sigma, double wTangent, double weight,
                                   unsigned &row);

      virtual void
      assembleInteriorPoints (unsigned id, int ncps, double weight, unsigned &row);

      void
      assembleInteriorPointsTD (unsigned id, int ncps, double wTangent, double weight, unsigned &row);

      virtual void
      assembleBoundaryPoints (unsigned id, int ncps, double weight, unsigned &row);

      virtual void
      assembleRegularisation (unsigned id, int ncps, double wCageRegInt, double wCageRegBnd, unsigned &row);

      virtual void
      addParamConstraint (const Eigen::Vector2i &id, const Eigen::Vector2d &params1, const Eigen::Vector2d &params2,
                          double weight, unsigned &row);

      virtual void
      addParamConstraintTD (const Eigen::Vector2i &id, const Eigen::Vector2d &params1, const Eigen::Vector2d &params2,
                            const Eigen::Vector3d &n, const Eigen::Vector3d &tu, const Eigen::Vector3d &tv,
                            double tangent_weight, double weight, unsigned &row);

      virtual void
      addPointConstraint (unsigned id, int ncps, const Eigen::Vector2d &params, const Eigen::Vector3d &point,
                          double weight, unsigned &row);

      void
      addPointConstraintTD (unsigned id, int ncps, const Eigen::Vector2d &params, const Eigen::Vector3d &p,
                            const Eigen::Vector3d &n, const Eigen::Vector3d &tu, const Eigen::Vector3d &tv,
                            double tangent_weight, double weight, unsigned &row);

      virtual void
      addCageInteriorRegularisation (unsigned id, int ncps, double weight, unsigned &row);
      virtual void
      addCageBoundaryRegularisation (unsigned id, int ncps, double weight, int side, unsigned &row);
      virtual void
      addCageCornerRegularisation (unsigned id, int ncps, double weight, unsigned &row);

    };

  }
}
#endif

