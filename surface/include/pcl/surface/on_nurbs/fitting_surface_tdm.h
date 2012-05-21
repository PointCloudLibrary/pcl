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

#ifndef NURBS_FITTING_PATCH_TDM_H
#define NURBS_FITTING_PATCH_TDM_H

#include <pcl/surface/on_nurbs/fitting_surface_pdm.h>

namespace pcl
{
  namespace on_nurbs
  {

    class FittingPatchTDM : public FittingPatch
    {
    public:
      struct ParameterTDM : public FittingPatch::Parameter
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

      FittingPatchTDM (NurbsDataSurface *data, const ON_NurbsSurface &ns);
      FittingPatchTDM (int order, NurbsDataSurface *data, Eigen::Vector3d z = Eigen::Vector3d (0.0, 0.0, 1.0));

      virtual void
      assemble (ParameterTDM param = ParameterTDM ());

      virtual void
      solve (double damp = 1.0);

      virtual void
      updateSurf (double damp);

    protected:

      virtual void
      assembleInterior (double wInt, double wTangent, unsigned &row);
      virtual void
      assembleBoundary (double wBnd, double wTangent, unsigned &row);

      virtual void
      addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, const Eigen::Vector3d &normal,
                          const Eigen::Vector3d &tu, const Eigen::Vector3d &tv, double tangent_weight, double weight,
                          unsigned &row);

      virtual void
      addCageInteriorRegularisation (double weight, unsigned &row);
      virtual void
      addCageBoundaryRegularisation (double weight, int side, unsigned &row);
      virtual void
      addCageCornerRegularisation (double weight, unsigned &row);

      virtual void
      addInteriorRegularisation (int order, int resU, int resV, double weight, unsigned &row)
      {
      }
      virtual void
      addBoundaryRegularisation (int order, int resU, int resV, double weight, unsigned &row)
      {
      }
    };

  }
}

#endif /* NURBS_FITTING_PATCH_TDM_H */
